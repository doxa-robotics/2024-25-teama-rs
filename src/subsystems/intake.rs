use alloc::sync::Arc;

use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::{smart::motor::MotorError, PortError},
    prelude::{sleep, spawn, AdiLineTracker, BrakeMode, Direction, Motor},
};

#[derive(Debug, Clone, Copy)]
pub enum IntakeState {
    Forward,
    Reverse,
    PartialIntake {
        start: Instant,
    },
    ArmIntake {
        start: Instant,
        settling_start: Option<Instant>,
    },
    ForwardUntil {
        end: Instant,
    },
    Stopped,
}

const RING_THRESHOLD: f64 = 0.4;
const TIMEOUT: u128 = 3000;
const ARM_INTAKE_SETTLING_TIME: u128 = 600;

pub struct Intake {
    motor: Arc<Mutex<Motor>>,
    line_tracker: Arc<Mutex<AdiLineTracker>>,
    state: Arc<Mutex<IntakeState>>,
}

#[derive(Debug, Snafu)]
pub enum IntakeError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
    #[snafu(display("line tracker error: {}", source))]
    LineTrackerPortError { source: PortError },
}

impl Intake {
    pub fn new(motor: Motor, line_tracker: AdiLineTracker) -> Self {
        Self {
            motor: Arc::new(Mutex::new(motor)),
            line_tracker: Arc::new(Mutex::new(line_tracker)),
            state: Arc::new(Mutex::new(IntakeState::Stopped)),
        }
    }

    pub fn temperature(&self) -> f64 {
        self.motor
            .try_lock()
            .map(|m| m.temperature().unwrap_or(0.0))
            .unwrap_or(0.0)
    }

    pub async fn state(&self) -> IntakeState {
        (*self.state.lock().await).clone()
    }

    pub async fn set_state(&mut self, state: IntakeState) {
        *self.state.lock().await = state;
    }

    pub async fn stop(&mut self) {
        *self.state.lock().await = IntakeState::Stopped;
    }

    pub async fn partial_intake(&mut self) {
        *self.state.lock().await = IntakeState::PartialIntake {
            start: Instant::now(),
        };
    }

    pub async fn arm_intake(&mut self) {
        *self.state.lock().await = IntakeState::ArmIntake {
            start: Instant::now(),
            settling_start: None,
        };
    }

    pub async fn run(&mut self, direction: Direction) {
        *self.state.lock().await = match direction {
            Direction::Forward => IntakeState::Forward,
            Direction::Reverse => IntakeState::Reverse,
        };
    }

    pub async fn run_until(&mut self, end: Instant) {
        *self.state.lock().await = IntakeState::ForwardUntil { end };
    }

    fn update(
        state: &mut IntakeState,
        motor: &mut Motor,
        line_tracker: &mut AdiLineTracker,
    ) -> Result<(), IntakeError> {
        let max_voltage = motor.max_voltage();
        match state {
            IntakeState::Forward => {
                motor.set_voltage(max_voltage).context(MotorSnafu)?;
            }
            IntakeState::Reverse => {
                motor.set_voltage(-max_voltage).context(MotorSnafu)?;
            }
            IntakeState::PartialIntake { start } => {
                if start.elapsed().as_millis() > TIMEOUT {
                    error!("Partially intaking timeout");
                    *state = IntakeState::Stopped;
                } else {
                    motor.set_voltage(max_voltage).context(MotorSnafu)?;
                    let current_value =
                        line_tracker.reflectivity().context(LineTrackerPortSnafu)?;
                    if current_value < RING_THRESHOLD {
                        *state = IntakeState::Stopped;
                    }
                }
            }
            IntakeState::ArmIntake {
                start,
                settling_start,
            } => {
                if let Some(settling_start) = settling_start {
                    if settling_start.elapsed().as_millis() > ARM_INTAKE_SETTLING_TIME {
                        *state = IntakeState::Stopped;
                    }
                } else if start.elapsed().as_millis() > TIMEOUT {
                    error!("Arm intaking timeout");
                    *state = IntakeState::Stopped;
                } else {
                    motor.set_voltage(max_voltage).context(MotorSnafu)?;
                    let current_value =
                        line_tracker.reflectivity().context(LineTrackerPortSnafu)?;
                    if current_value < RING_THRESHOLD {
                        *state = IntakeState::ArmIntake {
                            start: *start,
                            settling_start: Some(Instant::now()),
                        };
                    }
                }
            }
            IntakeState::Stopped => {
                let current_value = line_tracker.reflectivity().context(LineTrackerPortSnafu)?;
                if current_value < RING_THRESHOLD {
                    motor.brake(BrakeMode::Hold).context(MotorSnafu)?;
                } else {
                    motor.set_voltage(0.0).context(MotorSnafu)?;
                }
            }
            IntakeState::ForwardUntil { end } => {
                if Instant::now() > *end {
                    *state = IntakeState::Stopped;
                } else {
                    motor.set_voltage(max_voltage).context(MotorSnafu)?;
                }
            }
        }
        Ok(())
    }

    pub fn spawn_update_thread(&self) {
        let motor = self.motor.clone();
        let line_tracker = self.line_tracker.clone();
        let state = self.state.clone();
        spawn(async move {
            loop {
                let mut motor = motor.lock().await;
                let mut line_tracker = line_tracker.lock().await;
                let mut state = state.lock().await;
                if let Err(err) = Intake::update(&mut state, &mut motor, &mut line_tracker) {
                    error!("Intake update error: {}", err);
                }
                sleep(super::SUBSYSTEM_UPDATE_PERIOD).await;
            }
        });
    }
}
