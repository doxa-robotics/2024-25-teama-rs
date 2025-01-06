use alloc::sync::Arc;

use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::{smart::motor::MotorError, PortError},
    prelude::{sleep, spawn, AdiAnalogIn, BrakeMode, Direction, Motor},
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
    Stop,
}

const RING_THRESHOLD: u16 = 2500;
const TIMEOUT: u128 = 3000;
const ARM_INTAKE_SETTLING_TIME: u128 = 600;

#[derive(Debug)]
pub struct IntakeInner {
    motor: Motor,
    line_tracker: AdiAnalogIn,
    state: IntakeState,
}

#[derive(Debug, Snafu)]
pub enum IntakeError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
    #[snafu(display("line tracker error: {}", source))]
    LineTrackerPortError { source: PortError },
}

impl IntakeInner {
    async fn update(&mut self) -> Result<(), IntakeError> {
        match self.state {
            IntakeState::Forward => {
                self.motor
                    .set_voltage(self.motor.max_voltage())
                    .context(MotorSnafu)?;
            }
            IntakeState::Reverse => {
                self.motor
                    .set_voltage(-self.motor.max_voltage())
                    .context(MotorSnafu)?;
            }
            IntakeState::PartialIntake { start }
            | IntakeState::ArmIntake {
                start,
                settling_start: _,
            } => {
                if let IntakeState::ArmIntake {
                    start: _,
                    settling_start: Some(settling_start),
                } = self.state
                {
                    if settling_start.elapsed().as_millis() > ARM_INTAKE_SETTLING_TIME {
                        self.state = IntakeState::Stop;
                    }
                    return Ok(());
                }
                if start.elapsed().as_millis() > TIMEOUT {
                    error!("Partially intaking timeout");
                    self.state = IntakeState::Stop;
                } else {
                    self.motor
                        .set_voltage(self.motor.max_voltage())
                        .context(MotorSnafu)?;
                    let current_value = self.line_tracker.value().context(LineTrackerPortSnafu)?;
                    if current_value < RING_THRESHOLD {
                        match self.state {
                            IntakeState::PartialIntake { start: _ } => {
                                self.state = IntakeState::Stop;
                            }
                            IntakeState::ArmIntake {
                                start,
                                settling_start: _,
                            } => {
                                self.state = IntakeState::ArmIntake {
                                    start,
                                    settling_start: Some(Instant::now()),
                                };
                            }
                            _ => unreachable!(),
                        }
                    }
                }
            }
            IntakeState::Stop => {
                let current_value = self.line_tracker.value().context(LineTrackerPortSnafu)?;
                if current_value < RING_THRESHOLD {
                    self.motor.brake(BrakeMode::Hold).context(MotorSnafu)?;
                } else {
                    self.motor.set_voltage(0.0).context(MotorSnafu)?;
                }
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct Intake(Arc<Mutex<IntakeInner>>);

impl Intake {
    pub fn new(motor: Motor, line_tracker: AdiAnalogIn) -> Self {
        Self(Arc::new(Mutex::new(IntakeInner {
            motor,
            line_tracker,
            state: IntakeState::Stop,
        })))
    }

    pub fn temperature(&self) -> f64 {
        match self.0.try_lock() {
            Some(inner) => inner.motor.temperature().unwrap_or(0.0),
            None => 0.0,
        }
    }

    pub fn spawn_update(&self) {
        let inner = self.0.clone();
        spawn(async move {
            loop {
                let mut inner = inner.lock().await;
                if let Err(err) = inner.update().await {
                    error!("intake update error: {}", err);
                }
                drop(inner);

                sleep(super::SUBSYSTEM_UPDATE_PERIOD).await;
            }
        })
        .detach();
    }

    pub async fn stop(&self) {
        let mut inner = self.0.lock().await;
        inner.state = IntakeState::Stop;
    }

    pub async fn stop_if_running(&self) {
        let mut inner = self.0.lock().await;
        if matches!(inner.state, IntakeState::Forward | IntakeState::Reverse) {
            inner.state = IntakeState::Stop;
        }
    }

    pub async fn partial_intake(&self) {
        let mut inner = self.0.lock().await;
        inner.state = IntakeState::PartialIntake {
            start: Instant::now(),
        };
    }

    pub async fn arm_intake(&self) {
        let mut inner = self.0.lock().await;
        inner.state = IntakeState::ArmIntake {
            start: Instant::now(),
            settling_start: None,
        };
    }

    pub async fn run(&self, direction: Direction) {
        let mut inner = self.0.lock().await;
        inner.state = match direction {
            Direction::Forward => IntakeState::Forward,
            Direction::Reverse => IntakeState::Reverse,
        };
    }

    pub async fn state(&self) -> IntakeState {
        self.0.lock().await.state
    }

    pub async fn set_state(&self, state: IntakeState) {
        self.0.lock().await.state = state;
    }
}
