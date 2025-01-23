use alloc::sync::Arc;
use core::time::Duration;

use log::{debug, error};
use snafu::{ResultExt, Snafu};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::{smart::motor::MotorError, PortError},
    prelude::{sleep, spawn, AdiLineTracker, BrakeMode, Direction, Motor},
};

#[derive(Debug, Clone, Copy)]
#[allow(unused)]
pub enum RingColor {
    Red,
    Blue,
}

impl RingColor {
    fn reflectivity(&self) -> core::ops::Range<f64> {
        match self {
            RingColor::Red => 0.3..0.5,
            RingColor::Blue => 0.5..1.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum IntakeState {
    Forward {
        accept: Option<RingColor>,
        reject_time: Option<Instant>,
    },
    Reverse,
    PartialIntake {
        start: Instant,
    },
    ArmIntake {
        start: Instant,
    },
    Stop,
}

const RING_THRESHOLD: core::ops::Range<f64> = 0.4..1.0;
const TIMEOUT: u128 = 3000;
const RING_REJECT_STOP_TIME: Duration = Duration::from_millis(500);
const RING_REJECT_RESTART_TIME: Duration = Duration::from_millis(1000);
const LADY_BROWN_HOLD_DURATION: Duration = Duration::from_millis(3000);

#[derive(Debug)]
pub struct IntakeInner {
    motor: Motor,
    partial_line_tracker: AdiLineTracker,
    lady_brown_line_tracker: AdiLineTracker,
    state: IntakeState,
    lady_brown_line_tracker_start: Option<Instant>,
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
            IntakeState::Forward {
                accept,
                reject_time,
            } => {
                if let Some(reject_time) = reject_time
                    && (RING_REJECT_STOP_TIME..RING_REJECT_RESTART_TIME)
                        .contains(&reject_time.elapsed())
                {
                    self.motor.brake(BrakeMode::Brake).context(MotorSnafu)?;
                } else {
                    self.motor
                        .set_voltage(self.motor.max_voltage())
                        .context(MotorSnafu)?;
                }
                if let Some(accept_color) = accept {
                    let current_value = self
                        .partial_line_tracker
                        .reflectivity()
                        .context(LineTrackerPortSnafu)?;
                    if RING_THRESHOLD.contains(&current_value) {
                        debug!("ring accepted, reflectivity: {}", current_value);
                        // If there is a ring, test its color
                        if accept_color.reflectivity().contains(&current_value) {
                            debug!("ring color accepted");
                        } else {
                            debug!("ring color rejected");
                            self.state = IntakeState::Forward {
                                accept,
                                reject_time: Some(Instant::now()),
                            };
                        }
                    }
                }
            }
            IntakeState::Reverse => {
                self.motor
                    .set_voltage(-self.motor.max_voltage())
                    .context(MotorSnafu)?;
            }
            IntakeState::PartialIntake { start } | IntakeState::ArmIntake { start } => {
                if start.elapsed().as_millis() > TIMEOUT {
                    error!("Partially intaking timeout");
                    self.state = IntakeState::Stop;
                } else {
                    self.motor
                        .set_voltage(self.motor.max_voltage())
                        .context(MotorSnafu)?;
                    let current_value = if matches!(self.state, IntakeState::PartialIntake { .. }) {
                        self.partial_line_tracker
                            .reflectivity()
                            .context(LineTrackerPortSnafu)?
                    } else {
                        self.lady_brown_line_tracker
                            .reflectivity()
                            .context(LineTrackerPortSnafu)?
                    };
                    if RING_THRESHOLD.contains(&current_value) {
                        self.state = IntakeState::Stop;
                    }
                }
            }
            IntakeState::Stop => {
                if RING_THRESHOLD.contains(
                    &self
                        .partial_line_tracker
                        .reflectivity()
                        .context(LineTrackerPortSnafu)?,
                ) {
                    self.motor.brake(BrakeMode::Hold).context(MotorSnafu)?;
                } else if RING_THRESHOLD.contains(
                    &self
                        .lady_brown_line_tracker
                        .reflectivity()
                        .context(LineTrackerPortSnafu)?,
                ) && (self.lady_brown_line_tracker_start.is_none()
                    || self.lady_brown_line_tracker_start.unwrap().elapsed()
                        < LADY_BROWN_HOLD_DURATION)
                {
                    if self.lady_brown_line_tracker_start.is_none() {
                        self.lady_brown_line_tracker_start = Some(Instant::now());
                    }
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
    pub fn new(
        motor: Motor,
        partial_line_tracker: AdiLineTracker,
        lady_brown_line_tracker: AdiLineTracker,
    ) -> Self {
        Self(Arc::new(Mutex::new(IntakeInner {
            motor,
            partial_line_tracker,
            lady_brown_line_tracker,
            state: IntakeState::Stop,
            lady_brown_line_tracker_start: None,
        })))
    }

    pub async fn is_ring_released_in_lady_brown(&self) -> bool {
        let inner = self.0.lock().await;
        inner
            .lady_brown_line_tracker_start
            .is_some_and(|start| start.elapsed() > LADY_BROWN_HOLD_DURATION)
    }

    pub fn temperature(&self) -> f64 {
        match self.0.try_lock() {
            Some(inner) => inner.motor.temperature().unwrap_or(0.0),
            None => 0.0,
        }
    }

    pub fn task(&self) {
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
        };
    }

    pub async fn run(&self, direction: Direction) {
        let mut inner = self.0.lock().await;
        inner.state = match direction {
            Direction::Forward => IntakeState::Forward {
                accept: None,
                reject_time: None,
            },
            Direction::Reverse => IntakeState::Reverse,
        };
    }

    pub async fn run_forward_accept(&self, color: RingColor) {
        let mut inner = self.0.lock().await;
        inner.state = IntakeState::Forward {
            accept: Some(color),
            reject_time: None,
        };
    }
}
