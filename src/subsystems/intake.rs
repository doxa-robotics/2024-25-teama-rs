use core::time::Duration;

use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    core::time::Instant,
    devices::{smart::motor::MotorError, PortError},
    prelude::{AdiAnalogIn, BrakeMode, Direction, Motor},
};

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

const RING_THRESHOLD: u16 = 2500;
const TIMEOUT: u128 = 3000;
const ARM_INTAKE_SETTLING_TIME: u128 = 600;

pub struct Intake {
    motor: Motor,
    line_tracker: AdiAnalogIn,
    pub state: IntakeState,
}

#[derive(Debug, Snafu)]
pub enum IntakeError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
    #[snafu(display("line tracker error: {}", source))]
    LineTrackerPortError { source: PortError },
}

impl Intake {
    pub fn new(motor: Motor, line_tracker: AdiAnalogIn) -> Self {
        Self {
            motor,
            line_tracker,
            state: IntakeState::Stopped,
        }
    }

    pub fn temperature(&self) -> f64 {
        self.motor.temperature().unwrap_or(0.0)
    }

    pub fn update(&mut self) -> Result<(), IntakeError> {
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
                        self.state = IntakeState::Stopped;
                    }
                    return Ok(());
                }
                if start.elapsed().as_millis() > TIMEOUT {
                    error!("Partially intaking timeout");
                    self.state = IntakeState::Stopped;
                } else {
                    self.motor
                        .set_voltage(self.motor.max_voltage())
                        .context(MotorSnafu)?;
                    let current_value = self.line_tracker.value().context(LineTrackerPortSnafu)?;
                    if current_value < RING_THRESHOLD {
                        match self.state {
                            IntakeState::PartialIntake { start: _ } => {
                                self.state = IntakeState::Stopped;
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
            IntakeState::Stopped => {
                let current_value = self.line_tracker.value().context(LineTrackerPortSnafu)?;
                if current_value < RING_THRESHOLD {
                    self.motor.brake(BrakeMode::Hold).context(MotorSnafu)?;
                } else {
                    self.motor.set_voltage(0.0).context(MotorSnafu)?;
                }
            }
            IntakeState::ForwardUntil { end } => {
                if Instant::now() > end {
                    self.state = IntakeState::Stopped;
                } else {
                    self.motor
                        .set_voltage(self.motor.max_voltage())
                        .context(MotorSnafu)?;
                }
            }
        }
        Ok(())
    }

    pub fn stop(&mut self) {
        self.state = IntakeState::Stopped;
    }

    pub fn stop_if_running(&mut self) {
        if matches!(self.state, IntakeState::Forward | IntakeState::Reverse) {
            self.stop();
        }
    }

    pub fn partial_intake(&mut self) {
        self.state = IntakeState::PartialIntake {
            start: Instant::now(),
        };
    }

    pub fn arm_intake(&mut self) {
        self.state = IntakeState::ArmIntake {
            start: Instant::now(),
            settling_start: None,
        };
    }

    pub fn run(&mut self, direction: Direction) {
        self.state = match direction {
            Direction::Forward => IntakeState::Forward,
            Direction::Reverse => IntakeState::Reverse,
        };
    }
}
