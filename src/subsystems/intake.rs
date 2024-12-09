use snafu::{ResultExt, Snafu};
use vexide::{
    core::println,
    devices::{smart::motor::MotorError, PortError},
    prelude::{AdiAnalogIn, BrakeMode, Direction, Motor},
};

pub struct Intake {
    motor: Motor,
    line_tracker: AdiAnalogIn,
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
        }
    }

    pub fn run(&mut self, direction: Direction) -> Result<(), IntakeError> {
        self.motor
            .set_voltage(match direction {
                Direction::Forward => self.motor.max_voltage(),
                Direction::Reverse => -self.motor.max_voltage(),
            })
            .context(MotorSnafu)?;
        Ok(())
    }

    pub fn partial_intake(&mut self) -> Result<(), IntakeError> {
        let initial_value = self.line_tracker.value().context(LineTrackerPortSnafu)?;
        println!("initial: {}", initial_value);
        self.motor
            .set_voltage(self.motor.max_voltage())
            .context(MotorSnafu)?;
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), IntakeError> {
        self.motor.brake(BrakeMode::Coast).context(MotorSnafu)?;
        Ok(())
    }

    pub fn hold(&mut self) -> Result<(), IntakeError> {
        self.motor.brake(BrakeMode::Brake).context(MotorSnafu)?;
        Ok(())
    }
}
