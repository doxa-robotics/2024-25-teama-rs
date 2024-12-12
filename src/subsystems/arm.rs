use snafu::{ResultExt, Snafu};
use vexide::{
    devices::smart::motor::MotorError,
    prelude::{BrakeMode, Direction},
};

use crate::utils::motor_group::MotorGroup;

pub struct Arm {
    motors: MotorGroup,
}

#[derive(Debug, Snafu)]
pub enum ArmError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
}

impl Arm {
    pub fn new(motors: MotorGroup) -> Self {
        Self { motors }
    }

    pub fn run(&mut self, direction: Direction) -> Result<(), ArmError> {
        //if self.motors.velocity().context(MotorSnafu)?.abs() < TIMEOUT_VELOCITY_THRESHOLD {
        // self.motors.brake(BrakeMode::Brake).context(MotorSnafu)?;
        // } else {
        self.motors
            .set_velocity(match direction {
                Direction::Forward => 100,
                Direction::Reverse => -100,
            })
            .context(MotorSnafu)?;
        //}
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), ArmError> {
        self.motors.brake(BrakeMode::Brake).context(MotorSnafu)?;
        Ok(())
    }
}
