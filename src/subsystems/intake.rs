use core::time::Duration;

use log::{debug, error};
use snafu::{ResultExt, Snafu};
use vexide::{
    core::{println, time::Instant},
    devices::{adi::ADI_UPDATE_INTERVAL, smart::motor::MotorError, PortError},
    prelude::{sleep, AdiAnalogIn, BrakeMode, Direction, Motor},
};

const RING_THRESHOLD: u16 = 2500;
const TIMEOUT: u128 = 3000;

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

    pub async fn partial_intake(&mut self) -> Result<(), IntakeError> {
        self.run(Direction::Forward)?;
        let start = Instant::now();
        loop {
            let current_value = self.line_tracker.value().context(LineTrackerPortSnafu)?;
            if current_value < RING_THRESHOLD {
                break;
            }
            if start.elapsed().as_millis() > TIMEOUT {
                error!("Partially intaking timeout");
                break;
            }
            sleep(ADI_UPDATE_INTERVAL).await;
        }
        self.stop()?;
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), IntakeError> {
        let current_value = self.line_tracker.value().context(LineTrackerPortSnafu)?;
        if current_value < RING_THRESHOLD {
            self.motor.brake(BrakeMode::Hold).context(MotorSnafu)?;
        } else {
            self.motor.set_voltage(0.0).context(MotorSnafu)?;
        }
        Ok(())
    }

    pub fn hold(&mut self) -> Result<(), IntakeError> {
        self.motor.set_voltage(0.0).context(MotorSnafu)?;
        Ok(())
    }
}
