#![allow(unused)]

use alloc::vec::Vec;

use vexide::{devices::smart::motor::MotorError, prelude::*};

/// A motor group
///
/// Guaranteed to have at least one motor.
#[derive(Debug)]
pub struct MotorGroup(Vec<Motor>);

impl MotorGroup {
    /// Create a new MotorGroup, a group of motors which can be controlled
    /// together.
    ///
    /// # Panics
    ///
    /// Panics if no motors are provided
    pub fn new(motors: Vec<Motor>) -> MotorGroup {
        if motors.is_empty() {
            panic!("MotorGroup::new requires a positive number of motors");
        }
        MotorGroup(motors)
    }

    /// Create a new MotorGroup from smart ports and a boolean indicating reversed
    ///
    /// # Panics
    ///
    /// Panics if no ports are provided
    pub fn from_ports(ports: Vec<(SmartPort, bool)>, gearset: Gearset) -> MotorGroup {
        if ports.is_empty() {
            panic!("MotorGroup::from_ports requires a positive number of ports");
        }
        MotorGroup(
            ports
                .into_iter()
                .map(|(port, reversed)| {
                    Motor::new(
                        port,
                        gearset,
                        if reversed {
                            Direction::Reverse
                        } else {
                            Direction::Forward
                        },
                    )
                })
                .collect(),
        )
    }

    pub fn set_target(&mut self, target: MotorControl) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_target(target)?;
        }
        Ok(())
    }

    pub fn brake(&mut self, target: BrakeMode) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.brake(target)?;
        }
        Ok(())
    }

    pub fn set_velocity(&mut self, rpm: i32) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_velocity(rpm)?;
        }
        Ok(())
    }

    pub fn set_voltage(&mut self, volts: f64) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_voltage(volts)?;
        }
        Ok(())
    }

    pub fn set_position_target(
        &mut self,
        position: Position,
        velocity: i32,
    ) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_position_target(position, velocity)?;
        }
        Ok(())
    }

    pub fn target(&self) -> MotorControl {
        self.0[0].target()
    }

    pub fn position(&self) -> Result<Position, MotorError> {
        let mut total = Position::from_degrees(0.0);
        for motor in &self.0 {
            total += motor.position()?;
        };
        // tpr is what position uses internally, so we can use it here to avoid convertion internally
        Ok(Position::from_ticks(total.as_ticks(4_608_000) / self.0.len() as i64, 4_608_000))
    }

    pub fn reset_position(&mut self) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.reset_position()?;
        }
        Ok(())
    }

    pub fn velocity(&self) -> Result<f64, MotorError> {
        let mut total = 0.0;
        for motor in &self.0 {
            total += motor.velocity()?;
        }
        Ok(total / self.0.len() as f64)
    }

    pub fn voltage(&self) -> Result<f64, MotorError> {
        let mut total = 0.0;
        for motor in &self.0 {
            total += motor.voltage()?;
        }
        Ok(total / self.0.len() as f64)
    }

    pub fn current(&self) -> Result<f64, MotorError> {
        let mut total = 0.0;
        for motor in &self.0 {
            total += motor.current()?;
        }
        Ok(total / self.0.len() as f64)
    }

    pub fn temperature(&self) -> Result<f64, MotorError> {
        let mut total = 0.0;
        for motor in &self.0 {
            total += motor.temperature()?;
        }
        Ok(total / self.0.len() as f64)
    }

    pub fn power(&self) -> Result<f64, MotorError> {
        let mut total = 0.0;
        for motor in &self.0 {
            total += motor.power()?;
        }
        Ok(total / self.0.len() as f64)
    }
}
