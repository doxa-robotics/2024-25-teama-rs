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
        if motors.len() == 0 {
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
        if ports.len() == 0 {
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
}
