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
    #[deprecated]
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

    /// Sets the target that the motor group should attempt to reach.
    ///
    /// This could be a voltage, velocity, position, or even brake mode.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_target(MotorControl::Voltage(5.0));
    ///     sleep(Duration::from_secs(1)).await;
    ///     let _ = motor_group.set_target(MotorControl::Brake(BrakeMode::Hold));
    /// }
    /// ```
    pub fn set_target(&mut self, target: MotorControl) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_target(target)?;
        }
        Ok(())
    }

    /// Set the brake mode for all motors in the group
    pub fn brake(&mut self, mode: BrakeMode) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.brake(mode)?;
        }
        Ok(())
    }

    /// Set the velocity for all motors in the group
    pub fn set_velocity(&mut self, rpm: i32) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_velocity(rpm)?;
        }
        Ok(())
    }

    /// Sets the motor group's output voltage.
    ///
    /// This voltage value spans from -12 (fully spinning reverse) to +12 (fully spinning forwards) volts, and
    /// controls the raw output of the motor group.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Give the motor group full power:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_voltage(motor_group.max_voltage());
    /// }
    /// ```
    ///
    /// Drive the motor group based on a controller joystick:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let controller = peripherals.primary_controller;
    ///     loop {
    ///         let controller_state = controller.state().unwrap_or_default();
    ///         let voltage = controller_state.left_stick.x() * motor_group.max_voltage();
    ///         motor_group.set_voltage(voltage).unwrap();
    ///     }
    /// }
    /// ```
    pub fn set_voltage(&mut self, volts: f64) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_voltage(volts)?;
        }
        Ok(())
    }

    /// Sets an absolute position target for the motor group to attempt to reach.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    ///
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_position_target(Position::from_degrees(90.0), 200);
    /// }
    /// ```
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

    /// Changes the output velocity for a profiled movement (motor_move_absolute or motor_move_relative).
    ///
    /// This will have no effect if the motor group is not following a profiled movement.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     // Set the motor group's target to a Position so that changing the velocity isn't a noop.
    ///     let _ = motor_group.set_target(MotorControl::Position(Position::from_degrees(90.0), 200));
    ///     let _ = motor_group.set_profiled_velocity(100).unwrap();
    /// }
    /// ```
    pub fn set_profiled_velocity(&mut self, velocity: i32) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_profiled_velocity(velocity)?;
        }
        Ok(())
    }

    /// Sets the gearset of the motor group.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    /// - A [`MotorError::SetGearsetExp`] is returned if the motor is a 5.5W EXP Smart Motor, which has no swappable gearset.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     // This must be a V5 motor group
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     // Set the motor group to use the red gearset
    ///     motor_group.set_gearset(Gearset::Red).unwrap();
    /// }
    /// ```
    pub fn set_gearset(&mut self, gearset: Gearset) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_gearset(gearset)?;
        }
        Ok(())
    }

    /// Sets the current encoder position to zero without moving the motor group.
    ///
    /// Analogous to taring or resetting the encoder to the current position.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Move the motor group in increments of 10 degrees:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     loop {
    ///         motor_group.set_position_target(Position::from_degrees(10.0), 200).unwrap();
    ///         sleep(Duration::from_secs(1)).await;
    ///         motor_group.reset_position().unwrap();
    ///     }
    /// }
    /// ```
    pub fn reset_position(&mut self) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.reset_position()?;
        }
        Ok(())
    }

    /// Sets the current encoder position to the given position without moving the motor group.
    ///
    /// Analogous to taring or resetting the encoder so that the new position is equal to the given position.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Set the current position of the motor group to 90 degrees:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     motor_group.set_position(Position::from_degrees(90.0)).unwrap();
    /// }
    /// ```
    ///
    /// Reset the position of the motor group to 0 degrees (analogous to [`reset_position`](MotorGroup::reset_position)):
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     motor_group.set_position(Position::from_degrees(0.0)).unwrap();
    /// }
    /// ```
    pub fn set_position(&mut self, position: Position) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_position(position)?;
        }
        Ok(())
    }

    /// Sets the current limit for the motor group in amps.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Limit the current draw of a motor group to 2.5A:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_current_limit(2.5);
    /// }
    /// ```
    pub fn set_current_limit(&mut self, limit: f64) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_current_limit(limit)?;
        }
        Ok(())
    }

    /// Sets the voltage limit for the motor group in volts.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Limit the voltage of a motor group to 4V:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_voltage_limit(4.0);
    ///     // Will appear as if the voltage was set to only 4V
    ///     let _ = motor_group.set_voltage(12.0);
    /// }
    /// ```
    pub fn set_voltage_limit(&mut self, limit: f64) -> Result<(), MotorError> {
        for motor in &mut self.0 {
            motor.set_voltage_limit(limit)?;
        }
        Ok(())
    }

    /// Returns the current [`MotorControl`] target that the motor group is attempting to use.
    /// This value is set with [`MotorGroup::set_target`].
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     motor_group.set_target(MotorControl::Brake(BrakeMode::Hold));
    ///     let target = motor_group.target();
    ///     assert_eq!(target, MotorControl::Brake(BrakeMode::Hold));
    /// }
    /// ```
    pub fn target(&self) -> MotorControl {
        self.0[0].target()
    }

    /// Returns the maximum voltage for the motor group based off of its [motor type](MotorGroup::motor_type).
    ///
    /// # Examples
    ///
    /// Run a motor group at max speed, agnostic of its type:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// fn run_motor_group_at_max_speed(motor_group: &mut MotorGroup) {
    ///     motor_group.set_voltage(motor_group.max_voltage()).unwrap();
    /// }
    /// ```
    #[must_use]
    pub const fn max_voltage(&self) -> f64 {
        todo!()
        // self.0[0].max_voltage()
    }

    /// Returns the motor group's estimate of its angular velocity in rotations per minute (RPM).
    ///
    /// # Accuracy
    ///
    /// In some cases, this reported value may be noisy or inaccurate, especially for systems where accurate
    /// velocity control at high speeds is required (such as flywheels). If the accuracy of this value proves
    /// inadequate, you may opt to perform your own velocity calculations by differentiating [`MotorGroup::position`]
    /// over the reported internal timestamp of the motor group using [`MotorGroup::timestamp`].
    ///
    /// > For more information about Smart motor velocity estimation, see [this article](https://sylvie.fyi/sylib/docs/db/d8e/md_module_writeups__velocity__estimation.html).
    ///
    /// # Note
    ///
    /// To get the current **target** velocity instead of the estimated velocity, use [`MotorGroup::target`].
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Get the current velocity of a motor group:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.velocity().unwrap());
    /// }
    /// ```
    ///
    /// Calculate acceleration of a motor group:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     let mut last_velocity = motor_group.velocity().unwrap();
    ///     let mut start_time = Instant::now();
    ///     loop {
    ///         let velocity = motor_group.velocity().unwrap();
    ///         // Make sure we don't divide by zero
    ///         let elapsed = start_time.elapsed().as_secs_f64() + 0.001;
    ///
    ///         // Calculate acceleration
    ///         let acceleration = (velocity - last_velocity) / elapsed;
    ///         println!("Velocity: {:.2} RPM, Acceleration: {:.2} RPM/s", velocity, acceleration);
    ///
    ///         last_velocity = velocity;
    ///         start_time = Instant::now();
    ///    }
    /// }
    /// ```
    pub fn velocity(&self) -> Result<f64, MotorError> {
        let mut total_velocity = 0.0;
        for motor in &self.0 {
            total_velocity += motor.velocity()?;
        }
        Ok(total_velocity / self.0.len() as f64)
    }

    /// Returns the power drawn by the motor group in Watts.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the power drawn by a motor group:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     loop {
    ///         println!("Power: {:.2}W", motor_group.power().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn power(&self) -> Result<f64, MotorError> {
        let mut total_power = 0.0;
        for motor in &self.0 {
            total_power += motor.power()?;
        }
        Ok(total_power / self.0.len() as f64)
    }

    /// Returns the torque output of the motor group in Nm.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the torque output of a motor group:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     loop {
    ///         println!("Torque: {:.2}Nm", motor_group.torque().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn torque(&self) -> Result<f64, MotorError> {
        let mut total_torque = 0.0;
        for motor in &self.0 {
            total_torque += motor.torque()?;
        }
        Ok(total_torque / self.0.len() as f64)
    }

    /// Returns the voltage the motor group is drawing in volts.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the voltage drawn by a motor group:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     loop {
    ///         println!("Voltage: {:.2}V", motor_group.voltage().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn voltage(&self) -> Result<f64, MotorError> {
        let mut total_voltage = 0.0;
        for motor in &self.0 {
            total_voltage += motor.voltage()?;
        }
        Ok(total_voltage / self.0.len() as f64)
    }

    /// Returns the current position of the motor group.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the current position of a motor group:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     loop {
    ///         println!("Position: {:?}", motor_group.position().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn position(&self) -> Result<Position, MotorError> {
        let mut total_position = Position::from_degrees(0.0);
        for motor in &self.0 {
            total_position += motor.position()?;
        }
        Ok(total_position / self.0.len() as i64)
    }

    /// Returns the electrical current draw of the motor group in amps.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the current draw of a motor group:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     motor_group.set_voltage(motor_group.max_voltage()).unwrap();
    ///     loop {
    ///         println!("Current: {:.2}A", motor_group.current().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn current(&self) -> Result<f64, MotorError> {
        let mut total_current = 0.0;
        for motor in &self.0 {
            total_current += motor.current()?;
        }
        Ok(total_current / self.0.len() as f64)
    }

    /// Returns the efficiency of the motor group from a range of [0.0, 1.0].
    ///
    /// An efficiency of 1.0 means that the motor group is moving electrically while
    /// drawing no electrical power, and an efficiency of 0.0 means that the motor group
    /// is drawing power but not moving.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the efficiency of a motor group:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_voltage(motor_group.max_voltage())
    ///     loop {
    ///         println!("Efficiency: {:.2}", motor_group.efficiency().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn efficiency(&self) -> Result<f64, MotorError> {
        let mut total_efficiency = 0.0;
        for motor in &self.0 {
            total_efficiency += motor.efficiency()?;
        }
        Ok(total_efficiency / self.0.len() as f64)
    }

    /// Returns the internal temperature recorded by the motor group in increments of 5 °C.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Turn off the motor group if it gets too hot:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_voltage(12.0);
    ///     loop {
    ///         if motor_group.temperature().unwrap() > 30.0 {
    ///             let _ = motor_group.brake(BrakeMode::Coast);
    ///         } else {
    ///             let _ = motor_group.set_voltage(12.0);
    ///         }
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn temperature(&self) -> Result<f64, MotorError> {
        let mut total_temperature = 0.0;
        for motor in &self.0 {
            total_temperature += motor.temperature()?;
        }
        Ok(total_temperature / self.0.len() as f64)
    }
}
