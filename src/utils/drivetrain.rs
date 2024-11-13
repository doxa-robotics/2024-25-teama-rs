use vexide::{
    core::println,
    devices::smart::{imu::InertialError, motor::MotorError},
    prelude::{sleep, Float, InertialSensor, Motor, SmartDevice},
};

use super::motor_group::MotorGroup;

/// A configuration struct for a drivetrain
///
/// This struct is used to configure a drivetrain, and contains the following fields:
/// - `turning_p`: The proportional constant for turning
/// - `turning_i`: The integral constant for turning
/// - `turning_d`: The derivative constant for turning
/// - `turning_tolerance`: The tolerance for turning (in degrees)
/// - `drive_p`: The proportional constant for driving
/// - `drive_i`: The integral constant for driving
/// - `drive_d`: The derivative constant for driving
/// - `drive_tolerance`: The tolerance for driving (in mm)
/// - `tolerance_velocity`: The tolerance for velocity (in mm/s)
#[derive(Debug, Clone, Copy)]
pub struct DrivetrainConfig {
    /// The proportional constant for turning
    pub turning_p: f64,
    /// The integral constant for turning
    pub turning_i: f64,
    /// The derivative constant for turning
    pub turning_d: f64,
    /// The tolerance for turning (in degrees)
    pub turning_tolerance: f64,
    /// The proportional constant for driving
    pub drive_p: f64,
    /// The integral constant for driving
    pub drive_i: f64,
    /// The derivative constant for driving
    pub drive_d: f64,
    /// The tolerance for driving (in mm)
    pub drive_tolerance: f64,
    /// The tolerance for velocity (in mm/s)
    pub tolerance_velocity: f64,

    /// The distance a wheel travels in one rotation
    pub wheel_circumference: f64,
}

/// An error returned by the Drivetrain.
#[derive(Debug)]
pub enum DrivetrainError {
    /// An error occurred with a motor
    Motor(MotorError),
    Inertial(InertialError),
}

/// A drivetrain struct
///
/// This struct is used to control a drivetrain, which contains two MotorGroups, one for the left
/// side and one for the right side. This struct is used to control the drivetrain as a whole.
///
/// It supports a generic `drive_for` method which allows the robot to drive for a certain distance
/// using a PID controller from the Rust `pid` crate.
#[derive(Debug)]
pub struct Drivetrain {
    left: MotorGroup,
    right: MotorGroup,
    inertial: InertialSensor,
    config: DrivetrainConfig,
}

impl Drivetrain {
    /// Create a new Drivetrain
    pub fn new(
        left: MotorGroup,
        right: MotorGroup,
        inertial: InertialSensor,
        config: DrivetrainConfig,
    ) -> Drivetrain {
        Drivetrain {
            left,
            right,
            inertial,
            config,
        }
    }

    /// Returns a mutable reference to the left motors
    pub fn left(&mut self) -> &mut MotorGroup {
        &mut self.left
    }

    /// Returns a mutable reference to the right motors
    pub fn right(&mut self) -> &mut MotorGroup {
        &mut self.right
    }

    /// Returns a mutable reference to the inertial sensor
    pub fn inertial(&mut self) -> &mut InertialSensor {
        &mut self.inertial
    }

    /// Drive the robot for a certain distance
    ///
    /// This method uses a PID controller to drive the robot for a certain distance. It is a blocking (async)
    /// method, and will not return until the robot has driven the specified distance.
    ///
    /// # Arguments
    ///
    /// - `target_distance`: The distance to drive in mm
    pub async fn drive_for(&mut self, target_distance: f64) -> Result<(), DrivetrainError> {
        // Get the initial position
        self.left.reset_position().map_err(DrivetrainError::Motor)?;
        self.right
            .reset_position()
            .map_err(DrivetrainError::Motor)?;

        // since we just reset the position, the distance is 0 (in mm)
        let mut left_distance: f64 = self
            .left
            .position()
            .map_err(DrivetrainError::Motor)?
            .as_revolutions()
            * self.config.wheel_circumference;
        let mut right_distance: f64 = self
            .right
            .position()
            .map_err(DrivetrainError::Motor)?
            .as_revolutions()
            * self.config.wheel_circumference;
        let mut left_velocity = self.left.velocity().map_err(DrivetrainError::Motor)?;
        let mut right_velocity = self.right.velocity().map_err(DrivetrainError::Motor)?;

        let mut left_controller: pid::Pid<f64> =
            pid::Pid::new(target_distance, Motor::V5_MAX_VOLTAGE);
        left_controller
            .p(self.config.drive_p, f64::MAX)
            .i(self.config.drive_i, f64::MAX)
            .d(self.config.drive_d, f64::MAX);
        let mut right_controller: pid::Pid<f64> =
            pid::Pid::new(target_distance, Motor::V5_MAX_VOLTAGE);
        right_controller
            .p(self.config.drive_p, f64::MAX)
            .i(self.config.drive_i, f64::MAX)
            .d(self.config.drive_d, f64::MAX);

        while ((left_distance - target_distance).abs() >= self.config.drive_tolerance
            || (right_distance - target_distance).abs() >= self.config.drive_tolerance)
            || (left_velocity.abs() >= self.config.tolerance_velocity
                || right_velocity.abs() >= self.config.tolerance_velocity)
        {
            // Get the current position
            left_distance = self
                .left
                .position()
                .map_err(DrivetrainError::Motor)?
                .as_revolutions()
                * self.config.wheel_circumference;
            right_distance = self
                .right
                .position()
                .map_err(DrivetrainError::Motor)?
                .as_revolutions()
                * self.config.wheel_circumference;

            // Get the current velocity
            left_velocity = self.left.velocity().map_err(DrivetrainError::Motor)?;
            right_velocity = self.right.velocity().map_err(DrivetrainError::Motor)?;

            // Calculate the output
            let left_output = left_controller.next_control_output(left_distance).output;
            let right_output = right_controller.next_control_output(right_distance).output;

            // Set the output
            self.left
                .set_voltage(left_output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .map_err(DrivetrainError::Motor)?;
            self.right
                .set_voltage(right_output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .map_err(DrivetrainError::Motor)?;

            sleep(Motor::UPDATE_INTERVAL).await;
        }

        // brake the left and right motors
        self.left
            .brake(vexide::prelude::BrakeMode::Brake)
            .map_err(DrivetrainError::Motor)?;
        self.right
            .brake(vexide::prelude::BrakeMode::Brake)
            .map_err(DrivetrainError::Motor)?;
        Ok(())
    }

    /// Turn the robot a certain angle
    ///
    /// This method uses a PID controller to turn the robot a certain angle. It is a blocking (async)
    /// method, and will not return until the robot has turned the specified distance.
    ///
    /// # Arguments
    ///
    /// - `target_angle_delta`: The angle to turn by in radians
    pub async fn turn_for(&mut self, target_angle_delta: f64) -> Result<(), DrivetrainError> {
        // Get the initial position
        self.inertial
            .reset_heading()
            .map_err(DrivetrainError::Inertial)?;
        let mut heading = 0.0;

        let mut left_velocity = self.left.velocity().map_err(DrivetrainError::Motor)?;
        let mut right_velocity = self.right.velocity().map_err(DrivetrainError::Motor)?;

        let mut controller: pid::Pid<f64> =
            pid::Pid::new(target_angle_delta, Motor::V5_MAX_VOLTAGE);
        controller
            .p(self.config.turning_p, f64::MAX)
            .i(self.config.turning_i, f64::MAX)
            .d(self.config.turning_d, f64::MAX);

        while (heading - target_angle_delta).abs() >= self.config.turning_tolerance
            || (left_velocity.abs() >= self.config.tolerance_velocity
                || right_velocity.abs() >= self.config.tolerance_velocity)
        {
            // Get the current position
            heading = self
                .inertial
                .heading()
                .map_err(DrivetrainError::Inertial)?
                .rem_euclid(360.0);

            // Get the current velocity
            left_velocity = self.left.velocity().map_err(DrivetrainError::Motor)?;
            right_velocity = self.right.velocity().map_err(DrivetrainError::Motor)?;

            // Calculate the output
            let output = controller.next_control_output(heading).output;

            // Set the output
            self.left
                .set_voltage(output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .map_err(DrivetrainError::Motor)?;
            self.right
                .set_voltage(-output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .map_err(DrivetrainError::Motor)?;

            sleep(Motor::UPDATE_INTERVAL).await;
        }

        // brake the left and right motors
        self.left
            .brake(vexide::prelude::BrakeMode::Brake)
            .map_err(DrivetrainError::Motor)?;
        self.right
            .brake(vexide::prelude::BrakeMode::Brake)
            .map_err(DrivetrainError::Motor)?;
        Ok(())
    }
}