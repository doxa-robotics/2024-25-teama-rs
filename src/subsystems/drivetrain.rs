use alloc::sync::Arc;
use core::time::Duration;

use snafu::{ResultExt, Snafu};
use vexide::{
    core::{dbg, println, sync::Mutex, time::Instant},
    devices::smart::{imu::InertialError, motor::MotorError},
    prelude::{sleep, spawn, Float, InertialSensor, Motor, SmartDevice},
};

use super::super::utils::motor_group::MotorGroup;

#[derive(Debug, Snafu)]
pub enum DrivetrainError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
    #[snafu(display("inertial error: {}", source))]
    Inertial { source: InertialError },
}

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
    /// Timeout for driving/turning in ms
    pub timeout: Duration,

    /// The distance a wheel travels in one rotation
    pub wheel_circumference: f64,
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
    inertial: Arc<Mutex<InertialSensor>>,
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
            inertial: Arc::new(Mutex::new(inertial)),
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

    /// Is the inertial sensor calibrating?
    pub fn is_inertial_calibrating(&self) -> bool {
        self.inertial
            .try_lock()
            .map(|inertial| inertial.is_calibrating().unwrap_or(false))
            // assume yes if it's locked
            .unwrap_or(true)
    }

    pub fn inertial_heading(&self) -> f64 {
        self.inertial
            .try_lock()
            .map(|i| i.heading().unwrap_or(0.0))
            .unwrap_or(0.0)
    }

    /// Calibrate the inertial sensor
    pub fn calibrate_inertial(&mut self) {
        let inertial = self.inertial.clone();
        spawn(async move {
            let mut inertial = inertial.lock().await;
            if inertial.calibrate().await.is_err() {
                _ = inertial.calibrate().await;
            }
        })
        .detach();
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
        self.left.reset_position().context(MotorSnafu)?;
        self.right.reset_position().context(MotorSnafu)?;

        // since we just reset the position, the distance is 0 (in mm)
        let mut left_distance: f64 = self.left.position().context(MotorSnafu)?.as_revolutions()
            * self.config.wheel_circumference;
        let mut right_distance: f64 = self.right.position().context(MotorSnafu)?.as_revolutions()
            * self.config.wheel_circumference;
        let mut left_velocity = self.left.velocity().context(MotorSnafu)?;
        let mut right_velocity = self.right.velocity().context(MotorSnafu)?;

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

        let mut last_moving_instant = Instant::now();
        while ((left_distance - target_distance).abs() >= self.config.drive_tolerance
            || (right_distance - target_distance).abs() >= self.config.drive_tolerance)
            || (left_velocity.abs() >= self.config.tolerance_velocity
                || right_velocity.abs() >= self.config.tolerance_velocity)
        {
            // Get the current position
            left_distance = self.left.position().context(MotorSnafu)?.as_revolutions()
                * self.config.wheel_circumference;
            right_distance = self.right.position().context(MotorSnafu)?.as_revolutions()
                * self.config.wheel_circumference;

            // Get the current velocity
            left_velocity = self.left.velocity().context(MotorSnafu)?;
            right_velocity = self.right.velocity().context(MotorSnafu)?;

            if left_velocity.abs() > self.config.tolerance_velocity
                || right_velocity.abs() > self.config.tolerance_velocity
            {
                last_moving_instant = Instant::now();
            }
            if last_moving_instant.elapsed() > self.config.timeout {
                println!("Drivetrain.drive_for timed out");
                break;
            }

            // Calculate the output
            let left_output = left_controller.next_control_output(left_distance).output;
            let right_output = right_controller.next_control_output(right_distance).output;

            // Set the output
            self.left
                .set_voltage(left_output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .context(MotorSnafu)?;
            self.right
                .set_voltage(right_output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .context(MotorSnafu)?;

            sleep(Motor::UPDATE_INTERVAL).await;
        }

        // brake the left and right motors
        self.left
            .brake(vexide::prelude::BrakeMode::Hold)
            .context(MotorSnafu)?;
        self.right
            .brake(vexide::prelude::BrakeMode::Hold)
            .context(MotorSnafu)?;
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
        let mut inertial = self.inertial.lock().await;

        // Get the initial position
        inertial
            .set_heading(180.0 - target_angle_delta / 2.0)
            .context(InertialSnafu)?;
        let mut real_heading = inertial.heading().context(InertialSnafu)?;
        let target_heading = real_heading + target_angle_delta;
        let mut heading = real_heading;
        let mut heading_difference = 0.0;

        let mut left_velocity = self.left.velocity().context(MotorSnafu)?;
        let mut right_velocity = self.right.velocity().context(MotorSnafu)?;

        let mut controller: pid::Pid<f64> = pid::Pid::new(target_heading, Motor::V5_MAX_VOLTAGE);
        controller
            .p(self.config.turning_p, f64::MAX)
            .i(self.config.turning_i, f64::MAX)
            .d(self.config.turning_d, f64::MAX);

        let mut last_moving_instant = Instant::now();
        while (heading - target_heading).abs() >= self.config.turning_tolerance
            || (left_velocity.abs() >= self.config.tolerance_velocity
                || right_velocity.abs() >= self.config.tolerance_velocity)
        {
            // Get the current position
            real_heading = inertial.heading().context(InertialSnafu)?;
            heading = real_heading - heading_difference;
            if real_heading > 300.0 {
                inertial
                    .set_heading(real_heading - 200.0)
                    .context(InertialSnafu)?;
                heading_difference -= 200.0;
            } else if real_heading < 60.0 {
                inertial
                    .set_heading(real_heading + 200.0)
                    .context(InertialSnafu)?;
                heading_difference += 200.0;
            }

            // Get the current velocity
            left_velocity = self.left.velocity().context(MotorSnafu)?;
            right_velocity = self.right.velocity().context(MotorSnafu)?;

            if left_velocity.abs() > self.config.tolerance_velocity
                || right_velocity.abs() > self.config.tolerance_velocity
            {
                last_moving_instant = Instant::now();
            }
            if last_moving_instant.elapsed() > self.config.timeout {
                println!("Drivetrain.turn_for timed out");
                break;
            }

            // Calculate the output
            let output = controller.next_control_output(heading).output;
            dbg!(real_heading, heading, output);

            // Set the output
            self.left
                .set_voltage(output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .context(MotorSnafu)?;
            self.right
                .set_voltage(-output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .context(MotorSnafu)?;

            sleep(Motor::UPDATE_INTERVAL).await;
        }

        // brake the left and right motors
        self.left
            .brake(vexide::prelude::BrakeMode::Hold)
            .context(MotorSnafu)?;
        self.right
            .brake(vexide::prelude::BrakeMode::Hold)
            .context(MotorSnafu)?;
        Ok(())
    }

    pub fn temperature(&self) -> f64 {
        let left_temp = self.left.temperature();
        let right_temp = self.right.temperature();
        if left_temp.is_err() || right_temp.is_err() {
            return 0.0;
        }
        (left_temp.unwrap() + right_temp.unwrap()) / 2.0
    }
}
