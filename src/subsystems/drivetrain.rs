use alloc::sync::Arc;
use core::time::Duration;

use log::{debug, info, warn};
use snafu::{ResultExt, Snafu};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::smart::{imu::InertialError, motor::MotorError},
    prelude::{sleep, spawn, Float, InertialSensor, Motor, SmartDevice},
};

use super::super::utils::motor_group::MotorGroup;

const SIGN_ERROR_CHECK_DELAY: Duration = Duration::from_millis(100);

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
    left: Arc<Mutex<MotorGroup>>,
    right: Arc<Mutex<MotorGroup>>,
    inertial: Arc<Mutex<InertialSensor>>,
    config: DrivetrainConfig,
    negate_turns: bool,
    negate_turn_output: bool,
}

impl Drivetrain {
    /// Create a new Drivetrain
    pub fn new(
        left: Arc<Mutex<MotorGroup>>,
        right: Arc<Mutex<MotorGroup>>,
        inertial: Arc<Mutex<InertialSensor>>,
        config: DrivetrainConfig,
    ) -> Drivetrain {
        Drivetrain {
            left,
            right,
            inertial,
            config,
            negate_turns: false,
            negate_turn_output: false,
        }
    }

    pub async fn set_voltages(&mut self, left: f64, right: f64) -> Result<(), DrivetrainError> {
        self.left
            .lock()
            .await
            .set_voltage(left)
            .context(MotorSnafu)?;
        self.right
            .lock()
            .await
            .set_voltage(right)
            .context(MotorSnafu)?;
        Ok(())
    }

    pub fn set_negate_turns(&mut self, negate: bool) {
        self.negate_turns = negate;
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
        self.drive_for_advanced(target_distance, 1.0).await
    }
    pub async fn drive_for_advanced(
        &mut self,
        target_distance: f64,
        p_multiplier: f64,
    ) -> Result<(), DrivetrainError> {
        let drive_start = Instant::now();

        let mut left_distance: f64 = self
            .left
            .lock()
            .await
            .position()
            .context(MotorSnafu)?
            .as_revolutions()
            * self.config.wheel_circumference;
        let mut right_distance: f64 = self
            .right
            .lock()
            .await
            .position()
            .context(MotorSnafu)?
            .as_revolutions()
            * self.config.wheel_circumference;
        let mut left_velocity = self.left.lock().await.velocity().context(MotorSnafu)?;
        let mut right_velocity = self.right.lock().await.velocity().context(MotorSnafu)?;

        let mut left_controller: pid::Pid<f64> =
            pid::Pid::new(left_distance + target_distance, Motor::V5_MAX_VOLTAGE);
        left_controller
            .p(self.config.drive_p * p_multiplier, f64::MAX)
            .i(self.config.drive_i, f64::MAX)
            .d(self.config.drive_d, f64::MAX);
        let mut right_controller: pid::Pid<f64> =
            pid::Pid::new(right_distance + target_distance, Motor::V5_MAX_VOLTAGE);
        right_controller
            .p(self.config.drive_p * p_multiplier, f64::MAX)
            .i(self.config.drive_i, f64::MAX)
            .d(self.config.drive_d, f64::MAX);

        let mut last_moving_instant = Instant::now();
        while ((left_distance - left_controller.setpoint).abs() >= self.config.drive_tolerance
            || (right_distance - right_controller.setpoint).abs() >= self.config.drive_tolerance)
            || (left_velocity.abs() >= self.config.tolerance_velocity
                || right_velocity.abs() >= self.config.tolerance_velocity)
        {
            let mut left = self.left.lock().await;
            let mut right = self.right.lock().await;
            // Get the current position
            left_distance = left.position().context(MotorSnafu)?.as_revolutions()
                * self.config.wheel_circumference;
            right_distance = right.position().context(MotorSnafu)?.as_revolutions()
                * self.config.wheel_circumference;

            // Get the current velocity
            left_velocity = left.velocity().context(MotorSnafu)?;
            right_velocity = right.velocity().context(MotorSnafu)?;

            if left_velocity.abs() > self.config.tolerance_velocity
                || right_velocity.abs() > self.config.tolerance_velocity
            {
                last_moving_instant = Instant::now();
            }
            if last_moving_instant.elapsed() > self.config.timeout {
                warn!("Drivetrain.drive_for timed out");
                drop(left);
                drop(right);
                break;
            }

            // Calculate the output
            let left_output = left_controller.next_control_output(left_distance).output;
            let right_output = right_controller.next_control_output(right_distance).output;
            debug!(
                "left distance: {:?} / right distance: {:?} / left output: {:?} / right output: {:?} / target distance: {:?}",
                left_distance, right_distance, left_output, right_output, target_distance
            );

            // Set the output
            left.set_voltage(left_output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .context(MotorSnafu)?;
            right
                .set_voltage(right_output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .context(MotorSnafu)?;

            drop(left);
            drop(right);

            sleep(Motor::UPDATE_INTERVAL).await;
        }

        // brake the left and right motors
        self.left
            .lock()
            .await
            .brake(vexide::prelude::BrakeMode::Brake)
            .context(MotorSnafu)?;
        self.right
            .lock()
            .await
            .brake(vexide::prelude::BrakeMode::Brake)
            .context(MotorSnafu)?;

        log::info!(
            "drive_for finished in {}ms",
            drive_start.elapsed().as_millis()
        );
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
        let turn_start = Instant::now();
        let mut has_corrected_sign_error = false;

        let inertial = self.inertial.lock().await;

        // Get the initial position
        let mut heading = -inertial.rotation().context(InertialSnafu)?;
        let target_heading = heading - target_angle_delta;

        let mut left_velocity = self.left.lock().await.velocity().context(MotorSnafu)?;
        let mut right_velocity = self.right.lock().await.velocity().context(MotorSnafu)?;

        let mut controller: pid::Pid<f64> = pid::Pid::new(target_heading, Motor::V5_MAX_VOLTAGE);
        controller
            .p(self.config.turning_p, f64::MAX)
            .i(self.config.turning_i, f64::MAX)
            .d(self.config.turning_d, f64::MAX);

        let mut last_moving_instant = Instant::now();
        let initial_error = (heading - target_heading).abs();
        while (heading - target_heading).abs() >= self.config.turning_tolerance
            || (left_velocity.abs() >= self.config.tolerance_velocity
                || right_velocity.abs() >= self.config.tolerance_velocity)
        {
            let mut left = self.left.lock().await;
            let mut right = self.right.lock().await;

            // Check if the sign of the output is incorrect (i.e., the robot is turning the wrong way)
            // FIXME: this is a hacky way to fix the sign error, but it works for now :(
            if !has_corrected_sign_error && turn_start.elapsed() > SIGN_ERROR_CHECK_DELAY {
                if (heading - target_heading).abs() > initial_error {
                    self.negate_turn_output = !self.negate_turn_output;
                    warn!("Drivetrain.turn_for correcting sign error by negating output as error has increased in 100ms");
                }
                has_corrected_sign_error = true;
            }

            // Get the current position
            heading = -inertial.rotation().context(InertialSnafu)?;

            // Get the current velocity
            left_velocity = left.velocity().context(MotorSnafu)?;
            right_velocity = right.velocity().context(MotorSnafu)?;

            if left_velocity.abs() > self.config.tolerance_velocity
                || right_velocity.abs() > self.config.tolerance_velocity
            {
                last_moving_instant = Instant::now();
            }
            if last_moving_instant.elapsed() > self.config.timeout {
                warn!("Drivetrain.turn_for timed out");
                drop(left);
                drop(right);
                break;
            }

            // Calculate the output
            let mut output = controller.next_control_output(heading).output;
            if self.negate_turn_output {
                // FIXME: this is still hacky
                output *= -1.0;
            }

            debug!(
                "heading: {:?} / rotation: {:?} / output: {:?} / target heading: {:?} / physical orientation: {:?}",
                inertial.heading().unwrap_or(0.0),
                heading,
                output,
                target_heading,
                inertial.physical_orientation()
            );

            // Set the output
            left.set_voltage(-output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .context(MotorSnafu)?;
            right
                .set_voltage(output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE))
                .context(MotorSnafu)?;

            drop(left);
            drop(right);

            sleep(Motor::UPDATE_INTERVAL).await;
        }

        // brake the left and right motors
        self.left
            .lock()
            .await
            .brake(vexide::prelude::BrakeMode::Brake)
            .context(MotorSnafu)?;
        self.right
            .lock()
            .await
            .brake(vexide::prelude::BrakeMode::Brake)
            .context(MotorSnafu)?;

        log::info!(
            "turn_for finished in {}ms",
            turn_start.elapsed().as_millis()
        );

        Ok(())
    }

    /// Turn the robot to a certain angle, using turn_for under the hood.
    pub async fn turn_to(&mut self, mut target_angle: f64) -> Result<(), DrivetrainError> {
        if self.negate_turns {
            target_angle = -target_angle;
        }

        let inertial = self.inertial.lock().await;
        let current_angle = inertial.heading().context(InertialSnafu)?;
        drop(inertial);
        // Calculate the angle delta for the closest turn
        let mut angle_delta = target_angle - current_angle;
        if angle_delta > 180.0 {
            angle_delta -= 360.0;
        } else if angle_delta < -180.0 {
            angle_delta += 360.0;
        }
        info!(
            "starting from {}, turning to {}, delta: {}",
            current_angle, target_angle, angle_delta
        );
        self.turn_for(angle_delta).await
    }

    pub async fn reset_inertial(&mut self, heading: f64) -> Result<(), DrivetrainError> {
        self.inertial
            .lock()
            .await
            .set_heading(heading)
            .context(InertialSnafu)?;
        Ok(())
    }

    pub fn temperature(&self) -> f64 {
        let left_temp = self.left.try_lock().and_then(|x| x.temperature().ok());
        let right_temp = self.right.try_lock().and_then(|x| x.temperature().ok());
        if left_temp.is_none() || right_temp.is_none() {
            return 0.0;
        }
        (left_temp.unwrap() + right_temp.unwrap()) / 2.0
    }
}
