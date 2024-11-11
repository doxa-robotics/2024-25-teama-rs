use vexide::prelude::Motor;

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
}

/// A drivetrain struct
///
/// This struct is used to control a drivetrain, which contains two MotorGroups, one for the left
/// side and one for the right side. This struct is used to control the drivetrain as a whole.
///
/// It supports a generic `drive_for` method which allows the robot to drive for a certain distance
/// using a PID controller from the Rust `pid` crate.
pub struct Drivetrain {
    left: MotorGroup,
    right: MotorGroup,
    config: DrivetrainConfig,
}

impl Drivetrain {
    /// Create a new Drivetrain
    pub fn new(left: MotorGroup, right: MotorGroup, config: DrivetrainConfig) -> Drivetrain {
        Drivetrain {
            left,
            right,
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

    /// Drive the robot for a certain distance
    ///
    /// This method uses a PID controller to drive the robot for a certain distance. It is a blocking (async)
    /// method, and will not return until the robot has driven the specified distance.
    ///
    /// # Arguments
    ///
    /// - `target_distance`: The distance to drive in mm
    pub async fn drive_for(&mut self, target_distance: f64) {
        let mut controller: pid::Pid<f64> = pid::Pid::new(target_distance, Motor::V5_MAX_VOLTAGE)
        controller
            .p(self.config.drive_p, 1.0)
            .i(self.config.drive_i, 1.0)
            .d(self.config.drive_d, 1.0);

        // Get the initial position
        let left_initial = self.left.position();
    }
}
