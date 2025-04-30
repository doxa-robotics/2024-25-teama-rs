use core::{f64::consts::PI, time::Duration};

use libdoxa::{
    path_planner::cubic_parametric::CubicParametricPath,
    subsystems::drivetrain::actions::config::ActionConfig, utils::pose::Pose,
};
use vexide::prelude::Motor;

pub const TILES_TO_MM: f64 = 600.0;

pub const CONFIG: ActionConfig = ActionConfig {
    linear_kp: 0.1,
    linear_kp_limit: f64::MAX,
    linear_ki: 0.00001,
    linear_ki_limit: 0.6,
    linear_kd: 0.2,
    linear_kd_limit: f64::MAX,
    linear_limit: Motor::V5_MAX_VOLTAGE * 0.67,
    turn_kp: 35.0,
    turn_kp_limit: f64::MAX,
    turn_ki: 0.001,
    turn_ki_limit: 1.0,
    turn_kd: 250.0,
    turn_kd_limit: f64::MAX,
    turn_limit: Motor::V5_MAX_VOLTAGE,
    pursuit_turn_kp: 50.0,
    pursuit_turn_kp_limit: f64::MAX,
    pursuit_turn_ki: 0.00001,
    pursuit_turn_ki_limit: 1.0,
    pursuit_turn_kd: 13.0,
    pursuit_turn_kd_limit: f64::MAX,
    pursuit_turn_limit: Motor::V5_MAX_VOLTAGE,
    pursuit_lookahead: 200.0,
    linear_error_tolerance: 15.0,
    linear_velocity_tolerance: 5.0,
    linear_tolerance_duration: Duration::from_millis(100),
    linear_timeout: Duration::from_millis(2000),
    turn_error_tolerance: 0.2,
    turn_velocity_tolerance: 10.0,
    turn_tolerance_duration: Duration::from_millis(100),
    turn_timeout: Duration::from_millis(1500),
};

pub fn forward(
    distance_tiles: f64,
    config: ActionConfig,
) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::ForwardAction::new(
        distance_tiles * TILES_TO_MM,
        config,
    )
}

pub fn turn_to_point(
    point: Pose,
    config: ActionConfig,
) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::TurnToPointAction::new(
        point * TILES_TO_MM,
        false,
        config,
    )
}

pub fn drive_to_point(
    point: Pose,
    reverse: bool,
    config: ActionConfig,
) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::DriveToPointAction::new(
        point * TILES_TO_MM,
        reverse,
        config,
    )
}

pub fn boomerang_to_point(
    point: Pose,
    config: ActionConfig,
) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::BoomerangAction::new(point * TILES_TO_MM, config)
}

pub fn smooth_to_point(
    point: Pose,
    start_easing: f64,
    end_easing: f64,
    reverse: bool,
    disable_seeking_distance: Option<f64>,
    config: ActionConfig,
) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::LazyAction::new(move |current_pose| {
        libdoxa::subsystems::drivetrain::actions::PurePursuitAction::new(
            CubicParametricPath::new(
                Pose {
                    offset: current_pose.offset,
                    heading: if reverse {
                        current_pose.heading - PI
                    } else {
                        current_pose.heading
                    },
                },
                start_easing * TILES_TO_MM,
                point * TILES_TO_MM,
                end_easing * TILES_TO_MM,
            ),
            disable_seeking_distance,
            config,
        )
    })
}
