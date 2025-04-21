use core::time::Duration;

use libdoxa::{
    path_planner::cubic_parametric::CubicParametricPath,
    subsystems::drivetrain::actions::config::ActionConfig, utils::pose::Pose,
};

const TILES_TO_MM: f64 = 600.0;

pub const CONFIG: ActionConfig = ActionConfig {
    linear_kp: 0.0,
    linear_ki: 0.0,
    linear_kd: 0.0,
    turn_kp: 0.0,
    turn_ki: 0.0,
    turn_kd: 0.0,
    pursuit_turn_kp: 0.0,
    pursuit_turn_ki: 0.0,
    pursuit_turn_kd: 0.0,
    pursuit_lookahead: 0.0,
    linear_error_tolerance: 0.0,
    linear_velocity_tolerance: 0.0,
    linear_tolerance_duration: Duration::from_millis(0),
    linear_timeout: Duration::from_millis(0),
    turn_error_tolerance: 0.0,
    turn_velocity_tolerance: 0.0,
    turn_tolerance_duration: Duration::from_millis(0),
    turn_timeout: Duration::from_millis(0),
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
    libdoxa::subsystems::drivetrain::actions::TurnToPointAction::new(point * TILES_TO_MM, config)
}

pub fn drive_to_point(
    point: Pose,
    config: ActionConfig,
) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::DriveToPointAction::new(point * TILES_TO_MM, config)
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
    config: ActionConfig,
) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::LazyAction::new(move |current_pose| {
        libdoxa::subsystems::drivetrain::actions::PurePursuitAction::new(
            CubicParametricPath::new(
                current_pose,
                start_easing * TILES_TO_MM,
                point * TILES_TO_MM,
                end_easing * TILES_TO_MM,
            ),
            config,
        )
    })
}
