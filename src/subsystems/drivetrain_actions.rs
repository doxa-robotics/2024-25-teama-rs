use core::{f64::consts::PI, time::Duration};

use libdoxa::{
    path_planner::cubic_parametric::CubicParametricPath,
    subsystems::drivetrain::actions::config::ActionConfig, utils::pose::Pose,
};

pub const TILES_TO_MM: f64 = 600.0;

pub const CONFIG: ActionConfig = ActionConfig {
    linear_kp: 2.0,
    linear_kp_limit: f64::MAX,
    linear_ki: 0.0,
    linear_ki_limit: f64::MAX,
    linear_kd: 6.0,
    linear_kd_limit: f64::MAX,
    linear_limit: 450.0,
    turn_kp: 200.0,
    turn_kp_limit: f64::MAX,
    turn_ki: 10.0,
    turn_ki_limit: 1.0,
    turn_kd: 450.0,
    turn_kd_limit: f64::MAX,
    turn_limit: 450.0,
    boomerang_lock_distance: None,
    pursuit_turn_kp: 500.0,
    pursuit_turn_kp_limit: f64::MAX,
    pursuit_turn_ki: 0.0,
    pursuit_turn_ki_limit: 1.0,
    pursuit_turn_kd: 300.0,
    pursuit_turn_kd_limit: f64::MAX,
    pursuit_turn_limit: 450.0,
    pursuit_lookahead: 200.0,
    linear_error_tolerance: 15.0,
    linear_velocity_tolerance: 200.0,
    linear_tolerance_duration: Duration::from_millis(0),
    linear_timeout: Duration::from_millis(1500),
    turn_error_tolerance: 0.1,
    turn_velocity_tolerance: 200.0,
    turn_tolerance_duration: Duration::from_millis(0),
    turn_timeout: Duration::from_millis(1000),
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

#[allow(unused)]
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
