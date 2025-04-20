use core::time::Duration;

use libdoxa::{
    path_planner::cubic_parametric::CubicParametricPath,
    utils::{pose::Pose, settling::Tolerances},
};
use pid::Pid;
use vexide::prelude::Motor;

const TILES_TO_MM: f64 = 600.0;

pub struct ActionConfig {
    pub linear_kp: f64,
    pub linear_ki: f64,
    pub linear_kd: f64,

    pub turn_kp: f64,
    pub turn_ki: f64,
    pub turn_kd: f64,

    pub pursuit_turn_kp: f64,
    pub pursuit_turn_ki: f64,
    pub pursuit_turn_kd: f64,
}

pub fn forward(distance_tiles: f64) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::ForwardAction::new(
        {
            let mut controller = Pid::new(distance_tiles * TILES_TO_MM, Motor::V5_MAX_VOLTAGE);
            controller.p(0.1, Motor::V5_MAX_VOLTAGE);
            controller
        },
        Tolerances::new()
            .error_tolerance(5.0)
            .velocity_tolerance(5.0)
            .tolerance_duration(Duration::from_millis(200))
            .timeout(Duration::from_millis(1000)),
    )
}

pub fn turn_to_point(point: Pose) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::TurnToPointAction::new(
        point,
        {
            let mut controller = Pid::new(0.0, Motor::V5_MAX_VOLTAGE);
            controller.p(0.1, Motor::V5_MAX_VOLTAGE);
            controller
        },
        Tolerances::new()
            .error_tolerance(5.0)
            .velocity_tolerance(5.0)
            .tolerance_duration(Duration::from_millis(200))
            .timeout(Duration::from_millis(1000)),
    )
}

pub fn drive_to_point(point: Pose) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::DriveToPointAction::new(
        point,
        {
            let mut controller = Pid::new(0.0, Motor::V5_MAX_VOLTAGE);
            controller.p(0.1, Motor::V5_MAX_VOLTAGE);
            controller
        },
        {
            let mut controller = Pid::new(0.0, Motor::V5_MAX_VOLTAGE);
            controller.p(0.1, Motor::V5_MAX_VOLTAGE);
            controller
        },
        Tolerances::new()
            .error_tolerance(5.0)
            .velocity_tolerance(5.0)
            .tolerance_duration(Duration::from_millis(200))
            .timeout(Duration::from_millis(1000)),
        Tolerances::new()
            .error_tolerance(5.0)
            .velocity_tolerance(5.0)
            .tolerance_duration(Duration::from_millis(200))
            .timeout(Duration::from_millis(1000)),
    )
}

pub fn smooth_to_point(
    point: Pose,
    start_easing: f64,
    end_easing: f64,
) -> impl libdoxa::subsystems::drivetrain::actions::Action {
    libdoxa::subsystems::drivetrain::actions::LazyAction::new(move |current_pose| {
        libdoxa::subsystems::drivetrain::actions::PurePursuitAction::new(
            CubicParametricPath::new(current_pose, start_easing, point, end_easing),
            {
                let mut controller = Pid::new(0.0, Motor::V5_MAX_VOLTAGE);
                controller.p(0.1, Motor::V5_MAX_VOLTAGE);
                controller
            },
            {
                let mut controller = Pid::new(0.0, Motor::V5_MAX_VOLTAGE);
                controller.p(0.1, Motor::V5_MAX_VOLTAGE);
                controller
            },
            {
                let mut controller = Pid::new(0.0, Motor::V5_MAX_VOLTAGE);
                controller.p(0.1, Motor::V5_MAX_VOLTAGE);
                controller
            },
            Tolerances::new()
                .error_tolerance(5.0)
                .velocity_tolerance(5.0)
                .tolerance_duration(Duration::from_millis(200))
                .timeout(Duration::from_millis(1000)),
            100.0,
            Motor::V5_MAX_VOLTAGE / 2.0,
        )
    })
}
