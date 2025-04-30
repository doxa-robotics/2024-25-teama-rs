use core::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use vexide::prelude::Motor;

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG},
    Robot,
};

async fn route(robot: &mut Robot) {
    // Starting position
    robot
        .tracking
        .borrow_mut()
        .set_pose((600.0 * 2.0 + 350.0, -600.0 * 2.0 - 120.0, FRAC_PI_2).into());

    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (2.3, -0.4, FRAC_PI_4 * 3.0).into(),
            1.0,
            2.0,
            true,
            Some(400.0),
            CONFIG.with_linear_limit(Motor::V5_MAX_VOLTAGE * 0.7),
        ))
        .with_callback(move |pose| {
            if pose.y() > -0.6 {
                clamp.extend();
            }
        })
        .await;

    robot.intake.run(vexide::prelude::Direction::Forward);

    // get the ring before the goal
    let intake = robot.intake.clone();
    let mut intake_running_flag = false;
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.0, -1.0).into(),
            false,
            CONFIG,
        ))
        .with_callback(move |pose| {
            if !intake_running_flag && pose.y() < -0.6 {
                intake_running_flag = true;
                intake.partial_intake();
            }
        })
        .await;

    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (1.0, -1.0).into(),
            true,
            CONFIG,
        ))
        .with_callback(move |pose| {
            if pose.x() < 1.5 {
                clamp.extend();
            }
        })
        .await;
}

pub async fn blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Blue));
    robot
        .doinker
        .set_mirrored_state(libdoxa::subsystems::pneumatic::MirroredState::Mirrored);
    route(robot).await;
}

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    robot
        .doinker
        .set_mirrored_state(libdoxa::subsystems::pneumatic::MirroredState::Normal);
    route(robot).await;
}
