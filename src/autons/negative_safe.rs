use core::{f64::consts::PI, time::Duration};

use vexide::time::sleep;

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG, TILES_TO_MM},
    Robot,
};

async fn route(robot: &mut Robot) {
    robot
        .tracking
        .borrow_mut()
        .set_pose((-280.0, -1360.0, -0.74).into());

    // Alliance score
    // FIXME: uncomment after Q89
    // robot
    //     .drivetrain
    //     .action(drivetrain_actions::forward(
    //         0.255,
    //         CONFIG.with_linear_limit(300.0),
    //     ))
    //     .await;
    // sleep(Duration::from_millis(100)).await;
    // robot
    //     .lady_brown
    //     .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    // sleep(Duration::from_millis(700)).await;
    // robot
    //     .lady_brown
    //     .set_state(crate::subsystems::lady_brown::LadyBrownState::Initial);

    // Get goal at (1.0, -1.0)
    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-1.1, -1.0).into(),
            true,
            CONFIG
                .with_linear_error_tolerance(100.0)
                .with_turn_error_tolerance(0.1),
        ))
        .with_callback(move |pose| {
            if pose.y() > -1.1 * TILES_TO_MM {
                clamp.extend();
            }
        })
        .await;

    // Ring at (-2.0, -1.0)
    robot.intake.run(vexide::prelude::Direction::Forward);
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-2.0, -1.0).into(),
            false,
            CONFIG,
        ))
        .await;
    let intake = robot.intake.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-2.8, -2.8).into(),
            false,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .with_callback(move |pose| {
            if pose.y() < -2.0 * TILES_TO_MM {
                intake.run(vexide::prelude::Direction::Reverse);
            }
        })
        .await;
    robot.intake.run(vexide::prelude::Direction::Forward);
    sleep(Duration::from_millis(500)).await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            -0.3,
            CONFIG
                .with_linear_error_tolerance(100.0)
                .with_linear_velocity_tolerance(200.0)
                .with_linear_tolerance_duration(Duration::ZERO),
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            0.3,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    sleep(Duration::from_millis(500)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-0.2, -0.9).into(),
            false,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::turn_to_point(
            (0.2, -0.2).into(),
            CONFIG,
        ))
        .await;
}

pub async fn blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Blue));
    route(robot).await;
}

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    route(robot).await;
}
