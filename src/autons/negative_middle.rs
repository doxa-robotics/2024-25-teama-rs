use core::time::Duration;

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
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            0.255,
            CONFIG.with_linear_limit(300.0),
        ))
        .await;
    sleep(Duration::from_micros(100)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    sleep(Duration::from_millis(700)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::Initial);

    // Get goal at (1.0, -1.0)
    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-1.1, -1.0).into(),
            true,
            CONFIG
                .with_linear_error_tolerance(100.0)
                .with_turn_error_tolerance(0.2),
        ))
        .with_callback(move |pose| {
            if pose.y() > -1.1 * TILES_TO_MM {
                clamp.extend();
            }
        })
        .await;

    // Get center rings
    robot.doinker.non_dominant().extend();
    robot.intake_raiser.extend();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-0.49, -0.41).into(),
            false,
            CONFIG
                .with_linear_error_tolerance(20.0)
                .with_linear_limit(300.0),
        ))
        .await;
    sleep(Duration::from_millis(300)).await;
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-1.5, -1.5).into(),
            true,
            CONFIG
                .with_turn_error_tolerance(0.3)
                .with_linear_error_tolerance(200.0),
        ))
        .await;
    robot.intake_raiser.retract();
    robot.doinker.non_dominant().retract();
    robot.intake.run(vexide::prelude::Direction::Forward);
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-1.3, -1.0).into(),
            false,
            CONFIG.with_linear_error_tolerance(150.0),
        ))
        .await;
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
            (-2.9, -2.7).into(),
            false,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .with_callback(move |pose| {
            if pose.y() < -2.1 * TILES_TO_MM {
                intake.run(vexide::prelude::Direction::Reverse);
            }
        })
        .await;
    robot.intake.run(vexide::prelude::Direction::Forward);
    sleep(Duration::from_millis(500)).await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            -0.2,
            CONFIG
                .with_linear_error_tolerance(100.0)
                .with_linear_velocity_tolerance(200.0)
                .with_linear_tolerance_duration(Duration::ZERO),
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.2, CONFIG))
        .await;
    sleep(Duration::from_millis(600)).await;
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
