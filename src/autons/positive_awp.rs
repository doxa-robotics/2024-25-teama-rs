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
        .set_pose((280.0, -1360.0, 0.74 - PI).into());

    // Alliance score
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            0.255,
            CONFIG.with_linear_limit(300.0),
        ))
        .await;
    sleep(Duration::from_millis(100)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    sleep(Duration::from_millis(800)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::Initial);
    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (1.15, -0.8).into(),
            true,
            CONFIG
                .with_linear_error_tolerance(200.0)
                .with_linear_velocity_tolerance(600.0),
        ))
        .with_callback(move |pose| {
            if pose.y() > -1.1 * TILES_TO_MM {
                clamp.extend();
            }
        })
        .await;
    robot.intake.run(vexide::prelude::Direction::Forward);

    // Get ring at (2.0, -1.0)
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.1, -1.0).into(),
            false,
            CONFIG
                .with_linear_error_tolerance(200.0)
                .with_linear_velocity_tolerance(600.0),
        ))
        .await;
    sleep(Duration::from_millis(200)).await;

    // Get the ring in front of alliance stake
    let mut intake_raiser = robot.intake_raiser.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-0.2, -1.9).into(),
            false,
            CONFIG
                .with_linear_error_tolerance(200.0)
                .with_linear_velocity_tolerance(600.0),
        ))
        .with_callback(move |pose| {
            if pose.x() < 0.2 * TILES_TO_MM {
                intake_raiser.retract();
            } else if pose.x() < 1.5 * TILES_TO_MM {
                intake_raiser.extend();
            }
        })
        .await;

    // Drive to bar touch
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (0.0, 0.9).into(),
            false,
            CONFIG
                .with_turn_error_tolerance(0.1)
                .with_linear_velocity_tolerance(500.0)
                .with_linear_limit(400.0)
                .with_linear_error_tolerance(100.0),
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
