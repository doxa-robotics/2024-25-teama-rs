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
        .set_pose((360.0, -1400.0, 0.74 - PI).into());

    // Alliance score
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.25, CONFIG))
        .await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    sleep(Duration::from_millis(700)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::Initial);
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (1.1, -2.0).into(),
            true,
            CONFIG,
        ))
        .await;
    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (1.1, -0.9).into(),
            true,
            CONFIG, // CONFIG.with_linear_limit(Motor::V5_MAX_VOLTAGE * 0.6),
        ))
        .with_callback(move |pose| {
            if pose.y() > -1.3 * TILES_TO_MM {
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
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    sleep(Duration::from_millis(200)).await;

    // Get the ring in front of alliance stake
    let mut intake_raiser = robot.intake_raiser.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-0.2, -2.1).into(),
            false,
            CONFIG.with_linear_error_tolerance(50.0),
        ))
        .with_callback(move |pose| {
            if pose.x() < -1.5 * TILES_TO_MM {
                intake_raiser.extend();
            } else if pose.x() < 0.2 * TILES_TO_MM {
                intake_raiser.retract();
            }
        })
        .await;

    // Drive to bar touch
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (0.0, -1.45).into(),
            false,
            CONFIG
                .with_turn_error_tolerance(0.2)
                .with_linear_error_tolerance(100.0),
        ))
        .await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);

    // TODO: touch
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
