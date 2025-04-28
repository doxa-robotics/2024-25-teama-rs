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
        .set_pose((-360.0, -1400.0, -0.74).into());

    // Alliance score
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.24, CONFIG))
        .await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    sleep(Duration::from_millis(800)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::Initial);

    // Get goal at (1.0, -1.0)
    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-1.2, -0.8).into(),
            true,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .with_callback(move |pose| {
            if pose.y() > -1.1 * TILES_TO_MM {
                clamp.extend();
            }
        })
        .await;

    // Get center rings
    robot.doinker.non_dominant().extend();
    robot.intake.run(vexide::prelude::Direction::Forward);
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-0.74, -0.41).into(),
            false,
            CONFIG,
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(-0.15, CONFIG))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::turn_to_point((0.7, 4.0).into(), CONFIG))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.1, CONFIG))
        .await;
    robot.doinker.dominant().extend();
    sleep(Duration::from_millis(500)).await;
    return;

    // Get ring at (-2.0, -1.0)
    robot.intake.run(vexide::prelude::Direction::Forward);
    robot
        .drivetrain
        .action(drivetrain_actions::turn_to_point(
            (-2.0, -1.5).into(),
            CONFIG,
        ))
        .await;
    robot.doinker.dominant().retract();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-2.1, -1.0).into(),
            false,
            CONFIG.with_linear_error_tolerance(100.0),
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
