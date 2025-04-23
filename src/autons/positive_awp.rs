use core::{
    f64::consts::{FRAC_PI_2, FRAC_PI_4},
    time::Duration,
};

use vexide::{task::spawn, time::sleep};

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG, TILES_TO_MM},
    Robot,
};

async fn route(robot: &mut Robot) {
    robot
        .tracking
        .borrow_mut()
        .set_pose((0.5 * TILES_TO_MM, -2.7 * TILES_TO_MM, 0.0).into());

    // Alliance score
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    sleep(Duration::from_millis(700)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::Initial);

    // Corner
    let intake_clone = robot.intake.clone();
    spawn(async move {
        sleep(Duration::from_millis(1000)).await;
        intake_clone.partial_intake();
    })
    .detach();
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (2.6, -2.6, -FRAC_PI_4).into(),
            2.0,
            3.0,
            CONFIG.with_linear_error_tolerance(100.0),
            false,
        ))
        .await;

    // Get goal
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (1.0, -1.5, FRAC_PI_2).into(),
            2.0,
            4.0,
            CONFIG,
            true,
        ))
        .await;
    robot.clamp.extend();
    robot.intake.run(vexide::prelude::Direction::Forward);

    // Get ring at (2.0, -1.0)
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.3, -1.0).into(),
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    sleep(Duration::from_millis(700)).await;

    // Get the ring in front of alliance stake
    robot.intake_raiser.extend();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (0.0, -2.0).into(),
            CONFIG,
        ))
        .await;

    // Drive to bar touch
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    robot
        .drivetrain
        .action(drivetrain_actions::boomerang_to_point(
            (0.0, -1.2, 0.0).into(),
            CONFIG,
        ))
        .await;

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
