use core::{
    f64::consts::{FRAC_PI_2, PI},
    time::Duration,
};

use vexide::{
    prelude::{Direction, Motor},
    task::spawn,
    time::sleep,
};

use crate::{
    subsystems::drivetrain_actions::{self, set_reverse, CONFIG},
    Robot,
};

async fn route(robot: &mut Robot) {
    //Starting position
    // robot
    //     .tracking
    //     .borrow_mut()
    //     .set_pose((-600.0 * 2.0 - 350.0, -600.0 * 2.0 - 120.0, FRAC_PI_2).into());

    //doinker

    robot.doinker.non_dominant().extend();
    robot.intake.partial_intake();
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (-2.3, -0.6, 1.0).into(),
            4.0,
            2.0,
            false,
            true,
            CONFIG.with_linear_limit(Motor::V5_MAX_VOLTAGE * 0.7),
        ))
        .await;
    robot.doinker.non_dominant().retract();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-1.5, -0.7).into(),
            false,
            CONFIG,
        ))
        .await;
    let mut clamp_clone = robot.clamp.clone();
    spawn(async move {
        sleep(Duration::from_millis(2000)).await;
        clamp_clone.extend();
    })
    .detach();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-0.5, -1.5).into(),
            true,
            CONFIG,
        ))
        .await;
    robot.clamp.extend();
    robot.intake.run(Direction::Forward);
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-2.0, -2.4).into(),
            false,
            CONFIG,
        ))
        .await;
    sleep(Duration::from_millis(600)).await;
    robot.intake_raiser.extend();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-2.7, -2.7).into(),
            false,
            CONFIG,
        ))
        .await;
}

pub async fn blue(robot: &mut Robot) {
    set_reverse(true);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Blue));
    robot
        .doinker
        .set_mirrored_state(libdoxa::subsystems::pneumatic::MirroredState::Mirrored);
    robot
        .tracking
        .borrow_mut()
        .set_pose((-600.0 * 2.0 - 350.0, 600.0 * 2.0 - 120.0, PI - FRAC_PI_2).into());
    route(robot).await;
}

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    robot
        .tracking
        .borrow_mut()
        .set_pose((-600.0 * 2.0 - 350.0, -600.0 * 2.0 - 120.0, FRAC_PI_2).into());
    route(robot).await;
}
