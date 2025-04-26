use core::{f64::consts::FRAC_PI_2, time::Duration};

use vexide::{
    prelude::{Direction, Motor},
    task::spawn,
    time::sleep,
};

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG},
    Robot,
};

async fn route(robot: &mut Robot) {
    //Starting position
    robot
        .tracking
        .borrow_mut()
        .set_pose((-600.0 * 2.0 - 350.0, -600.0 * 2.0 - 120.0, FRAC_PI_2).into());

    //doinker

    robot.doinker.non_dominant().extend();
    robot.intake.partial_intake();
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (-2.3, -0.4, 1.0).into(),
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
            (-1.5, -0.4).into(),
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

    //run intake and extend clamp
    // robot
    //     .drivetrain
    //     .action(drivetrain_actions::smooth_to_point(
    //         (-1.0, -1.0, 0.0).into(),
    //         2.0,
    //         2.0,
    //         true,
    //         false,
    //         CONFIG,
    //     ))
    //     .await;
    return;

    robot
        .drivetrain
        .action(drivetrain_actions::forward(-0.3, CONFIG))
        .await;
    //unclamp, clamp and doinker. Start intake.
    robot.clamp.retract();
    robot.doinker.dominant().retract();
    robot.intake.run(Direction::Forward);

    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (-2.0, -1.0, 3.92).into(),
            1.0,
            1.0,
            false,
            false,
            CONFIG,
        ))
        .await;

    robot.intake.stop_hold();
    robot.intake_raiser.extend();
    robot.clamp.extend();

    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (0.0, -2.0, 0.0).into(),
            1.0,
            1.0,
            false,
            false,
            CONFIG,
        ))
        .await;
    robot.intake.partial_intake();
    robot.intake_raiser.retract();

    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (-0.0, -3.0, -1.57).into(),
            1.0,
            1.0,
            false,
            false,
            CONFIG,
        ))
        .await;

    robot
        .drivetrain
        .action(drivetrain_actions::forward(-2.0, CONFIG))
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
}

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    route(robot).await;
}
