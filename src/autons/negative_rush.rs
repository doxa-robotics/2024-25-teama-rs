use core::f64::consts::FRAC_PI_2;

use vexide::prelude::Direction;

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG},
    Robot,
};

async fn route(robot: &mut Robot) {
    //Starting position
    robot
        .tracking
        .borrow_mut()
        .set_pose((-1620_f64, -1632_f64, FRAC_PI_2).into());
    robot
        .drivetrain
        .action(drivetrain_actions::forward(1.75, CONFIG))
        .await;

    //doinker

    robot.doinker.dominant().extend();
    //
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (-2.0, -0.4, 0.78).into(),
            2.0,
            4.0,
            false,
            true,
            CONFIG,
        ))
        .await;

    //run intake and extend clamp
    robot.intake.partial_intake();
    robot.clamp.extend();
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (-1.25, -0.6, 2.36).into(),
            1.0,
            1.0,
            false,
            false,
            CONFIG,
        ))
        .await;

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
    route(robot).await;
}

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    route(robot).await;
}
