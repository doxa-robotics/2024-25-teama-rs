use core::f64::consts::FRAC_PI_2;

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG},
    Robot,
};

pub async fn red(robot: &mut Robot) {
    robot
        .tracking
        .borrow_mut()
        .set_pose((-1620_f64, -1632_f64, FRAC_PI_2).into());
    robot
        .drivetrain
        .action(drivetrain_actions::forward(1.75, CONFIG))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (-2.2, -0.2, 0.0).into(),
            1.0,
            1.0,
            CONFIG,
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.3, CONFIG))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (-1.25, -0.6, 2.36).into(),
            1.0,
            1.0,
            CONFIG,
        ))
        .await;
    robot.clamp.extend();
}

pub async fn blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    red(robot).await;
}
