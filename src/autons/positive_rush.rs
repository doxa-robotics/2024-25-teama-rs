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
}

pub async fn blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    red(robot).await;
}
