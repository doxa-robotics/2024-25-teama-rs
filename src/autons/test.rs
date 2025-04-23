use crate::{
    subsystems::drivetrain_actions::{self, CONFIG},
    Robot,
};

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_pose((0.0, 0.0, 0.0).into());
    robot
        .drivetrain
        .action(drivetrain_actions::smooth_to_point(
            (1.0, 1.0, -0.3).into(),
            1.0, // start_easing
            1.0, // end_easing
            CONFIG,
            false,
        ))
        .await;
}

pub async fn blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    red(robot).await;
}
