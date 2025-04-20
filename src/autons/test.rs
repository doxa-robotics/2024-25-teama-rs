use crate::{subsystems::drivetrain_actions, Robot};

pub async fn test_auton_red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_pose((0.0, 0.0, 0.0).into());
    robot
        .drivetrain
        .action(drivetrain_actions::forward(100.0))
        .await;
}

pub async fn test_auton_blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    test_auton_red(robot).await;
}
