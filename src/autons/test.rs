use crate::{
    subsystems::drivetrain_actions::{self, CONFIG},
    Robot,
};

pub async fn test_auton_red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_pose((0.0, 0.0, 0.0).into());
    robot
        .drivetrain
        .action(drivetrain_actions::forward(1.0, CONFIG))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(1.0, CONFIG))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::turn_to_point((0.0, 0.0).into(), CONFIG))
        .await;
}

pub async fn test_auton_blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    test_auton_red(robot).await;
}
