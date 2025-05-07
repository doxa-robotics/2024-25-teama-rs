use crate::{
    subsystems::drivetrain_actions::{self, CONFIG},
    Robot,
};

pub async fn route(robot: &mut Robot) {
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.2, CONFIG))
        .await;
}
