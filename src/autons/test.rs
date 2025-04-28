use core::time::Duration;

use vexide::time::sleep;

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG},
    Robot,
};

async fn route(robot: &mut Robot) {
    loop {
        log::debug!("{:?}", robot.tracking.borrow().pose());
        sleep(Duration::from_millis(100)).await;
    }
    return;
    robot
        .drivetrain
        .action(drivetrain_actions::turn_to_point((1.0, 3.0).into(), CONFIG))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.1, CONFIG))
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
