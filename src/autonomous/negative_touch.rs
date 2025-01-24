use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

struct NegativeTouch;

#[async_trait]
impl AutonRoutine<Robot> for NegativeTouch {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        robot.drivetrain.reset_inertial(180.0).await?;
        robot.clamp.unclamp()?;
        robot.drivetrain.drive_for(-560.0).await?;
        robot.drivetrain.drive_for(-40.0).await?;
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        robot.drivetrain.turn_to(-30.0).await?;
        robot.intake.run(Direction::Forward).await;
        robot.drivetrain.drive_for(700.0).await?;
        robot.drivetrain.turn_to(10.0).await?;
        robot.drivetrain.drive_for(600.0).await?;
        sleep(Duration::from_millis(500)).await;
        robot.drivetrain.drive_for(-300.0).await?;
        robot.intake.stop().await;
        robot.drivetrain.turn_to(90.0).await?;
        robot.drivetrain.drive_for(900.0).await?;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Stake side"
    }

    fn description(&self) -> &'static str {
        "Clamp forward; barely miss rings"
    }
}

super::red_auton!(RedPositive, NegativeTouch, "Positive (+)");
super::blue_auton!(BluePositive, NegativeTouch, "Positive (+)");
