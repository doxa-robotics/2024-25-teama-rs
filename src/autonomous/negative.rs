use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

struct Negative;

#[async_trait]
impl AutonRoutine<Robot> for Negative {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        // 0 degrees
        robot.drivetrain.reset_inertial(270.0).await?;
        robot.clamp.clamp()?;
        // Reverse 30cm
        robot
            .drivetrain
            .drive_for_advanced(-380.0, 1.0, 0.7)
            .await?;
        // Turn to 0 degrees
        robot.drivetrain.turn_to(0.0).await?;
        // Reverse 10cm
        robot.drivetrain.drive_for(-165.0).await?;
        robot.intake.run(Direction::Forward).await;
        sleep(Duration::from_millis(800)).await;
        robot.intake.stop().await;
        robot.drivetrain.drive_for(400.0).await?;
        robot.clamp.unclamp()?;
        robot.drivetrain.turn_to(180.0 - 48.0).await?;
        robot.drivetrain.drive_for(-460.0).await?;
        robot.drivetrain.drive_for(-120.0).await?;
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        robot.drivetrain.turn_to(-48.0).await?;
        robot.intake.run(Direction::Forward).await;
        robot.drivetrain.drive_for(710.0).await?;
        sleep(Duration::from_millis(500)).await;
        robot.drivetrain.drive_for(-480.0).await?;
        robot.drivetrain.turn_to(-90.0).await?;
        robot.drivetrain.drive_for(400.0).await?;
        sleep(Duration::from_millis(1000)).await;
        robot.intake.stop().await;
        Ok(())
    }

    fn name(&self) -> &'static str {
        ""
    }

    fn description(&self) -> &'static str {
        "Parallel to line, approaching center"
    }
}

super::red_auton!(RedNegative, Negative, "Negative (-)");
super::blue_auton!(BlueNegative, Negative, "Negative (-)");
