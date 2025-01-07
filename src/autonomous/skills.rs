use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

pub struct Skills;

#[async_trait]
impl AutonRoutine<Robot> for Skills {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        // 0 degrees
        robot.drivetrain.reset_inertial(0.0).await?;
        // Place the ring on the stake
        robot.intake.run(Direction::Forward).await;
        sleep(Duration::from_secs(1)).await;
        robot.intake.stop().await;
        // Drive forward 40cm
        robot.drivetrain.drive_for(400.0).await?;
        // Turn right 90 degrees
        robot.drivetrain.turn_for(90.0).await?;
        // Unclamp the goal
        robot.clamp.unclamp()?;
        // Drive backward 40cm
        robot.drivetrain.drive_for(-400.0).await?;
        // Clamp the goal and wait 500ms for the clamp to close
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        // Turn to 60 degrees
        robot.drivetrain.turn_to(60.0).await?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        // Drive forward 100cm
        robot.drivetrain.drive_for(1000.0).await?;
        // Drive backward 30cm
        robot.drivetrain.drive_for(-300.0).await?;
        // Turn the intake off
        robot.intake.stop().await;
        // Turn to 180 degrees
        robot.drivetrain.turn_to(180.0).await?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        // Drive forward 130cm
        robot.drivetrain.drive_for(1300.0).await?;
        // Turn to 45 degrees
        robot.drivetrain.turn_to(45.0).await?;
        // Drive forward 30cm
        robot.drivetrain.drive_for(300.0).await?;
        // Turn the intake off
        robot.intake.stop().await;
        // Turn to 110 degrees
        robot.drivetrain.turn_to(110.0).await?;
        // Drive backward 60cm
        robot.drivetrain.drive_for(-600.0).await?;
        // Clamp the goal and wait 500ms for the clamp to close
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Skills"
    }

    fn description(&self) -> &'static str {
        "Final skills"
    }
}
