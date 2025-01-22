use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::{subsystems::arm::ArmState, Robot};

pub struct Skills;

#[async_trait]
impl AutonRoutine<Robot> for Skills {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        //------------------Start to passed middle--------------------//
        // 0 degrees
        robot.drivetrain.reset_inertial(0.0).await?;
        // Place the ring on the stake
        robot.intake.run(Direction::Forward).await;
        sleep(Duration::from_secs(1)).await;
        robot.intake.stop().await;
        // Drive forward 40cm
        robot.drivetrain.drive_for(420.0).await?;
        // Turn right 90 degrees
        robot.drivetrain.turn_to(-90.0).await?;
        // Clamp the goal
        robot.clamp.unclamp()?;
        // Drive backward 65cm
        robot.drivetrain.drive_for(-650.0).await?;
        // Clamp the goal and wait 500ms for the clamp to close
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        // Turn to 20 degrees
        robot.drivetrain.turn_to(20.0).await?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        // Drive forward 43cm
        robot.drivetrain.drive_for(430.0).await?;
        // Turn to 30 degrees
        robot.drivetrain.turn_to(30.0).await?;
        // Drive 80cm
        robot.drivetrain.drive_for(800.0).await?;
        sleep(Duration::from_millis(500)).await;
        // Drive backward 72cm
        robot.drivetrain.drive_for(72.0).await?;
        // Turn to 30 degrees
        robot.drivetrain.turn_to(30.0).await?;
        robot.intake.stop().await;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Skills"
    }

    fn description(&self) -> &'static str {
        "Final skills"
    }
}
