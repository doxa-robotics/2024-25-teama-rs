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
        // 0 degrees
        robot.drivetrain.reset_inertial(0.0).await?;
        // Place the ring on the stake
        robot.intake.run(Direction::Forward).await;
        sleep(Duration::from_secs(1)).await;
        robot.intake.stop().await;
        // Drive forward 40cm
        robot.drivetrain.drive_for(330.0).await?;
        // Turn right 90 degrees
        robot.drivetrain.turn_to(90.0).await?;
        // Clamp the goal
        robot.clamp.unclamp()?;
        // Drive backward 40cm
        robot.drivetrain.drive_for(-340.0).await?;
        robot.drivetrain.drive_for(-40.0).await?;
        // Clamp the goal and wait 500ms for the clamp to close
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        // Turn to 60 degrees
        robot.drivetrain.turn_to(60.0).await?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        // Drive forward 100cm
        robot.drivetrain.drive_for(1100.0).await?;
        // Turn to 55 degrees
        robot.drivetrain.turn_to(53.0).await?;
        sleep(Duration::from_millis(500)).await;
        // Drive 1 meter
        robot.drivetrain.drive_for(1110.0).await?;
        // Drive backward 30cm
        robot.drivetrain.drive_for(-220.0).await?;
        // Turn the intake off
        robot.intake.stop().await;
        // Turn to 180 degrees
        robot.drivetrain.turn_to(180.0).await?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        // Drive forward 130cm
        robot.drivetrain.drive_for(400.0).await?;
        robot.drivetrain.drive_for(700.0).await?;
        robot.drivetrain.drive_for(400.0).await?;
        // Turn to 45 degrees
        robot.drivetrain.turn_to(45.0).await?;
        // Drive forward 30cm
        robot.drivetrain.drive_for_advanced(300.0, 0.8).await?;
        // Turn the intake off
        robot.intake.stop().await;
        // Turn to -35 degrees and back up 10cm
        robot.drivetrain.turn_to(-28.0).await?;
        robot.drivetrain.drive_for(-200.0).await?;
        // Unclamp the goal and wait 500ms for the clamp to open
        robot.clamp.unclamp()?;
        sleep(Duration::from_millis(500)).await;
        robot.drivetrain.drive_for(150.0).await?;
        // Turn to 110 degrees
        robot.drivetrain.turn_to(90.0).await?;
        // Drive backward 60cm
        robot.drivetrain.drive_for(-600.0).await?;
        // Clamp the goal and wait 500ms for the clamp to close
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        // Turn to -35 degrees
        robot.drivetrain.turn_to(-35.0).await?;
        // Drive foward 150 cm while intaking
        robot.intake.run(Direction::Forward).await;
        robot.drivetrain.drive_for(1500.0).await?;
        // Wait a bit to finish intaking
        sleep(Duration::from_millis(500)).await;
        // Turn the intake off
        robot.intake.stop().await;
        // Turn to -135 degrees
        robot.drivetrain.turn_to(-135.0).await?;
        // Start the intake again and drive forward 70cm
        robot.intake.run(Direction::Forward).await;
        robot.drivetrain.drive_for(770.0).await?;
        // Turn to -90 degrees
        robot.drivetrain.turn_to(-90.0).await?;
        // Drive 70cm
        robot.drivetrain.drive_for(730.0).await?;
        // Turn to 180 degrees and drive downwards 90cm
        robot.drivetrain.turn_to(180.0).await?;
        robot.drivetrain.drive_for(600.0).await?;
        robot.drivetrain.drive_for(400.0).await?;
        robot.drivetrain.turn_to(-45.0).await?;
        robot.drivetrain.drive_for(300.0).await?;
        robot.drivetrain.turn_to(20.0).await?;
        // Turn off the intake
        robot.intake.stop().await;
        // Turn to 20 degrees and release the goal
        robot.drivetrain.drive_for(-200.0).await?;
        robot.clamp.unclamp()?;
        sleep(Duration::from_millis(500)).await;
        // Drive out a bit and turn to 20 degrees
        robot.drivetrain.drive_for(100.0).await?;
        return Ok(());
        robot.drivetrain.turn_to(20.0).await?;
        // Drive forward 190cm
        robot.drivetrain.drive_for(1900.0).await?;
        // Turn to -90 degrees
        robot.drivetrain.turn_to(-90.0).await?;
        // Turn on the intake and drive forward 70cm
        robot.intake.run(Direction::Forward).await;
        robot.arm.set_state(ArmState::Intake).await;
        robot.drivetrain.drive_for(600.0).await?;
        // Use lady brown - we need to keep the intake on
        robot.arm.set_state(ArmState::MaxExpansion).await;
        sleep(Duration::from_millis(300)).await;
        robot.intake.stop().await;
        // Should have gone on
        sleep(Duration::from_millis(500)).await;
        robot.arm.set_state(ArmState::Initial).await;
        return Ok(());
        // Turn to 25 degrees
        robot.drivetrain.turn_to(25.0).await?;
        // Drive forward 75cm while intaking
        robot.intake.run(Direction::Forward).await;
        robot.drivetrain.drive_for(750.0).await?;
        // Turn the intake off
        robot.intake.stop().await;
        // Turn to 180+35 degrees
        robot.drivetrain.turn_to(215.0).await?;
        // Drive backward 70cm
        robot.drivetrain.drive_for(-700.0).await?;
        // Clamp the goal and wait 500ms for the clamp to close
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        // Turn to 145 degrees
        robot.drivetrain.turn_to(145.0).await?;
        // Drive backward 70cm and unclamp the goal
        robot.drivetrain.drive_for(-700.0).await?;
        robot.clamp.unclamp()?;
        // Turn to -80 degrees
        robot.drivetrain.turn_to(-80.0).await?;
        // Drive backward 130cm
        robot.drivetrain.drive_for(-1300.0).await?;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Skills"
    }

    fn description(&self) -> &'static str {
        "Final skills"
    }
}
