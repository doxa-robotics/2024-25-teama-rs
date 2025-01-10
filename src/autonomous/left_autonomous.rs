use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

struct LeftAuton;

#[async_trait]
impl AutonRoutine<Robot> for LeftAuton {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        // 0 degrees
        robot.drivetrain.reset_inertial(0.0).await?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        // Move forward cm
        robot.drivetrain.drive_for(500.0).await?;

        sleep(Duration::from_millis(1000)).await;
        robot.intake.stop().await;
        //turn
        robot.drivetrain.turn_to(-90.0).await?;
        //unclamp
        robot.clamp.unclamp()?;
        // Move back cm
        robot.drivetrain.drive_for(-250.0).await?;
        //unclamp
        robot.clamp.clamp()?;
        //score
        robot.intake.run(Direction::Forward).await;
        sleep(Duration::from_millis(1000)).await;
        robot.intake.stop().await;

        //clamp
        robot.clamp.clamp()?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        sleep(Duration::from_millis(1000)).await;
        robot.intake.stop().await;
        //back 137cm
        robot.drivetrain.drive_for(1370.0).await?;
        //turn
        robot.drivetrain.turn_to(90.0).await?;
        //unclamp
        robot.clamp.unclamp()?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        //back 50
        robot.drivetrain.drive_for(500.0).await?;
        // Turn the intake off

        Ok(())
    }

    fn name(&self) -> &'static str {
        ""
    }

    fn description(&self) -> &'static str {
        "Rebecca"
    }
}

super::red_auton!(RedLeftAuton, LeftAuton, "Right Auton");
super::blue_auton!(BlueLeftAuton, LeftAuton, "Left Auton");
