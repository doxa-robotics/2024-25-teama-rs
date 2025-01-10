use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

pub struct RightAuton;

#[async_trait]
impl AutonRoutine<Robot> for RightAuton {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        // 0 degrees
        robot.drivetrain.reset_inertial(0.0).await?;
        //turn
        robot.drivetrain.turn_to(25.0).await?;
        //unclamp
        robot.clamp.unclamp()?;
        // Move forward 97cm
        robot.drivetrain.drive_for(-970.0).await?;
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
        robot.intake.stop().await;
        //turn 180
        robot.drivetrain.turn_to(180.0).await?;
        //move forward 10
        robot.drivetrain.drive_for(100.0).await?;
        //move back 15
        robot.drivetrain.drive_for(-150.0).await?;
        //turn
        robot.drivetrain.turn_to(25.0).await?;
        //unclamp
        robot.clamp.unclamp()?;
        //move forward 70
        robot.drivetrain.drive_for(700.0).await?;
        //clamp
        robot.clamp.clamp()?;
        //turn
        robot.drivetrain.turn_to(-48.0).await?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        //move forward 60
        robot.drivetrain.drive_for(600.0).await?;
        //turn
        robot.drivetrain.turn_to(-150.0).await?;
        //move forward 40
        robot.drivetrain.drive_for(400.0).await?;
        // Turn the intake off
        robot.intake.stop().await;
        //turn
        robot.drivetrain.turn_to(-48.0).await?;
        //move Back to ladder
        robot.drivetrain.drive_for(900.0).await?;

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Right Red Auton"
    }

    fn description(&self) -> &'static str {
        "Rebecca"
    }
}

super::red_auton!(RedRightAuton, RightAuton, "Left Auton");
super::blue_auton!(BlueRightAuton, RightAuton, "Right Auton");
