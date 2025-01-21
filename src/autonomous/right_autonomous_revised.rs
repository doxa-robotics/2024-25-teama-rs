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
//--------------------grab the stake---------------------------//
        // 0 degrees
        robot.drivetrain.reset_inertial(0.0).await?;
        //turn
        robot.drivetrain.turn_to(180.0).await?;
        //unclamp
        robot.clamp.unclamp()?;
        // Move  64cm
        robot.drivetrain.drive_for(-640.0).await?;
        //turn
        robot.drivetrain.turn_to(140.0).await?;
        // Move  20cm
        robot.drivetrain.drive_for(-200.0).await?;
        //clamp
        robot.clamp.clamp()?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        sleep(Duration::from_millis(1000)).await;
        robot.intake.stop().await;
        //turn
        robot.drivetrain.turn_to(290.0).await?;
        //unclamp
        robot.clamp.unclamp()?;
        // Move  5cm
        robot.drivetrain.drive_for(50.0).await?;
        //turn
        robot.drivetrain.turn_to(30.0).await?;
    
//--------------------- grab stake 2 and score 2----------------//

        //move
        robot.drivetrain.drive_for(-250.0).await?;
        //clamp
        robot.clamp.clamp()?;
        //turn
        robot.drivetrain.turn_to(90.0).await?;
        // Turn the intake on
        robot.intake.run(Direction::Forward).await;
        //move 50
        robot.drivetrain.drive_for(250.0).await?;
        //turn 180
        robot.drivetrain.turn_to(180.0).await?;
        //move forward 50
        robot.drivetrain.drive_for(500.0).await?;

//------------------------get last ring and get to the ladder-----------------//
        //turn
        robot.drivetrain.turn_to(290.0).await?;
        //move forward 75
        robot.drivetrain.drive_for(750.0).await?;
        //turn
        robot.drivetrain.turn_to(0.0).await?;
        // Turn the intake off
        robot.intake.stop().await;
        //move forward 60
        robot.drivetrain.drive_for(450.0).await?;

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
