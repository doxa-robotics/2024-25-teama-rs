use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

struct Positive;

#[async_trait]
impl AutonRoutine<Robot> for Positive {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        //--------------------grab the stake---------------------------//
        // 0 degrees
        robot.drivetrain.reset_inertial(180.0).await?;
        //unclamp
        robot.clamp.unclamp()?;
        // Move  64cm
        robot.drivetrain.drive_for(-840.0).await?;
        //turn
        robot.drivetrain.turn_to(150.0).await?;
        // Move  20cm
        robot.drivetrain.drive_for(-300.0).await?;
        robot.drivetrain.drive_for(-85.0).await?;
        //clamp
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        robot.intake.run(Direction::Forward).await;
        robot.drivetrain.drive_for(150.0).await?;
        robot.intake.stop().await;
        //turn
        robot.drivetrain.turn_to(220.0).await?;
        //unclamp
        robot.clamp.unclamp()?;
        robot.intake.run(Direction::Forward).await;
        let intake = robot.intake.clone();
        spawn(async move {
            sleep(Duration::from_millis(1000)).await;
            intake.stop().await;
        })
        .detach();
        // Move  5cm
        robot.drivetrain.drive_for(280.0).await?;
        sleep(Duration::from_millis(500)).await;
        //turn
        robot.drivetrain.turn_to(90.0).await?;

        //--------------------- grab stake 2 and score 2----------------//

        //move
        robot.drivetrain.drive_for(-270.0).await?;
        robot.drivetrain.drive_for(-100.0).await?;
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        robot.drivetrain.turn_to(-65.0).await?;
        robot.intake.run(Direction::Forward).await;
        robot.drivetrain.drive_for(510.0).await?;
        sleep(Duration::from_millis(500)).await;
        robot.intake.stop().await;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Stake side"
    }

    fn description(&self) -> &'static str {
        "Clamp forward; barely miss rings"
    }
}

super::red_auton!(RedPositive, Positive, "Positive (+)");
super::blue_auton!(BluePositive, Positive, "Positive (+)");
