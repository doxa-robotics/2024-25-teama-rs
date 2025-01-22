use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

struct NewAuton;

#[async_trait]
impl AutonRoutine<Robot> for NewAuton {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        // 0 degrees
        robot.drivetrain.reset_inertial(90.0).await?;
        robot.clamp.clamp()?;
        // Reverse 30cm
        robot.drivetrain.drive_for_advanced(-380.0, 0.5).await?;
        // Turn to 0 degrees
        robot.drivetrain.turn_to(0.0).await?;
        // Reverse 10cm
        robot.drivetrain.drive_for(-160.0).await?;
        robot.intake.run(Direction::Forward).await;
        sleep(Duration::from_millis(1200)).await;
        robot.intake.stop().await;
        robot.drivetrain.drive_for(400.0).await?;
        robot.drivetrain.turn_to(360.0 - 48.0).await?;
        robot.clamp.unclamp()?;
        robot.drivetrain.drive_for_advanced(-480.0, 0.6).await?;
        robot.drivetrain.drive_for(-150.0).await?;
        robot.clamp.clamp()?;
        sleep(Duration::from_millis(500)).await;
        robot.drivetrain.turn_for(180.0).await?;
        robot.intake.run(Direction::Forward).await;
        robot.drivetrain.drive_for(680.0).await?;
        sleep(Duration::from_millis(1500)).await;
        robot.drivetrain.drive_for(-200.0).await?;
        robot.clamp.unclamp()?;

        Ok(())
    }

    fn name(&self) -> &'static str {
        ""
    }

    fn description(&self) -> &'static str {
        "It's 5:30am and I'm tired"
    }
}

super::red_auton!(RedNewAuton, NewAuton, "New Auton");
super::blue_auton!(BlueNewAuton, NewAuton, "New Auton");
