use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

pub struct BlueAutonomous;

#[async_trait]
impl AutonRoutine<Robot> for BlueAutonomous {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        // 0 degrees
        robot.drivetrain.reset_inertial(0.0).await?;
        //turn
        //unclamp
        robot.clamp.unclamp()?;
        // Move forward 97cm
        robot.drivetrain.drive_for(970.0).await?;
        //clamp
        robot.clamp.clamp()?;
        //back 137cm
        robot.drivetrain.drive_for(-1370.0).await?;
        //turn

        // TODO: unfinished

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Blue autonomous"
    }

    fn description(&self) -> &'static str {
        "Rebecca"
    }
}
