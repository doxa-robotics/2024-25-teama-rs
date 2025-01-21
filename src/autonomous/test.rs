use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use log::debug;
use vexide::{core::println, prelude::sleep};

use crate::Robot;

pub struct Test;

#[async_trait]
impl AutonRoutine<Robot> for Test {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        robot.drivetrain.reset_inertial(0.0).await?;
        robot.drivetrain.turn_to(45.0).await?;
        robot.drivetrain.turn_to(-45.0).await?;
        debug!("done");

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Test"
    }

    fn description(&self) -> &'static str {
        "A test autonomous routine"
    }
}
