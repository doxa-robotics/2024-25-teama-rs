use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::{core::println, prelude::sleep};

use crate::Robot;

pub struct Test;

#[async_trait]
impl AutonRoutine<Robot> for Test {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        robot.drivetrain.reset_inertial(0.0).await?;
        robot.drivetrain.turn_for(10.0).await?;

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Test"
    }

    fn description(&self) -> &'static str {
        "A test autonomous routine"
    }
}
