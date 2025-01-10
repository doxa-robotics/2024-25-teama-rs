use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

pub struct FiveRingRed;

#[async_trait]
impl AutonRoutine<Robot> for FiveRingRed {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        robot.drivetrain.set_negate_turns(false);

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Five ring red"
    }

    fn description(&self) -> &'static str {
        "Preload needed; start right of middle"
    }
}
