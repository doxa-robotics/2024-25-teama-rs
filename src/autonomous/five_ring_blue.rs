use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::*;

use crate::Robot;

pub struct FiveRingBlue;

#[async_trait]
impl AutonRoutine<Robot> for FiveRingBlue {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        // TODO: write auton
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Five ring blue"
    }

    fn description(&self) -> &'static str {
        "Preload needed; start left of middle"
    }
}
