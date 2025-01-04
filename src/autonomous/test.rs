use alloc::boxed::Box;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;

use crate::Robot;

pub struct Test;

#[async_trait]
impl AutonRoutine<Robot> for Test {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        robot.intake.partial_intake();
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Test"
    }

    fn description(&self) -> &'static str {
        "A test autonomous routine"
    }
}
