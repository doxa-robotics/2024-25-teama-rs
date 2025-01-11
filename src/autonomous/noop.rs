use alloc::boxed::Box;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;

use crate::Robot;

pub struct Noop;

#[async_trait]
impl AutonRoutine<Robot> for Noop {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        crate::opcontrol::opcontrol(robot).await?;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "None"
    }

    fn description(&self) -> &'static str {
        "An autonomous routine that does nothing, generated using AI!"
    }
}
