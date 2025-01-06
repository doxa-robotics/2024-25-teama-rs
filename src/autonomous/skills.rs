use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use doxa_selector::AutonRoutine;
use vexide::prelude::sleep;

use crate::Robot;

pub struct Skills;

#[async_trait]
impl AutonRoutine<Robot> for Skills {
    type Return = super::Return;

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        // Place the ring on the stake
        robot.intake.run(vexide::prelude::Direction::Forward);
        sleep(Duration::from_secs(2)).await;
        robot.intake.stop();
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Auton 1"
    }

    fn description(&self) -> &'static str {
        "An autonomous routine that drives forward, intakes a ring, then turns and puts it on a stake."
    }
}
