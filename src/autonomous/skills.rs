use alloc::boxed::Box;
use core::time::Duration;

use vexide::prelude::sleep;

use crate::{autonomous::AutonomousRoutine, RobotDevices};

pub struct Skills;

impl AutonomousRoutine for Skills {
    fn run<'a>(
        &'a self,
        devices: &'a mut RobotDevices,
    ) -> Box<dyn core::future::Future<Output = ()> + Unpin + 'a> {
        Box::new(Box::pin(async move {
            // Place the ring on the stake
            devices.intake.set_velocity(450).ok();
            sleep(Duration::from_secs(2)).await;
            devices.intake.brake(vexide::prelude::BrakeMode::Brake).ok();
        }))
    }

    fn name(&self) -> &str {
        "Auton 1"
    }

    fn description(&self) -> &str {
        "An autonomous routine that drives forward, intakes a ring, then turns and puts it on a stake."
    }
}
