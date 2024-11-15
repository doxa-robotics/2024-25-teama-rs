use alloc::boxed::Box;

use crate::{autonomous::AutonomousRoutine, RobotDevices};

pub struct Noop;

impl AutonomousRoutine for Noop {
    fn run(
        &self,
        _devices: &mut RobotDevices,
    ) -> alloc::boxed::Box<(dyn core::future::Future<Output = ()> + Unpin)> {
        // This routine does nothing
        Box::new(Box::pin(async {}))
    }

    fn name(&self) -> &str {
        "Noop"
    }

    fn description(&self) -> &str {
        "An autonomous routine that does nothing, generated using AI. Fancy!"
    }
}
