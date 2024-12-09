use alloc::boxed::Box;

use crate::{autonomous::AutonomousRoutine, RobotDevices};

pub struct Test;

impl AutonomousRoutine for Test {
    fn run<'a>(
        &'a self,
        devices: &'a mut RobotDevices,
    ) -> alloc::boxed::Box<(dyn core::future::Future<Output = ()> + Unpin + 'a)> {
        Box::new(Box::pin(async {
            devices.intake.partial_intake().ok();
        }))
    }

    fn name(&self) -> &str {
        "Test"
    }

    fn description(&self) -> &str {
        "A test autonomous routine"
    }
}
