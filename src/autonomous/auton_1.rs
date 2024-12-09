use alloc::boxed::Box;
use core::time::Duration;

use vexide::prelude::sleep;

use crate::{autonomous::AutonomousRoutine, RobotDevices};

pub struct Auton1;

impl AutonomousRoutine for Auton1 {
    fn run<'a>(
        &'a self,
        devices: &'a mut RobotDevices,
    ) -> Box<dyn core::future::Future<Output = ()> + Unpin + 'a> {
        Box::new(Box::pin(async move {
            devices.drivetrain.drive_for(-290.0).await.ok();
            devices.drivetrain.turn_for(90.0).await.ok();
            devices.drivetrain.drive_for(-60.0).await.ok();

            // Place the ring on the stake
            devices.intake.run(vexide::prelude::Direction::Forward).ok();
            sleep(Duration::from_secs(2)).await;
            devices.intake.stop().ok();
        }))
    }

    fn name(&self) -> &str {
        "Auton 1"
    }

    fn description(&self) -> &str {
        "An autonomous routine that drives forward, intakes a ring, then turns and puts it on a stake."
    }
}
