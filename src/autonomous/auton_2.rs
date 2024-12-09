use alloc::boxed::Box;
use core::time::Duration;

use vexide::prelude::sleep;

use crate::{autonomous::AutonomousRoutine, RobotDevices};

pub struct Auton2;

impl AutonomousRoutine for Auton2 {
    fn run<'a>(
        &'a self,
        devices: &'a mut RobotDevices,
    ) -> Box<dyn core::future::Future<Output = ()> + Unpin + 'a> {
        Box::new(Box::pin(async move {
            devices.drivetrain.drive_for(650.0).await.ok();
            devices.clamp.clamp().ok();
            devices.intake.run(vexide::prelude::Direction::Forward).ok();
            sleep(Duration::from_secs(2)).await;
            devices.intake.stop().ok();

            devices.drivetrain.drive_for(-600.0).await.ok();
            devices.drivetrain.turn_for(45.0).await.ok();
            devices.intake.run(vexide::prelude::Direction::Forward).ok();
            devices.drivetrain.drive_for(600.0).await.ok();
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
