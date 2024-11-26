use alloc::boxed::Box;
use core::future::Future;

use crate::RobotDevices;

pub mod auton_1;
pub mod auton_2;
pub mod noop;
pub mod skills;
pub mod test;

/// Enum of all autonomous routines
///
/// Please remember to add new autonomous routines here when declaring the
/// module.
pub static AUTONOMOUS_ROUTINES: &[&dyn AutonomousRoutine] =
    &[&noop::Noop, &auton_1::Auton1, &test::Test];

pub trait AutonomousRoutine: Sync {
    fn run<'a>(
        &'a self,
        devices: &'a mut RobotDevices,
    ) -> Box<dyn Future<Output = ()> + Unpin + 'a>;
    fn name(&self) -> &str;
    fn description(&self) -> &str;
}
