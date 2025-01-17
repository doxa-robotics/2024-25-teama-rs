use core::time::Duration;

pub mod arm;
pub mod clamp;
pub mod doinker;
pub mod drivetrain;
pub mod intake;

const SUBSYSTEM_UPDATE_PERIOD: Duration = Duration::from_millis(10);
