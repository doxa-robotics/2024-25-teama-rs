use core::time::Duration;

pub mod clamp;
pub mod doinker;
pub mod drivetrain;
pub mod intake;
pub mod lady_brown;

const SUBSYSTEM_UPDATE_PERIOD: Duration = Duration::from_millis(10);
