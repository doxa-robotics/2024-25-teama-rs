use core::time::Duration;

pub mod intake;
pub mod lady_brown;

pub type Clamp = libdoxa::subsystems::pneumatic::PneumaticSubsystem<1, false>;
pub type Doinker = libdoxa::subsystems::pneumatic::MirroredPneumaticSubsystem<1, false>;

const SUBSYSTEM_UPDATE_PERIOD: Duration = Duration::from_millis(10);
