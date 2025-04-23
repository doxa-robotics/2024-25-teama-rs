use core::fmt::Display;

pub mod negative_rush;
pub mod positive_awp;
pub mod positive_rush;
pub mod test;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[allow(unused)]
pub enum AutonCategory {
    RedPositive,
    RedNegative,
    BluePositive,
    BlueNegative,
    Skills,
    Test,
}

impl Display for AutonCategory {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AutonCategory::RedPositive => write!(f, "Red +"),
            AutonCategory::RedNegative => write!(f, "Red -"),
            AutonCategory::BluePositive => write!(f, "Blue +"),
            AutonCategory::BlueNegative => write!(f, "Blue -"),
            AutonCategory::Skills => write!(f, "Skills"),
            AutonCategory::Test => write!(f, "Test"),
        }
    }
}
