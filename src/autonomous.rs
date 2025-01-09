use alloc::{boxed::Box, collections::btree_map::BTreeMap};
use core::{
    error::Error,
    fmt::{self, Display, Formatter},
};

use doxa_selector::AutonRoutine;

use crate::Robot;

pub mod blue_autonomous;
pub mod five_ring_blue;
pub mod five_ring_red;
pub mod leftred_autonomous;
pub mod noop;
pub mod red_autonomous;
pub mod rightred_autonomous;
pub mod skills;
pub mod test;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Category {
    Skills,
    Red,
    Blue,
    Test,
}

impl Display for Category {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Category::Red => write!(f, "Red"),
            Category::Blue => write!(f, "Blue"),
            Category::Skills => write!(f, "Skills"),
            Category::Test => write!(f, "Test/none"),
        }
    }
}

pub type Return = Result<(), Box<dyn Error>>;

pub fn autonomous_routes<'a>(
) -> BTreeMap<Category, &'static [&'a dyn doxa_selector::AutonRoutine<Robot, Return = Return>]> {
    let mut map: BTreeMap<Category, &[&dyn AutonRoutine<Robot, Return = Return>]> = BTreeMap::new();
    map.insert(Category::Skills, &[&skills::Skills]);
    map.insert(
        Category::Red,
        &[
            &five_ring_red::FiveRingRed,
            &leftred_autonomous::LeftRedAuton,
            &red_autonomous::RedAuton,
            &rightred_autonomous::RightRedAuton,
        ],
    );
    map.insert(
        Category::Blue,
        &[
            &five_ring_blue::FiveRingBlue,
            &blue_autonomous::BlueAutonomous,
        ],
    );
    map.insert(Category::Test, &[&test::Test, &noop::Noop]);
    map
}
