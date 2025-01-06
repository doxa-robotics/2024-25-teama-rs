use alloc::{boxed::Box, collections::btree_map::BTreeMap};
use core::{
    error::Error,
    fmt::{self, Display, Formatter},
};

use doxa_selector::AutonRoutine;

use crate::Robot;

pub mod auton_1;
pub mod auton_2;
pub mod noop;
pub mod skills;
// pub mod test;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Category {
    Skills,
    Autonomous,
    Test,
}

impl Display for Category {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Category::Skills => write!(f, "Skills"),
            Category::Autonomous => write!(f, "Autonomous"),
            Category::Test => write!(f, "Test"),
        }
    }
}

pub type Return = Result<(), Box<dyn Error>>;

pub fn autonomous_routes<'a>(
) -> BTreeMap<Category, &'static [&'a dyn doxa_selector::AutonRoutine<Robot, Return = Return>]> {
    let mut map: BTreeMap<Category, &[&dyn AutonRoutine<Robot, Return = Return>]> = BTreeMap::new();
    map.insert(Category::Skills, &[&skills::Skills]);
    map.insert(Category::Autonomous, &[&auton_1::Auton1, &auton_2::Auton2]);
    // map.insert(Category::Test, &[&test::Test]);
    map
}
