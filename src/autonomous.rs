use alloc::{boxed::Box, collections::btree_map::BTreeMap};
use core::{
    error::Error,
    fmt::{self, Display, Formatter},
};

use doxa_selector::AutonRoutine;

use crate::Robot;

pub mod five_ring;
pub mod left_autonomous;
pub mod new_auton;
pub mod noop;
pub mod right_autonomous;
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
            &left_autonomous::RedLeftAuton,
            &right_autonomous::RedRightAuton,
            &new_auton::RedNewAuton,
        ],
    );
    map.insert(
        Category::Blue,
        &[
            &left_autonomous::BlueLeftAuton,
            &right_autonomous::BlueRightAuton,
            &new_auton::BlueNewAuton,
        ],
    );
    map.insert(Category::Test, &[&test::Test, &noop::Noop]);
    map
}

macro_rules! auton {
    ($generated_name: ident, $target_name: ident, $negate_turns: expr, $name: expr) => {
        pub struct $generated_name;

        #[async_trait]
        impl AutonRoutine<Robot> for $generated_name {
            type Return = super::Return;

            async fn run(&self, robot: &mut Robot) -> Self::Return {
                robot.drivetrain.set_negate_turns($negate_turns);
                $target_name.run(robot).await
            }

            fn name(&self) -> &'static str {
                $name
            }

            fn description(&self) -> &'static str {
                $target_name.description()
            }
        }
    };
    ($generated_name: ident, $target_name: ident, $negate_turns: expr) => {
        pub struct $generated_name;

        #[async_trait]
        impl AutonRoutine<Robot> for $generated_name {
            type Return = super::Return;

            async fn run(&self, robot: &mut Robot) -> Self::Return {
                robot.drivetrain.set_negate_turns($negate_turns);
                $target_name.run(robot).await
            }

            fn name(&self) -> &'static str {
                $target_name.name()
            }

            fn description(&self) -> &'static str {
                $target_name.description()
            }
        }
    };
}

macro_rules! blue_auton {
    ($generated_name: ident, $target_name: ident, $name: expr) => {
        super::auton!($generated_name, $target_name, true, $name);
    };
}
macro_rules! red_auton {
    ($generated_name: ident, $target_name: ident, $name: expr) => {
        super::auton!($generated_name, $target_name, false, $name);
    };
}

use auton;
use blue_auton;
use red_auton;
