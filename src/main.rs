#![no_main]
#![no_std]
#![feature(never_type)]
#![feature(let_chains)]

extern crate alloc;

mod opcontrol;
mod subsystems;
mod utils;

use alloc::{
    rc::Rc,
    string::{String, ToString},
    vec,
    vec::Vec,
};
use core::{cell::RefCell, time::Duration};

use doxa_selector::{CompeteWithSelector, CompeteWithSelectorExt};
use libdoxa::{subsystems::tracking::wheel::TrackingWheel, utils::vec2::Vec2};
use log::{error, info};
use subsystems::{intake::Intake, lady_brown::LadyBrown, Clamp};
use utils::logger;
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};
use vexide_motorgroup::MotorGroup;

struct Robot {
    controller: Controller,

    inertial: Rc<RefCell<InertialSensor>>,
    drivetrain: libdoxa::subsystems::drivetrain::Drivetrain,

    intake: Intake,
    clamp: Clamp,
    lady_brown: LadyBrown,
}

impl CompeteWithSelector for Robot {
    type Category = String;
    type Return = ();

    fn autonomous_routes<'a, 'b>(
        &'b self,
    ) -> alloc::collections::btree_map::BTreeMap<
        Self::Category,
        impl AsRef<[&'a dyn doxa_selector::AutonRoutine<Self, Return = Self::Return>]>,
    >
    where
        Self: 'a,
    {
        alloc::collections::btree_map::BTreeMap::<
            String,
            Vec<&'a dyn doxa_selector::AutonRoutine<Self, Return = Self::Return>>,
        >::new()
    }

    async fn driver(&mut self) {
        info!("Driver starting");

        loop {
            let Err(err) = opcontrol::opcontrol(self).await;
            error!("opcontrol crashed, restarting in 500ms! {}", err);
            sleep(Duration::from_millis(500)).await;
        }
    }

    fn calibrate_gyro(&mut self) {
        _ = self.inertial.borrow_mut().calibrate();
    }

    fn is_gyro_calibrating(&self) -> bool {
        self.inertial.borrow().is_calibrating().unwrap_or(false)
    }

    fn diagnostics(&self) -> vec::Vec<(alloc::string::String, alloc::string::String)> {
        vec![
            (
                "Gyro".to_string(),
                if self.inertial.borrow().is_calibrating().unwrap_or(false) {
                    "Calibrating".to_string()
                } else {
                    self.inertial.borrow().heading().unwrap_or(-1.0).to_string()
                },
            ),
            /*(
                "Drivetrain temp (C)".to_string(),
                self.drivetrain.temperature().to_string(),
            ),*/
            (
                "Arm temp (C)".to_string(),
                self.lady_brown.temperature().to_string(),
            ),
            (
                "Intake temp (C)".to_string(),
                self.intake.temperature().to_string(),
            ),
        ]
    }

    fn controller(&self) -> Option<&vexide::devices::controller::Controller> {
        Some(&self.controller)
    }

    fn autonomous_route_started(
        &mut self,
        _route: &dyn doxa_selector::AutonRoutine<Self, Return = Self::Return>,
    ) {
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    logger::init().expect("failed to initialize logger");

    let left_motors = Rc::new(RefCell::new(MotorGroup::new(vec![
        Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
    ])));
    let right_motors = Rc::new(RefCell::new(MotorGroup::new(vec![
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
    ])));
    let inertial = Rc::new(RefCell::new(InertialSensor::new(peripherals.port_20)));

    let robot = Robot {
        controller: peripherals.primary_controller,
        inertial: inertial.clone(),
        drivetrain: libdoxa::subsystems::drivetrain::Drivetrain::new(
            left_motors.clone(),
            right_motors.clone(),
            inertial.clone(),
            65.0,
            libdoxa::subsystems::tracking::TrackingSubsystem::new([
                TrackingWheel::new(
                    158.0,
                    122.0,
                    libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Perpendicular,
                    RotationSensor::new(peripherals.port_17, Direction::Forward),
                )
            ], [
                TrackingWheel::new(
                    165.0,
                    0.0,
                    libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
                    left_motors,
                ),
                TrackingWheel::new(
                    165.0,
                    0.0,
                    libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
                    right_motors,
                )
            ], inertial.clone(), Vec2::default()),
        ),

        intake: Intake::new(Motor::new(
            peripherals.port_4,
            Gearset::Blue,
            Direction::Reverse,
        )),
        clamp: Clamp::new([AdiDigitalOut::new(peripherals.adi_b)]),
        lady_brown: LadyBrown::new(
            MotorGroup::new(vec![
                Motor::new_exp(peripherals.port_1, Direction::Forward),
                Motor::new_exp(peripherals.port_2, Direction::Reverse),
            ]),
            3.0,
        )
        .expect("failed to initialize arm"),
    };

    info!("-- Status --");
    // info!("Drivetrain temp: {:?}", robot.drivetrain.temperature());
    info!("Arm temp: {:?}", robot.lady_brown.temperature());
    info!("Intake temp: {:?}", robot.intake.temperature());

    info!("starting subsystem background tasks");
    robot.lady_brown.task();
    robot.intake.task();

    info!("entering competing");
    robot.compete_with_selector(peripherals.display, None).await;
}
