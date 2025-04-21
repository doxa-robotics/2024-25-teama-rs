#![no_main]
#![no_std]
#![feature(never_type)]
#![feature(let_chains)]

extern crate alloc;

mod autons;
mod opcontrol;
mod subsystems;
mod utils;

use alloc::{rc::Rc, sync::Arc, vec};
use core::{cell::RefCell, time::Duration};

use ::autons::prelude::{SelectCompete, SelectCompeteExt};
use autons::AutonCategory;
use autons_controller::{route, ControllerSelect};
use libdoxa::{subsystems::tracking::wheel::TrackingWheel, utils::pose::Pose};
use log::{error, info};
use subsystems::{intake::Intake, lady_brown::LadyBrown, Clamp, Doinker, IntakeRaiser};
use utils::logger;
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO, sync::Mutex};
use vexide_motorgroup::MotorGroup;

struct Robot {
    controller: Arc<Mutex<Controller>>,

    drivetrain: libdoxa::subsystems::drivetrain::Drivetrain,
    tracking: Rc<RefCell<libdoxa::subsystems::tracking::TrackingSubsystem>>,

    intake: Intake,
    intake_raiser: IntakeRaiser,
    doinker: Doinker,
    clamp: Clamp,
    lady_brown: LadyBrown,
}

impl SelectCompete for Robot {
    async fn driver(&mut self) {
        self.tracking.borrow_mut().set_reverse(false);
        self.drivetrain.set_max_voltage(Motor::V5_MAX_VOLTAGE);
        info!("Driver starting");

        loop {
            let Err(err) = opcontrol::opcontrol(self).await;
            error!("opcontrol crashed, restarting in 500ms! {}", err);
            sleep(Duration::from_millis(500)).await;
        }
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
    let tracking = Rc::new(RefCell::new(
        libdoxa::subsystems::tracking::TrackingSubsystem::new(
            [TrackingWheel::new(
                158.0,
                122.0,
                libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Perpendicular,
                RotationSensor::new(peripherals.port_17, Direction::Forward),
            )],
            [
                TrackingWheel::new(
                    165.0 * 2.0,
                    0.0,
                    libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
                    left_motors.clone(),
                ),
                TrackingWheel::new(
                    165.0 * 2.0,
                    0.0,
                    libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
                    right_motors.clone(),
                ),
            ],
            inertial.clone(),
            Pose::default(),
        ),
    ));

    let mut robot = Robot {
        controller: Arc::new(Mutex::new(peripherals.primary_controller)),

        intake_raiser: IntakeRaiser::new([AdiDigitalOut::new(peripherals.adi_c)]),

        doinker: Doinker::new(
            [AdiDigitalOut::new(peripherals.adi_d)],
            [AdiDigitalOut::new(peripherals.adi_e)],
            libdoxa::subsystems::pneumatic::MirroredState::Normal,
        ),

        drivetrain: libdoxa::subsystems::drivetrain::Drivetrain::new(
            left_motors.clone(),
            right_motors.clone(),
            65.0,
            Motor::V5_MAX_VOLTAGE,
            tracking.clone(),
        ),
        tracking: tracking.clone(),

        intake: Intake::new(
            Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
            VisionSensor::new(peripherals.port_11),
            DistanceSensor::new(peripherals.port_12),
        ),

        clamp: Clamp::new([AdiDigitalOut::new(peripherals.adi_b)]),

        lady_brown: LadyBrown::new(
            MotorGroup::new(vec![
                Motor::new_exp(peripherals.port_1, Direction::Forward),
                Motor::new_exp(peripherals.port_2, Direction::Reverse),
            ]),
            3.0,
            AdiDigitalIn::new(peripherals.adi_f),
        )
        .expect("failed to initialize arm"),
    };

    info!("-- Status --");
    // info!("Drivetrain temp: {:?}", robot.drivetrain.temperature());
    info!("Arm temp: {:?}", robot.lady_brown.temperature());

    info!("starting subsystem background tasks");
    robot.lady_brown.task();

    info!("entering competing");
    let controller = robot.controller.clone();

    vexide::task::spawn(async move {
        log::debug!("run");
    })
    .detach();
    sleep(Duration::from_millis(100)).await;

    #[cfg(feature = "no_selector")]
    {
        log::info!("No selector feature enabled, running autonomously");
        autons::test::red(&mut robot).await;
        log::debug!("run");
    }
    robot
        .compete(ControllerSelect::new(
            controller.clone(),
            [
                route!(AutonCategory::Test, "Test red", autons::test::red),
                route!(AutonCategory::Test, "Test blue", autons::test::blue),
                route!(
                    AutonCategory::RedNegative,
                    "Negative rush",
                    autons::negative_rush::red
                ),
                route!(
                    AutonCategory::BlueNegative,
                    "Negative rush",
                    autons::negative_rush::blue
                ),
                route!(
                    AutonCategory::RedPositive,
                    "Positive rush",
                    autons::positive_rush::red
                ),
                route!(
                    AutonCategory::RedPositive,
                    "Positive rush",
                    autons::positive_rush::blue
                ),
            ],
        ))
        .await;
}
