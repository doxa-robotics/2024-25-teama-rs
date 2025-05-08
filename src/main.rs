#![no_main]
#![no_std]
#![feature(never_type)]
#![feature(let_chains)]

extern crate alloc;

mod autons;
mod graphics;
mod opcontrol;
mod subsystems;
mod utils;

use alloc::{rc::Rc, vec};
use core::{cell::RefCell, time::Duration};

use ::autons::prelude::{SelectCompete, SelectCompeteExt};
use autons::AutonCategory;
use autons_controller::{route, ControllerSelect};
use libdoxa::{subsystems::tracking::wheel::TrackingWheel, utils::pose::Pose};
use log::{error, info};
use subsystems::{intake::Intake, lady_brown::LadyBrown, Clamp, Doinker, IntakeRaiser};
use utils::logger;
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};
use vexide_motorgroup::MotorGroup;

const DRIVETRAIN_CIRCUMFERENCE: f64 = 164.0;

struct Robot {
    controller: Rc<RefCell<Controller>>,
    is_selecting: Rc<RefCell<bool>>,

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
        self.intake.set_accept(None);
        info!("Driver starting");

        loop {
            let Err(err) = opcontrol::opcontrol(self).await;
            error!("opcontrol crashed, restarting in 500ms! {}", err);
            sleep(Duration::from_millis(500)).await;
        }
    }

    async fn before_route(&mut self) {
        // self.drivetrain.set_max_voltage(Motor::V5_MAX_VOLTAGE * 0.7);
        info!("Auton start");
    }

    async fn after_route(&mut self) {
        self.drivetrain.set_max_voltage(Motor::V5_MAX_VOLTAGE);
        info!("Auton end");
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    logger::init().expect("failed to initialize logger");

    let left_motors = Rc::new(RefCell::new(MotorGroup::new(vec![
        Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
    ])));
    let right_motors = Rc::new(RefCell::new(MotorGroup::new(vec![
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
    ])));
    let inertial = Rc::new(RefCell::new(InertialSensor::new(peripherals.port_18)));
    let tracking = Rc::new(RefCell::new(
        libdoxa::subsystems::tracking::TrackingSubsystem::new::<RotationSensor, _, _>(
            [TrackingWheel::new(
                158.0,
                43.0,
                libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Perpendicular,
                RotationSensor::new(peripherals.port_19, Direction::Forward), // TODO: verify this is the right port (was 19)
            )],
            [
                TrackingWheel::new(
                    DRIVETRAIN_CIRCUMFERENCE,
                    0.0,
                    libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
                    left_motors.clone(),
                ),
                TrackingWheel::new(
                    DRIVETRAIN_CIRCUMFERENCE,
                    0.0,
                    libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
                    right_motors.clone(),
                ),
            ],
            inertial.clone(),
            Pose::default(),
        ),
    ));

    #[allow(unused_mut)]
    let mut robot = Robot {
        controller: Rc::new(RefCell::new(peripherals.primary_controller)),
        is_selecting: Rc::new(RefCell::new(true)),

        intake_raiser: IntakeRaiser::new([AdiDigitalOut::new(peripherals.adi_c)]),

        doinker: Doinker::new(
            [AdiDigitalOut::new(peripherals.adi_d)],
            [AdiDigitalOut::new(peripherals.adi_e)],
            libdoxa::subsystems::pneumatic::MirroredState::Normal,
        ),

        drivetrain: libdoxa::subsystems::drivetrain::Drivetrain::new(
            left_motors.clone(),
            right_motors.clone(),
            DRIVETRAIN_CIRCUMFERENCE,
            Motor::V5_MAX_VOLTAGE,
            tracking.clone(),
            3000.0,
        ),
        tracking: tracking.clone(),

        intake: Intake::new(
            Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
            VisionSensor::new(peripherals.port_11),
            AdiLineTracker::new(peripherals.adi_g),
            OpticalSensor::new(peripherals.port_15),
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

    info!("entering competing");
    let controller = robot.controller.clone();
    let is_selecting = robot.is_selecting.clone();

    sleep(Duration::from_millis(100)).await;

    #[cfg(feature = "no_selector")]
    {
        let mut debug_render = DebugRender::new(peripherals.display);
        debug_render.render();

        log::info!("No selector feature enabled, running autonomously after inertial calibration");
        while inertial.borrow().is_calibrating().unwrap() {
            vexide::time::sleep(Duration::from_millis(100)).await;
        }
        while inertial.borrow().heading().unwrap_or(0.0) == 0.0 {
            vexide::time::sleep(Duration::from_millis(100)).await;
        }
        let start = Instant::now();
        autons::positive_new::blue(&mut robot).await;
        // autons::test::red(&mut robot).await;
        log::info!("Auton finished in {:?}", start.elapsed());
    }
    #[cfg(not(feature = "no_selector"))]
    {
        graphics::render_banner(peripherals.display);
    }
    robot
        .compete(ControllerSelect::new(
            controller.clone(),
            is_selecting.clone(),
            [
                route!(AutonCategory::Test, "Test red", autons::test::red),
                route!(AutonCategory::Test, "Test blue", autons::test::blue),
                route!(AutonCategory::Test, "Forward", autons::forward::route),
                route!(
                    AutonCategory::RedNegative,
                    "Rush",
                    autons::negative_rush::red
                ),
                route!(
                    AutonCategory::BlueNegative,
                    "Rush",
                    autons::negative_rush::blue
                ),
                route!(
                    AutonCategory::RedNegative,
                    "Middle",
                    autons::negative_middle::red
                ),
                route!(
                    AutonCategory::BlueNegative,
                    "Middle",
                    autons::negative_middle::blue
                ),
                route!(
                    AutonCategory::RedNegative,
                    "Safe",
                    autons::negative_safe::red
                ),
                route!(
                    AutonCategory::BlueNegative,
                    "Safe",
                    autons::negative_safe::blue
                ),
                route!(
                    AutonCategory::RedPositive,
                    "Rush",
                    autons::positive_rush::red
                ),
                route!(
                    AutonCategory::BluePositive,
                    "Rush",
                    autons::positive_rush::blue
                ),
                route!(AutonCategory::RedPositive, "AWP", autons::positive_awp::red),
                route!(
                    AutonCategory::BluePositive,
                    "AWP",
                    autons::positive_awp::blue
                ),
                route!(AutonCategory::RedPositive, "New", autons::positive_new::red),
                route!(
                    AutonCategory::BluePositive,
                    "New",
                    autons::positive_new::blue
                ),
            ],
        ))
        .await;
}
