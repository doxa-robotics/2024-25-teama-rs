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

use libdoxa::{
    subsystems::{drivetrain::actions, tracking::wheel::TrackingWheel},
    utils::{settling::Tolerances, vec2::Vec2},
};
use log::{error, info};
use pid::Pid;
use subsystems::{intake::Intake, lady_brown::LadyBrown, Clamp, Doinker, IntakeRaiser};
use utils::logger;
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};
use vexide_motorgroup::MotorGroup;

struct Robot {
    controller: Controller,

    inertial: Rc<RefCell<InertialSensor>>,
    drivetrain: libdoxa::subsystems::drivetrain::Drivetrain,

    intake: Intake,
    intake_raiser: IntakeRaiser,
    doinker: Doinker,
    clamp: Clamp,
    lady_brown: LadyBrown,
}

impl Compete for Robot {
    async fn driver(&mut self) {
        info!("Driver starting");

        loop {
            let Err(err) = opcontrol::opcontrol(self).await;
            error!("opcontrol crashed, restarting in 500ms! {}", err);
            sleep(Duration::from_millis(500)).await;
        }
    }

    async fn autonomous(&mut self) {
        // self.drivetrain
        //     .action(
        //         libdoxa::subsystems::drivetrain::actions::ForwardAction::new(
        //             {
        //                 let mut controller = Pid::new(100.0, Motor::V5_MAX_VOLTAGE);
        //                 controller.p(0.1, Motor::V5_MAX_VOLTAGE);
        //                 controller
        //             },
        //             Tolerances::new()
        //                 .error_tolerance(5.0)
        //                 .velocity_tolerance(5.0)
        //                 .tolerance_duration(Duration::from_millis(200))
        //                 .timeout(Duration::from_millis(1000)),
        //         ),
        //     )
        //     .await;
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
        intake_raiser: IntakeRaiser::new([AdiDigitalOut::new(peripherals.adi_c)]),
        doinker: Doinker::new(
            [
                AdiDigitalOut::new(peripherals.adi_d),
            ],
            [
                AdiDigitalOut::new(peripherals.adi_e),
            ],
            libdoxa::subsystems::pneumatic::MirroredState::Normal,
        ),
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
            ], inertial.clone(), Vec2::default(),0.0),
        ),

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
    robot.compete().await;
}
