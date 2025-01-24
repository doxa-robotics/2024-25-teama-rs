#![no_main]
#![no_std]
#![feature(never_type)]
#![feature(let_chains)]

extern crate alloc;

mod autonomous;
mod opcontrol;
mod subsystems;
mod utils;

use alloc::{string::ToString, sync::Arc, vec};
use core::time::Duration;

use doxa_selector::{CompeteWithSelector, CompeteWithSelectorExt};
use log::{error, info};
use subsystems::{
    clamp::Clamp,
    doinker::Doinker,
    drivetrain::{Drivetrain, DrivetrainConfig},
    intake::Intake,
    lady_brown::LadyBrown,
};
use utils::{logger, motor_group::MotorGroup};
use vexide::{core::sync::Mutex, prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};

struct Robot {
    controller: Controller,

    drivetrain: Drivetrain,

    intake: Intake,
    clamp: Clamp,
    doinker: Doinker,
    lady_brown: LadyBrown,
}

impl CompeteWithSelector for Robot {
    type Category = autonomous::Category;
    type Return = autonomous::Return;

    fn autonomous_routes<'a, 'b>(
        &'b self,
    ) -> alloc::collections::btree_map::BTreeMap<
        Self::Category,
        impl AsRef<[&'a dyn doxa_selector::AutonRoutine<Self, Return = Self::Return>]>,
    >
    where
        Self: 'a,
    {
        autonomous::autonomous_routes()
    }

    async fn driver(&mut self) {
        info!("Driver starting");

        self.intake.calibrate_reflectivity().await;

        loop {
            let Err(err) = opcontrol::opcontrol(self).await;
            error!("opcontrol crashed, restarting! {}", err);
        }
    }

    fn calibrate_gyro(&mut self) {
        self.drivetrain.calibrate_inertial();
    }

    fn is_gyro_calibrating(&self) -> bool {
        self.drivetrain.is_inertial_calibrating()
    }

    fn diagnostics(&self) -> vec::Vec<(alloc::string::String, alloc::string::String)> {
        vec![
            (
                "Gyro".to_string(),
                if self.drivetrain.is_inertial_calibrating() {
                    "Calibrating".to_string()
                } else {
                    self.drivetrain.inertial_heading().to_string()
                },
            ),
            (
                "Drivetrain temp (C)".to_string(),
                self.drivetrain.temperature().to_string(),
            ),
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
        let intake = self.intake.clone();
        spawn(async move {
            intake.calibrate_reflectivity().await;
        })
        .detach();
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    logger::init().expect("failed to initialize logger");

    let left_motors = Arc::new(Mutex::new(MotorGroup::from_ports(
        vec![
            (peripherals.port_12, true),
            (peripherals.port_14, true),
            (peripherals.port_16, true),
        ],
        Gearset::Blue,
    )));
    let right_motors = Arc::new(Mutex::new(MotorGroup::from_ports(
        vec![
            (peripherals.port_7, false),
            (peripherals.port_8, false),
            (peripherals.port_17, false),
        ],
        Gearset::Blue,
    )));
    let inertial = Arc::new(Mutex::new(InertialSensor::new(peripherals.port_4)));

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            left_motors.clone(),
            right_motors.clone(),
            inertial.clone(),
            DrivetrainConfig {
                drive_p: 0.065,
                drive_i: 0.0,
                drive_d: 0.48,
                drive_tolerance: 5.0,

                turning_p: 0.28,
                turning_i: 0.0,
                turning_d: 0.7,
                turning_tolerance: 5.0,

                tolerance_velocity: 10.0,
                timeout: Duration::from_millis(500),
                wheel_circumference: 165.0,
            },
        ),

        intake: Intake::new(
            Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
            AdiLineTracker::new(peripherals.adi_b),
            AdiLineTracker::new(peripherals.adi_c),
        ),
        clamp: Clamp::new(AdiDigitalOut::with_initial_level(
            peripherals.adi_a,
            vexide::devices::adi::digital::LogicLevel::High,
        )),
        doinker: Doinker::new(AdiDigitalOut::with_initial_level(
            peripherals.adi_f,
            vexide::devices::adi::digital::LogicLevel::Low,
        )),
        lady_brown: LadyBrown::new(
            MotorGroup::new(vec![
                Motor::new_exp(peripherals.port_2, Direction::Forward),
                Motor::new_exp(peripherals.port_3, Direction::Reverse),
            ]),
            RotationSensor::new(peripherals.port_9, Direction::Forward),
        )
        .expect("failed to initialize arm"),
    };

    info!("-- Status --");
    info!("Drivetrain temp: {:?}", robot.drivetrain.temperature());
    info!("Arm temp: {:?}", robot.lady_brown.temperature());
    info!("Intake temp: {:?}", robot.intake.temperature());

    info!("starting subsystem background tasks");
    robot.lady_brown.task();
    robot.intake.task();

    info!("entering competing");
    robot.compete_with_selector(peripherals.display, None).await;
}
