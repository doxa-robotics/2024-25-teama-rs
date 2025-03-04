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

        loop {
            let Err(err) = opcontrol::opcontrol(self).await;
            error!("opcontrol crashed, restarting in 500ms! {}", err);
            sleep(Duration::from_millis(500)).await;
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
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    logger::init().expect("failed to initialize logger");

    let left_motors = Arc::new(Mutex::new(MotorGroup::from_ports(
        vec![
            (peripherals.port_7, true),
            (peripherals.port_6, true),
            (peripherals.port_5, true),
        ],
        Gearset::Blue,
    )));
    let right_motors = Arc::new(Mutex::new(MotorGroup::from_ports(
        vec![
            (peripherals.port_10, false),
            (peripherals.port_9, false),
            (peripherals.port_8, false),
        ],
        Gearset::Blue,
    )));
    let inertial = Arc::new(Mutex::new(InertialSensor::new(peripherals.port_20)));

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

        intake: Intake::new(Motor::new(
            peripherals.port_4,
            Gearset::Blue,
            Direction::Reverse,
        )),
        clamp: Clamp::new(AdiDigitalOut::new(peripherals.adi_a)),
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
    info!("Drivetrain temp: {:?}", robot.drivetrain.temperature());
    info!("Arm temp: {:?}", robot.lady_brown.temperature());
    info!("Intake temp: {:?}", robot.intake.temperature());

    info!("starting subsystem background tasks");
    robot.lady_brown.task();
    robot.intake.task();

    info!("entering competing");
    robot.compete_with_selector(peripherals.display, None).await;
}
