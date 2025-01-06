#![no_main]
#![no_std]
#![feature(never_type)]

extern crate alloc;

mod autonomous;
mod opcontrol;
mod subsystems;
mod utils;

use alloc::{string::ToString, vec};
use core::time::Duration;

use doxa_selector::{CompeteWithSelector, CompeteWithSelectorExt};
use log::{error, info};
use subsystems::{
    arm::Arm,
    clamp::Clamp,
    doinker::Doinker,
    drivetrain::{Drivetrain, DrivetrainConfig},
    intake::Intake,
};
use utils::{logger, motor_group::MotorGroup};
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};

struct Robot {
    controller: Controller,

    drivetrain: Drivetrain,

    intake: Intake,
    clamp: Clamp,
    doinker: Doinker,
    arm: Arm,
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
                self.arm.temperature().to_string(),
            ),
            (
                "Intake temp (C)".to_string(),
                self.intake.temperature().to_string(),
            ),
        ]
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    logger::init().expect("failed to initialize logger");

    let robot = Robot {
        controller: peripherals.primary_controller,

        drivetrain: Drivetrain::new(
            MotorGroup::from_ports(
                vec![
                    (peripherals.port_12, true),
                    (peripherals.port_14, true),
                    (peripherals.port_16, true),
                ],
                Gearset::Blue,
            ),
            MotorGroup::from_ports(
                vec![
                    (peripherals.port_7, false),
                    (peripherals.port_8, false),
                    (peripherals.port_17, false),
                ],
                Gearset::Blue,
            ),
            InertialSensor::new(peripherals.port_18),
            DrivetrainConfig {
                drive_p: 0.3,
                drive_i: 0.0,
                drive_d: 0.0,
                drive_tolerance: 5.0,

                turning_p: 0.3,
                turning_i: 0.001,
                turning_d: 0.1,
                turning_tolerance: 3.0,

                tolerance_velocity: 5.0,
                timeout: Duration::from_secs(3),
                wheel_circumference: 165.0,
            },
        ),

        intake: Intake::new(
            Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
            AdiAnalogIn::new(peripherals.adi_b),
        ),
        clamp: Clamp::new(AdiDigitalOut::new(peripherals.adi_a)),
        doinker: Doinker::new(AdiDigitalOut::new(peripherals.adi_f)),
        arm: Arm::new(
            MotorGroup::new(vec![
                Motor::new_exp(peripherals.port_2, Direction::Forward),
                Motor::new_exp(peripherals.port_3, Direction::Reverse),
            ]),
            RotationSensor::new(peripherals.port_9, Direction::Forward),
        )
        .expect("failed to initialize arm"),
    };

    info!("starting subsystem background tasks");
    robot.arm.spawn_update();
    robot.intake.spawn_update();

    info!("competing");
    robot.compete_with_selector(peripherals.display).await;
}
