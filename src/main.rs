#![no_main]
#![no_std]
#![feature(never_type)]

extern crate alloc;

mod autonomous;
mod opcontrol;
mod utils;

use alloc::{boxed::Box, vec};
use core::{pin::pin, time::Duration};

use autonomous::AutonomousRoutine;
use utils::{
    drivetrain::{Drivetrain, DrivetrainConfig},
    intake::Intake,
    motor_group::MotorGroup,
};
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};

struct RobotDevices {
    controller: Controller,

    drivetrain: Drivetrain,

    intake: Intake,
    clamp: AdiDigitalOut,
    doinker: AdiDigitalOut,
}

struct Robot {
    devices: RobotDevices,
    autonomous_routine: Box<dyn AutonomousRoutine>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        pin!(self.autonomous_routine.run(&mut self.devices)).await;
        println!("Autonomous done!");
    }

    async fn driver(&mut self) {
        println!("Driver started");

        loop {
            let Err(err) = opcontrol::opcontrol(&mut self.devices).await;
            println!("opcontrol crashed, restarting! {}", err);
        }
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    let mut robot = Robot {
        devices: RobotDevices {
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
            clamp: AdiDigitalOut::new(peripherals.adi_a),
            doinker: AdiDigitalOut::with_initial_level(
                peripherals.adi_f,
                vexide::devices::adi::digital::LogicLevel::High,
            ),
        },
        autonomous_routine: Box::new(autonomous::test::Test {}),
    };

    println!("competing");
    robot
        .devices
        .drivetrain
        .inertial()
        .calibrate()
        .await
        .unwrap();
    robot.compete().await;
}
