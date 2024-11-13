#![no_main]
#![no_std]
#![feature(never_type)]

extern crate alloc;

mod opcontrol;
mod utils;

use alloc::vec;

use utils::{
    drivetrain::{Drivetrain, DrivetrainConfig},
    motor_group::MotorGroup,
};
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};

struct RobotDevices {
    controller: Controller,

    drivetrain: Drivetrain,

    intake: Motor,
    lift: Motor,
    clamp: AdiDigitalOut,
    doinker: AdiDigitalOut,
}

struct Robot {
    devices: RobotDevices,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        // calibrate the inertial sensor
        self.devices
            .drivetrain
            .inertial()
            .calibrate()
            .await
            .unwrap();
        self.devices.drivetrain.turn_for(90.0).await.unwrap();
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
    let robot = Robot {
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
                    drive_p: -0.05,
                    drive_d: 0.0,
                    drive_i: 0.0,
                    drive_tolerance: 5.0,

                    turning_p: 0.01,
                    turning_d: 0.0,
                    turning_i: 0.0,
                    turning_tolerance: 3.0,

                    tolerance_velocity: 5.0,
                    wheel_circumference: 165.0,
                },
            ),

            clamp: AdiDigitalOut::new(peripherals.adi_a),
            doinker: AdiDigitalOut::new(peripherals.adi_f),
            intake: Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
            lift: Motor::new(peripherals.port_21, Gearset::Blue, Direction::Forward), // TODO
        },
    };

    robot.compete().await;
}
