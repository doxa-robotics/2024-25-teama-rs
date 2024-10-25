#![no_main]
#![no_std]
#![feature(never_type)]

extern crate alloc;

mod opcontrol;
mod utils;

use alloc::vec;

use utils::motor_group::MotorGroup;
use vexide::prelude::*;

struct RobotDevices {
    controller: Controller,

    drivetrain_left: MotorGroup,
    drivetrain_right: MotorGroup,

    intake: Motor,
    lift: Motor,
    clamp: AdiSolenoid,
    doinker: AdiSolenoid,
}

struct Robot {
    devices: RobotDevices,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        println!("Driver started");

        loop {
            let Err(err) = opcontrol::opcontrol(&mut self.devices).await;
            println!("opcontrol crashed, restarting! {}", err);
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot {
        devices: RobotDevices {
            controller: peripherals.primary_controller,

            drivetrain_left: MotorGroup::from_ports(
                vec![
                    (peripherals.port_12, true),
                    (peripherals.port_14, true),
                    (peripherals.port_16, true),
                ],
                Gearset::Blue,
            ),
            drivetrain_right: MotorGroup::from_ports(
                vec![
                    (peripherals.port_7, false),
                    (peripherals.port_8, false),
                    (peripherals.port_17, false),
                ],
                Gearset::Blue,
            ),

            clamp: AdiSolenoid::new(peripherals.adi_a),
            doinker: AdiSolenoid::new(peripherals.adi_b), // TODO
            intake: Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
            lift: Motor::new(peripherals.port_21, Gearset::Blue, Direction::Forward), // TODO
        },
    };

    robot.compete().await;
}
