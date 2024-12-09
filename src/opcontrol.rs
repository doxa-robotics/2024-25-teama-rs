use core::{fmt::Display, time::Duration};

use snafu::Snafu;
use vexide::{
    devices::{controller::ControllerError, smart::motor::MotorError, PortError},
    prelude::*,
};

use crate::RobotDevices;

fn curve_stick(input: f64) -> f64 {
    let raw = input.powf(2.0);
    if input >= 0.0 {
        raw
    } else {
        -raw
    }
}

#[derive(Debug, Snafu)]
pub enum OpcontrolError {
    #[snafu(display("controller error: {}", source))]
    Controller { source: ControllerError },
    #[snafu(display("motor error: {}", source))]
    Intake { source: IntakeError },
    #[snafu(display("port error: {}", source))]
    Port { source: PortError },
}

pub async fn opcontrol(devices: &mut RobotDevices) -> Result<!, OpcontrolError> {
    loop {
        let state = devices.controller.state()?;

        let speed = curve_stick(state.left_stick.y());
        let turn = curve_stick(state.right_stick.x());

        let left_percent = (speed + turn).clamp(-1.0, 1.0);
        let right_percent = (speed - turn).clamp(-1.0, 1.0);

        devices
            .drivetrain
            .left()
            .set_voltage(Motor::V5_MAX_VOLTAGE * left_percent)
            .map_err(OpcontrolError::Motor)?;
        devices
            .drivetrain
            .right()
            .set_voltage(Motor::V5_MAX_VOLTAGE * right_percent)
            .map_err(OpcontrolError::Motor)?;

        if state.button_r1.is_pressed() {
            devices.intake.run(Direction::Forward)?;
        } else if state.button_l1.is_pressed() {
            devices.intake.run(Direction::Reverse)?;
        } else {
            devices.intake.stop()?;
        }

        if state.button_a.is_now_pressed() {
            devices.clamp.toggle()?;
        }

        if state.button_b.is_now_pressed() {
            devices.doinker.toggle()?;
        }

        // TODO: lift control

        sleep(Duration::from_millis(10)).await;
    }
}
