use core::{fmt::Display, time::Duration};

use vexide::{
    devices::{controller::ControllerError, smart::motor::MotorError, PortError},
    prelude::*,
};

use crate::{utils::motor_group::MotorGroup, RobotDevices};

fn curve_stick(input: f64) -> f64 {
    let raw = input.powf(2.0);
    if input >= 0.0 {
        raw
    } else {
        -raw
    }
}

pub enum OpcontrolError {
    Controller(ControllerError),
    Motor(MotorError),
    Port(PortError),
}

impl Display for OpcontrolError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            OpcontrolError::Controller(err) => write!(f, "controller error: {}", err),
            OpcontrolError::Motor(err) => write!(f, "motor error: {}", err),
            OpcontrolError::Port(err) => write!(f, "port error: {}", err),
        }
    }
}

pub async fn opcontrol(devices: &mut RobotDevices) -> Result<!, OpcontrolError> {
    loop {
        let state = devices
            .controller
            .state()
            .map_err(OpcontrolError::Controller)?;

        let speed = curve_stick(state.left_stick.y());
        let turn = curve_stick(state.right_stick.x());

        let left_percent = (speed - turn).clamp(-1.0, 1.0);
        let right_percent = (speed + turn).clamp(-1.0, 1.0);

        devices
            .drivetrain_left
            .set_voltage(Motor::V5_MAX_VOLTAGE * left_percent)
            .map_err(OpcontrolError::Motor)?;
        devices
            .drivetrain_right
            .set_voltage(Motor::V5_MAX_VOLTAGE * right_percent)
            .map_err(OpcontrolError::Motor)?;

        if state.right_trigger_1.is_pressed() {
            devices
                .intake
                .set_velocity(600)
                .map_err(OpcontrolError::Motor)?;
        } else if state.left_trigger_1.is_pressed() {
            devices
                .intake
                .set_velocity(-600)
                .map_err(OpcontrolError::Motor)?;
        } else {
            devices
                .intake
                .brake(BrakeMode::Brake)
                .map_err(OpcontrolError::Motor)?;
        }

        if state.button_a.is_now_pressed() {
            devices.clamp.toggle().map_err(OpcontrolError::Port)?;
        }

        if state.button_b.is_now_pressed() {
            devices.doinker.toggle().map_err(OpcontrolError::Port)?;
        }

        // TODO: lift control

        sleep(Duration::from_millis(10)).await;
    }
}
