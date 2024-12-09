use core::time::Duration;

use snafu::{ResultExt, Snafu};
use vexide::{
    devices::{controller::ControllerError, smart::motor::MotorError},
    prelude::*,
};

use crate::{
    subsystems::{clamp::ClampError, doinker::DoinkerError, intake::IntakeError},
    RobotDevices,
};

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
    #[snafu(display("intake error: {}", source))]
    Intake { source: IntakeError },
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
    #[snafu(display("clamp error: {}", source))]
    Clamp { source: ClampError },
    #[snafu(display("doinker error: {}", source))]
    Doinker { source: DoinkerError },
}

pub async fn opcontrol(devices: &mut RobotDevices) -> Result<!, OpcontrolError> {
    loop {
        let state = devices.controller.state().context(ControllerSnafu)?;

        let speed = curve_stick(state.left_stick.y());
        let turn = curve_stick(state.right_stick.x());

        let left_percent = (speed + turn).clamp(-1.0, 1.0);
        let right_percent = (speed - turn).clamp(-1.0, 1.0);

        devices
            .drivetrain
            .left()
            .set_voltage(Motor::V5_MAX_VOLTAGE * left_percent)
            .context(MotorSnafu)?;
        devices
            .drivetrain
            .right()
            .set_voltage(Motor::V5_MAX_VOLTAGE * right_percent)
            .context(MotorSnafu)?;

        if state.button_r1.is_pressed() {
            devices
                .intake
                .run(Direction::Forward)
                .context(IntakeSnafu)?;
        } else if state.button_l1.is_pressed() {
            devices
                .intake
                .run(Direction::Reverse)
                .context(IntakeSnafu)?;
        } else {
            devices.intake.stop().context(IntakeSnafu)?;
        }

        if state.button_a.is_now_pressed() {
            devices.clamp.toggle().context(ClampSnafu)?;
        }

        if state.button_b.is_now_pressed() {
            devices.doinker.toggle().context(DoinkerSnafu)?;
        }

        // TODO: lift control

        sleep(Duration::from_millis(10)).await;
    }
}
