use core::time::Duration;

use snafu::{ResultExt, Snafu};
use vexide::{devices::smart::motor::MotorError, prelude::*};

use crate::{
    subsystems::{arm::ArmState, clamp::ClampError, doinker::DoinkerError},
    Robot,
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
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
    #[snafu(display("clamp error: {}", source))]
    Clamp { source: ClampError },
    #[snafu(display("doinker error: {}", source))]
    Doinker { source: DoinkerError },
}

pub async fn opcontrol(robot: &mut Robot) -> Result<!, OpcontrolError> {
    loop {
        let state = robot.controller.state().unwrap_or_default();

        let speed = curve_stick(state.left_stick.y());
        let turn = curve_stick(state.right_stick.x());

        let left_percent = (speed + turn).clamp(-1.0, 1.0);
        let right_percent = (speed - turn).clamp(-1.0, 1.0);

        robot
            .drivetrain
            .left()
            .set_voltage(Motor::V5_MAX_VOLTAGE * left_percent)
            .context(MotorSnafu)?;
        robot
            .drivetrain
            .right()
            .set_voltage(Motor::V5_MAX_VOLTAGE * right_percent)
            .context(MotorSnafu)?;

        if state.button_r1.is_now_pressed() {
            robot.intake.run(Direction::Forward).await;
        }
        if state.button_l1.is_now_pressed() {
            robot.intake.run(Direction::Reverse).await;
        }
        if state.button_r1.is_now_released() || state.button_l1.is_now_released() {
            robot.intake.stop().await;
        }
        if state.button_y.is_now_pressed() {
            robot.intake.partial_intake().await;
        }

        if state.button_r2.is_now_pressed() {
            match robot.arm.state().await {
                ArmState::Initial => robot.arm.set_state(ArmState::Intake).await,
                ArmState::Intake => {
                    let arm = robot.arm.clone();
                    let intake = robot.intake.clone();
                    spawn(async move {
                        intake.run(Direction::Forward).await;
                        sleep(Duration::from_millis(200)).await;
                        arm.set_state(ArmState::MaxExpansion).await;
                        sleep(Duration::from_millis(300)).await;
                        intake.stop().await;
                    })
                    .detach();
                }
                ArmState::MaxExpansion => robot.arm.set_state(ArmState::Initial).await,
                ArmState::Manual(_) => robot.arm.set_state(ArmState::Initial).await,
            }
        }
        if state.button_up.is_pressed() {
            robot.arm.manual_add(2.0).await;
        }
        if state.button_down.is_pressed() {
            robot.arm.manual_add(-2.0).await;
        }

        if state.button_a.is_now_pressed() {
            robot.clamp.toggle().context(ClampSnafu)?;
        }

        if state.button_b.is_now_pressed() {
            robot.doinker.toggle().context(DoinkerSnafu)?;
        }

        sleep(Duration::from_millis(10)).await;
    }
}
