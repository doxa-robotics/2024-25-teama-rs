use core::time::Duration;

use libdoxa::subsystems::drivetrain::VoltagePair;
use snafu::Snafu;
use vexide::prelude::*;

use crate::{
    subsystems::{drivetrain::DrivetrainError, lady_brown::LadyBrownState},
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
    #[snafu(display("drivetrain error: {}", source))]
    Drivetrain { source: DrivetrainError },
}

pub async fn opcontrol(robot: &mut Robot) -> Result<!, OpcontrolError> {
    robot.intake.stop().await;
    loop {
        let state = robot.controller.state().unwrap_or_default();

        let speed = curve_stick(state.left_stick.y());
        let turn = curve_stick(state.right_stick.x());

        let left_percent = (speed + turn).clamp(-1.0, 1.0);
        let right_percent = (speed - turn).clamp(-1.0, 1.0);

        robot.drivetrain.set_voltage(VoltagePair {
            left: Motor::V5_MAX_VOLTAGE * left_percent,
            right: Motor::V5_MAX_VOLTAGE * right_percent,
        });

        if state.button_r1.is_now_pressed() {
            robot.intake.run(Direction::Forward).await;
        }
        if state.button_l1.is_now_pressed() {
            robot.intake.run(Direction::Reverse).await;
        }
        if state.button_r1.is_now_released() || state.button_l1.is_now_released() {
            robot.intake.stop().await;
        }

        if state.button_l2.is_now_pressed() {
            match robot.lady_brown.state().await {
                LadyBrownState::Initial => robot.lady_brown.set_state(LadyBrownState::Intake).await,
                LadyBrownState::Intake => {
                    robot
                        .lady_brown
                        .set_state(LadyBrownState::MaxExpansion)
                        .await
                }
                LadyBrownState::MaxExpansion => {
                    robot.lady_brown.set_state(LadyBrownState::Initial).await
                }
                LadyBrownState::Manual(_) => {
                    robot.lady_brown.set_state(LadyBrownState::Initial).await
                }
            }
        }
        if state.button_up.is_pressed() {
            robot.lady_brown.manual_add(2.0).await;
        }
        if state.button_down.is_pressed() {
            robot.lady_brown.manual_add(-2.0).await;
        }

        if state.button_a.is_now_pressed() {
            robot.clamp.toggle();
        }

        sleep(Duration::from_millis(10)).await;
    }
}
