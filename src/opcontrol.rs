use core::time::Duration;

use snafu::{ResultExt, Snafu};
use vexide::prelude::*;

use crate::{
    subsystems::{
        clamp::ClampError, doinker::DoinkerError, drivetrain::DrivetrainError,
        lady_brown::LadyBrownState,
    },
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
    #[snafu(display("clamp error: {}", source))]
    Clamp { source: ClampError },
    #[snafu(display("doinker error: {}", source))]
    Doinker { source: DoinkerError },
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

        robot
            .drivetrain
            .set_voltages(
                Motor::V5_MAX_VOLTAGE * left_percent,
                Motor::V5_MAX_VOLTAGE * right_percent,
            )
            .await
            .context(DrivetrainSnafu)?;

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

        if state.button_l2.is_now_pressed() {
            match robot.lady_brown.state().await {
                LadyBrownState::Initial => robot.lady_brown.set_state(LadyBrownState::Intake).await,
                LadyBrownState::Intake => {
                    let arm = robot.lady_brown.clone();
                    let intake = robot.intake.clone();
                    let should_sleep = robot.intake.is_ring_released_in_lady_brown().await;
                    spawn(async move {
                        intake.run(Direction::Forward).await;
                        if should_sleep {
                            sleep(Duration::from_millis(200)).await;
                        }
                        arm.set_state(LadyBrownState::MaxExpansion).await;
                        sleep(Duration::from_millis(300)).await;
                        intake.stop().await;
                    })
                    .detach();
                }
                LadyBrownState::MaxExpansion => robot.lady_brown.set_state(LadyBrownState::Initial).await,
                LadyBrownState::Manual(_) => robot.lady_brown.set_state(LadyBrownState::Initial).await,
            }
        }
        if state.button_up.is_pressed() {
            robot.lady_brown.manual_add(2.0).await;
        }
        if state.button_down.is_pressed() {
            robot.lady_brown.manual_add(-2.0).await;
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
