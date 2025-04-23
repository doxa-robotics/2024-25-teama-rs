use core::time::Duration;

use libdoxa::subsystems::drivetrain::VoltagePair;
use snafu::Snafu;
use vexide::prelude::*;

use crate::{subsystems::lady_brown::LadyBrownState, Robot};

fn curve_drive(input: f64) -> f64 {
    let raw = input.powf(2.0);
    if input >= 0.0 {
        raw
    } else {
        -raw
    }
}

fn curve_turn(input: f64) -> f64 {
    let raw = input.powf(2.0);
    (if input >= 0.0 { raw } else { -raw }) / 2.0
}

#[derive(Debug, Snafu)]
pub enum OpcontrolError {}

pub async fn opcontrol(robot: &mut Robot) -> Result<!, OpcontrolError> {
    robot.intake.stop();
    loop {
        let state = robot
            .controller
            .try_lock()
            .and_then(|c| c.state().ok())
            .unwrap_or_default();

        let speed = curve_drive(state.left_stick.y());
        let turn = curve_turn(state.right_stick.x());

        let left_percent = (speed + turn).clamp(-1.0, 1.0);
        let right_percent = (speed - turn).clamp(-1.0, 1.0);

        robot.drivetrain.set_voltage(VoltagePair {
            left: Motor::V5_MAX_VOLTAGE * left_percent,
            right: Motor::V5_MAX_VOLTAGE * right_percent,
        });

        if state.button_r1.is_now_pressed() {
            robot.intake.run(Direction::Forward);
        }
        if state.button_l1.is_now_pressed() {
            robot.intake.run(Direction::Reverse);
        }
        if state.button_r1.is_now_released() || state.button_l1.is_now_released() {
            if matches!(robot.lady_brown.state(), LadyBrownState::Intake) {
                robot.intake.stop_hold();
            } else {
                robot.intake.stop();
            }
        }

        if state.button_y.is_now_pressed() {
            if robot.doinker.left.extended() {
                robot.doinker.left.retract();
            } else {
                robot.doinker.left.extend();
                robot.doinker.right.retract();
            }
        }
        if state.button_a.is_now_pressed() {
            if robot.doinker.right.extended() {
                robot.doinker.right.retract();
            } else {
                robot.doinker.right.extend();
                robot.doinker.left.retract();
            }
        }
        if state.button_x.is_pressed() {
            robot.intake_raiser.extend();
        } else {
            robot.intake_raiser.retract();
        }

        if state.button_l2.is_now_pressed() {
            match robot.lady_brown.state() {
                LadyBrownState::Initial => robot.lady_brown.set_state(LadyBrownState::Intake),
                LadyBrownState::Intake => {
                    robot.intake.stop();
                    robot.lady_brown.set_state(LadyBrownState::MaxExpansion);
                }
                LadyBrownState::MaxExpansion => robot.lady_brown.set_state(LadyBrownState::Initial),
                LadyBrownState::Manual(_) => robot.lady_brown.set_state(LadyBrownState::Initial),
            }
        }
        if state.button_up.is_pressed() {
            robot.lady_brown.manual_add(2.0);
        }
        if state.button_down.is_pressed() {
            robot.lady_brown.manual_add(-2.0);
        }

        if state.button_b.is_now_pressed() {
            robot.clamp.toggle();
        }

        // Macros
        if state.button_left.is_now_pressed() {
            let intake = robot.intake.clone();
            let lady_brown = robot.lady_brown.clone();
            spawn(async move {
                intake.stop();
                lady_brown.set_state(LadyBrownState::MaxExpansion);
                sleep(Duration::from_millis(800)).await;
                lady_brown.set_state(LadyBrownState::Intake);
                sleep(Duration::from_millis(300)).await;
                intake.run(Direction::Forward);
                sleep(Duration::from_millis(600)).await;
                intake.stop();
                lady_brown.set_state(LadyBrownState::MaxExpansion);
            })
            .detach();
        }

        sleep(Duration::from_millis(10));
    }
}
