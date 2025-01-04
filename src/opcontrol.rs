use core::time::Duration;

use snafu::{ResultExt, Snafu};
use vexide::{
    core::time::Instant,
    devices::{controller::ControllerError, smart::motor::MotorError},
    prelude::*,
};

use crate::{
    subsystems::{
        arm::{ArmError, ArmState},
        clamp::ClampError,
        doinker::DoinkerError,
        intake::{IntakeError, IntakeState},
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
    #[snafu(display("arm error: {}", source))]
    Arm { source: ArmError },
}

pub async fn opcontrol(robot: &mut Robot) -> Result<!, OpcontrolError> {
    loop {
        let state = devices.controller.state().unwrap_or_default();

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

        if state.button_r1.is_pressed() {
            if matches!(robot.arm.state, ArmState::Intake) {
                if !matches!(robot.intake.state, IntakeState::ArmIntake { .. }) {
                    robot.intake.arm_intake();
                }
            } else {
                robot.intake.run(Direction::Forward);
            }
        } else if state.button_l1.is_pressed() {
            robot.intake.run(Direction::Reverse);
        } else if matches!(
            robot.intake.state,
            IntakeState::ArmIntake { .. } | IntakeState::Forward | IntakeState::Reverse
        ) {
            robot.intake.stop();
        }
        if state.button_y.is_now_pressed() {
            robot.intake.partial_intake();
        }
        robot.intake.update().context(IntakeSnafu)?;

        if state.button_x.is_now_pressed() {
            robot.arm.next_state();
            if matches!(robot.arm.state, ArmState::Delay { .. }) {
                robot.intake.state = IntakeState::ForwardUntil {
                    end: Instant::now() + Duration::from_millis(500),
                };
            }
        }
        if state.button_l2.is_pressed() {
            robot.arm.state.add(2.0);
        }
        if state.button_r2.is_pressed() {
            robot.arm.state.add(-2.0);
        }
        robot.arm.update().context(ArmSnafu)?;

        if state.button_a.is_now_pressed() {
            robot.clamp.toggle().context(ClampSnafu)?;
        }

        if state.button_b.is_now_pressed() {
            robot.doinker.toggle().context(DoinkerSnafu)?;
        }

        // TODO: lift control

        sleep(Duration::from_millis(10)).await;
    }
}
