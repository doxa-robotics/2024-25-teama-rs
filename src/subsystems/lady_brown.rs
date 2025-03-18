use alloc::sync::Arc;
use core::f64;

use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    devices::smart::motor::MotorError,
    prelude::{sleep, spawn, Motor, Position},
    sync::Mutex,
};

use crate::utils::motor_group::MotorGroup;

#[derive(Debug, Clone, Copy)]
pub enum LadyBrownState {
    Initial,
    Intake,
    MaxExpansion,
    Manual(f64),
}

impl Default for LadyBrownState {
    fn default() -> Self {
        Self::Initial
    }
}

impl LadyBrownState {
    pub const INITIAL_ARM_ANGLE: f64 = 10.0;
    pub const INTAKE_ANGLE: f64 = Self::INITIAL_ARM_ANGLE + 23.5;
    pub const MAX_EXPANSION_ANGLE: f64 = Self::INITIAL_ARM_ANGLE + 160.0;

    pub fn angle(&self) -> f64 {
        match self {
            Self::Intake => Self::INTAKE_ANGLE,
            Self::MaxExpansion => Self::MAX_EXPANSION_ANGLE,
            Self::Initial => Self::INITIAL_ARM_ANGLE,
            Self::Manual(angle) => *angle,
        }
    }

    pub fn add(&mut self, angle: f64) {
        let new_angle = self.angle() + angle;
        *self = Self::Manual(new_angle.clamp(Self::INITIAL_ARM_ANGLE, Self::MAX_EXPANSION_ANGLE))
    }
}

#[derive(Debug)]
struct LadyBrownInner {
    motors: MotorGroup,
    gear_ratio: f64,
    pid: pid::Pid<f64>,
    state: LadyBrownState,
}

impl LadyBrownInner {
    async fn update(&mut self) -> Result<(), LadyBrownError> {
        self.pid.setpoint = self.state.angle();
        let current_angle: f64 =
            self.motors.position().context(MotorSnafu)?.as_degrees() / self.gear_ratio;
        let output = self.pid.next_control_output(current_angle);
        self.motors.set_voltage(output.output).context(MotorSnafu)?;
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct LadyBrown(Arc<Mutex<LadyBrownInner>>);

#[derive(Debug, Snafu)]
pub enum LadyBrownError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
}

impl LadyBrown {
    pub fn new(mut motors: MotorGroup, gear_ratio: f64) -> Result<Self, LadyBrownError> {
        motors
            .set_position(Position::from_degrees(
                LadyBrownState::Initial.angle() * gear_ratio,
            ))
            .context(MotorSnafu)?;

        let mut pid = pid::Pid::new(LadyBrownState::Initial.angle(), Motor::EXP_MAX_VOLTAGE);
        pid.p(0.5, f64::MAX);
        pid.i(0.005, 0.3);
        pid.d(0.11, f64::MAX);

        Ok(Self(Arc::new(Mutex::new(LadyBrownInner {
            motors,
            pid,
            gear_ratio,
            state: LadyBrownState::default(),
        }))))
    }

    pub async fn reset_rotation(&self) -> Result<(), LadyBrownError> {
        let mut inner = self.0.lock().await;
        let new_position = Position::from_degrees(inner.state.angle() * inner.gear_ratio);
        inner
            .motors
            .set_position(new_position)
            .context(MotorSnafu)?;
        Ok(())
    }

    pub fn task(&self) {
        let inner = self.0.clone();
        spawn(async move {
            loop {
                let mut inner = inner.lock().await;
                if let Err(err) = inner.update().await {
                    error!("arm update error: {}", err);
                }
                drop(inner);

                sleep(super::SUBSYSTEM_UPDATE_PERIOD).await;
            }
        })
        .detach();
    }

    pub async fn manual_add(&mut self, angle_change: f64) {
        let mut inner = self.0.lock().await;
        inner.state.add(angle_change);
    }

    pub async fn state(&self) -> LadyBrownState {
        self.0.lock().await.state
    }

    pub async fn set_state(&self, state: LadyBrownState) {
        self.0.lock().await.state = state;
    }

    pub fn temperature(&self) -> f64 {
        match self.0.try_lock() {
            Some(inner) => inner.motors.temperature().unwrap_or(0.0),
            None => 0.0,
        }
    }
}
