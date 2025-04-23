use alloc::sync::Arc;
use core::f64;

use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    prelude::{sleep, spawn, AdiDigitalIn, Motor, Position},
    sync::Mutex,
};
use vexide_motorgroup::{MotorGroup, MotorGroupError};

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
    pub const INTAKE_ANGLE: f64 = Self::INITIAL_ARM_ANGLE + 30.0;
    pub const MAX_EXPANSION_ANGLE: f64 = Self::INITIAL_ARM_ANGLE + 147.0;

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
    limit: AdiDigitalIn,
    gear_ratio: f64,
    pid: pid::Pid<f64>,
    state: LadyBrownState,
}

impl LadyBrownInner {
    async fn update(&mut self) -> Result<(), LadyBrownError> {
        self.pid.setpoint = self.state.angle();
        let current_angle: f64 = self
            .motors
            .position()
            .map_err(|_| LadyBrownError::MotorGet)?
            .as_degrees()
            / self.gear_ratio;
        let output = self.pid.next_control_output(current_angle);
        self.motors.set_voltage(output.output).context(MotorSnafu)?;
        if matches!(self.state, LadyBrownState::Initial)
            && current_angle <= LadyBrownState::INITIAL_ARM_ANGLE + 5.0
        {
            if self.limit.is_high().unwrap_or(true) {
                self.motors
                    .set_position(Position::from_degrees(
                        LadyBrownState::INITIAL_ARM_ANGLE * self.gear_ratio,
                    ))
                    .context(MotorSnafu)?;
            } else {
                self.motors.set_voltage(-4.0).context(MotorSnafu)?;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct LadyBrown(Arc<Mutex<LadyBrownInner>>);

#[derive(Debug, Snafu)]
pub enum LadyBrownError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorGroupError },
    #[snafu(display("motor error get"))]
    MotorGet,
}

impl LadyBrown {
    pub fn new(
        mut motors: MotorGroup,
        gear_ratio: f64,
        limit: AdiDigitalIn,
    ) -> Result<Self, LadyBrownError> {
        motors
            .set_position(Position::from_degrees(
                LadyBrownState::Initial.angle() * gear_ratio,
            ))
            .context(MotorSnafu)?;

        let mut pid = pid::Pid::new(LadyBrownState::Initial.angle(), Motor::EXP_MAX_VOLTAGE);
        pid.p(0.5, f64::MAX);
        pid.i(0.005, 0.3);
        pid.d(0.11, f64::MAX);

        let inner = Arc::new(Mutex::new(LadyBrownInner {
            motors,
            pid,
            limit,
            gear_ratio,
            state: LadyBrownState::default(),
        }));
        let inner_clone = inner.clone();
        spawn(async move {
            loop {
                let mut inner = inner_clone.lock().await;
                if let Err(err) = inner.update().await {
                    error!("arm update error: {}", err);
                }
                drop(inner);

                sleep(super::SUBSYSTEM_UPDATE_PERIOD).await;
            }
        })
        .detach();

        Ok(Self(inner))
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
