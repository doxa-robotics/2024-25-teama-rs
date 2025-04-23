use alloc::rc::Rc;
use core::{cell::RefCell, f64};

use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    prelude::{sleep, spawn, AdiDigitalIn, Motor, Position},
    task::Task,
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
    pub const MAX_EXPANSION_ANGLE: f64 = Self::INITIAL_ARM_ANGLE + 150.0;

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
    gear_ratio: f64,
    state: LadyBrownState,
}

#[derive(Debug, Clone)]
pub struct LadyBrown {
    inner: Rc<RefCell<LadyBrownInner>>,
    _task: Rc<Task<()>>,
}

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

        let inner = Rc::new(RefCell::new(LadyBrownInner {
            gear_ratio,
            state: LadyBrownState::default(),
        }));
        let inner_clone = inner.clone();
        let task = spawn(async move {
            loop {
                if let Err(err) = LadyBrown::update(
                    &mut motors,
                    &mut pid,
                    &limit,
                    gear_ratio,
                    &mut inner_clone.borrow_mut().state,
                ) {
                    error!("arm update error: {}", err);
                }

                sleep(super::SUBSYSTEM_UPDATE_PERIOD).await;
            }
        });

        Ok(Self {
            inner,
            _task: Rc::new(task),
        })
    }

    fn update(
        motors: &mut MotorGroup,
        pid: &mut pid::Pid<f64>,
        limit: &AdiDigitalIn,
        gear_ratio: f64,
        state: &mut LadyBrownState,
    ) -> Result<(), LadyBrownError> {
        pid.setpoint = state.angle();
        let current_angle: f64 = motors
            .position()
            .map_err(|_| LadyBrownError::MotorGet)?
            .as_degrees()
            / gear_ratio;
        let output = pid.next_control_output(current_angle);
        motors.set_voltage(output.output).context(MotorSnafu)?;
        if matches!(state, LadyBrownState::Initial)
            && current_angle <= LadyBrownState::INITIAL_ARM_ANGLE + 5.0
        {
            if limit.is_high().unwrap_or(true) {
                motors
                    .set_position(Position::from_degrees(
                        LadyBrownState::INITIAL_ARM_ANGLE * gear_ratio,
                    ))
                    .context(MotorSnafu)?;
            } else {
                motors.set_voltage(-4.0).context(MotorSnafu)?;
            }
        }
        Ok(())
    }

    pub fn manual_add(&mut self, angle_change: f64) {
        self.inner.borrow_mut().state.add(angle_change);
    }

    pub fn state(&self) -> LadyBrownState {
        self.inner.borrow().state
    }

    pub fn set_state(&self, state: LadyBrownState) {
        self.inner.borrow_mut().state = state;
    }
}
