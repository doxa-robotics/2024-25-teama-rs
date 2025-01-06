use alloc::{boxed::Box, sync::Arc};
use core::{f64, time::Duration};

use snafu::{ResultExt, Snafu};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::smart::motor::MotorError,
    prelude::{sleep, spawn, Direction, Motor, Position, RotationSensor},
};

use crate::utils::motor_group::MotorGroup;

#[derive(Debug, Clone)]
pub enum ArmState {
    Initial,
    Intake,
    MaxExpansion,
    Manual(f64),
    Delay {
        instant: Instant,
        current: Box<ArmState>,
        target: Box<ArmState>,
    },
}

impl Default for ArmState {
    fn default() -> Self {
        Self::Initial
    }
}

impl ArmState {
    pub const INITIAL_ARM_ANGLE: f64 = 10.0;
    pub const INTAKE_ANGLE: f64 = Self::INITIAL_ARM_ANGLE + 40.0;
    pub const MAX_EXPANSION_ANGLE: f64 = Self::INITIAL_ARM_ANGLE + 165.0;

    pub fn angle(&self) -> f64 {
        match self {
            Self::Intake => Self::INTAKE_ANGLE,
            Self::MaxExpansion => Self::MAX_EXPANSION_ANGLE,
            Self::Initial => Self::INITIAL_ARM_ANGLE,
            Self::Manual(angle) => *angle,
            Self::Delay { current, .. } => current.angle(),
        }
    }

    pub fn add(&mut self, angle: f64) {
        let new_angle = self.angle() + angle;
        *self = Self::Manual(new_angle.clamp(Self::INITIAL_ARM_ANGLE, Self::MAX_EXPANSION_ANGLE))
    }
}

#[derive(Debug)]
pub struct Arm {
    motors: Arc<Mutex<MotorGroup>>,
    rotation: Arc<Mutex<RotationSensor>>,
    state: Arc<Mutex<ArmState>>,
}

#[derive(Debug, Snafu)]
pub enum ArmError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
    #[snafu(display("rotation sensor error: {}", source))]
    RotationSensor { source: vexide::devices::PortError },
}

impl Arm {
    pub fn new(motors: MotorGroup, mut rotation_sensor: RotationSensor) -> Result<Self, ArmError> {
        rotation_sensor
            .set_direction(Direction::Reverse)
            .context(RotationSensorSnafu)?;
        rotation_sensor
            .set_position(Position::from_degrees(ArmState::INITIAL_ARM_ANGLE))
            .context(RotationSensorSnafu)?;

        Ok(Self {
            motors: Arc::new(Mutex::new(motors)),
            rotation: Arc::new(Mutex::new(rotation_sensor)),
            state: Arc::new(Mutex::new(ArmState::default())),
        })
    }

    pub async fn reset_rotation(&self) -> Result<(), ArmError> {
        let mut rotation = self.rotation.lock().await;
        rotation
            .set_position(Position::from_degrees(ArmState::INITIAL_ARM_ANGLE))
            .context(RotationSensorSnafu)?;
        Ok(())
    }

    pub async fn next_state(&self) {
        let mut state = self.state.lock().await;
        *state = match *state {
            ArmState::Initial => ArmState::Intake,
            ArmState::Intake => ArmState::Delay {
                instant: Instant::now() + Duration::from_millis(200),
                current: Box::new(ArmState::Intake),
                target: Box::new(ArmState::MaxExpansion),
            },
            ArmState::MaxExpansion => ArmState::Initial,
            ArmState::Delay { .. } => ArmState::MaxExpansion,
            ArmState::Manual(_) => ArmState::Initial,
        };
    }

    pub async fn update(&self) -> Result<(), ArmError> {
        let mut state = self.state.lock().await;
        if matches!(*state, ArmState::Delay { instant, .. } if instant < Instant::now()) {
            *state = match &*state {
                ArmState::Delay { target, .. } => *target.clone(),
                _ => unreachable!(),
            };
        }
        Ok(())
    }

    pub async fn get_state(&self) -> ArmState {
        let state = self.state.lock().await;
        state.clone()
    }

    pub async fn set_state(&self, new_state: ArmState) {
        let mut state = self.state.lock().await;
        *state = new_state;
    }

    pub fn spawn_update_thread(&self) {
        let rotation = self.rotation.clone();
        let motors = self.motors.clone();
        let state = self.state.clone();

        spawn(async move {
            let mut pid = pid::Pid::new(ArmState::Initial.angle(), Motor::EXP_MAX_VOLTAGE);
            pid.p(0.2, f64::MAX);

            loop {
                let state = state.lock().await;
                let rotation = rotation.lock().await;
                let motors = motors.lock().await;

                pid.setpoint = state.angle();

                let current_angle = rotation
                    .position()
                    .context(RotationSensorSnafu)?
                    .as_degrees();

                let output = pid.next_control_output(current_angle);

                let mut motors = self.motors.lock().await;
                motors.set_voltage(output.output).context(MotorSnafu)?;

                sleep(super::SUBSYSTEM_UPDATE_PERIOD).await;
            }
        });
    }

    pub async fn temperature(&self) -> f64 {
        let motors = self.motors.lock().await;
        motors.temperature().unwrap_or(0.0)
    }
}
