use alloc::boxed::Box;
use core::{f64, time::Duration};

use snafu::{ResultExt, Snafu};
use vexide::{
    core::time::Instant,
    devices::smart::motor::MotorError,
    prelude::{Direction, Motor, Position, RotationSensor},
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
    motors: MotorGroup,
    rotation: RotationSensor,
    pid: pid::Pid<f64>,
    pub state: ArmState,
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

        let mut pid = pid::Pid::new(ArmState::Initial.angle(), Motor::EXP_MAX_VOLTAGE);
        pid.p(0.2, f64::MAX);

        Ok(Self {
            motors,
            pid,
            rotation: rotation_sensor,
            state: ArmState::default(),
        })
    }

    pub fn reset_rotation(&mut self) -> Result<(), ArmError> {
        self.rotation
            .set_position(Position::from_degrees(ArmState::INITIAL_ARM_ANGLE))
            .context(RotationSensorSnafu)?;
        Ok(())
    }

    pub fn next_state(&mut self) {
        self.state = match self.state {
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

    pub fn update(&mut self) -> Result<(), ArmError> {
        if matches!(self.state, ArmState::Delay { instant, .. } if instant < Instant::now()) {
            self.state = match &self.state {
                ArmState::Delay { target, .. } => *target.clone(),
                _ => unreachable!(),
            };
        }
        self.pid.setpoint = self.state.angle();
        let current_angle = self
            .rotation
            .position()
            .context(RotationSensorSnafu)?
            .as_degrees();
        let output = self.pid.next_control_output(current_angle);
        self.motors.set_voltage(output.output).context(MotorSnafu)?;
        Ok(())
    }

    pub fn temperature(&self) -> f64 {
        self.motors.temperature().unwrap_or(0.0)
    }
}
