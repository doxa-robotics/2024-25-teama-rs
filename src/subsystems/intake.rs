use alloc::sync::Arc;
use core::{ops::Not, time::Duration};

use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    devices::{
        smart::{motor::MotorError, vision::DetectionSource},
        PortError,
    },
    prelude::{
        sleep, spawn, BrakeMode, Direction, Motor, VisionMode, VisionSensor, VisionSignature,
    },
    sync::Mutex,
    time::Instant,
};

const RED_SIGNATURE: u8 = 1;
const BLUE_SIGNATURE: u8 = 2;

#[derive(Debug, Clone, Copy)]
#[allow(unused)]
pub enum RingColor {
    Red,
    Blue,
}

impl RingColor {
    fn signature(&self) -> u8 {
        match self {
            RingColor::Red => RED_SIGNATURE,
            RingColor::Blue => BLUE_SIGNATURE,
        }
    }
}

impl Not for RingColor {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            RingColor::Red => RingColor::Blue,
            RingColor::Blue => RingColor::Red,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum IntakeState {
    Forward {
        accept: Option<RingColor>,
        reject_time: Option<Instant>,
    },
    Reverse,
    Stop,
    StopHold,
}

const TIMEOUT: u128 = 3000;
const RING_REJECT_STOP_TIME: Duration = Duration::from_millis(300);
const RING_REJECT_RESTART_TIME: Duration = Duration::from_millis(1000);

#[derive(Debug)]
pub struct IntakeInner {
    motor: Motor,
    vision: VisionSensor,
    state: IntakeState,
}

#[derive(Debug, Snafu)]
pub enum IntakeError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
    #[snafu(display("port error: {}", source))]
    PortError { source: PortError },
    #[snafu(display("vision error: {}", source))]
    Vision {
        source: vexide::devices::smart::vision::VisionError,
    },
}

impl IntakeInner {
    async fn update(&mut self) -> Result<(), IntakeError> {
        match self.state {
            IntakeState::Forward {
                accept,
                ref mut reject_time,
            } => {
                if let Some(reject_time) = reject_time
                    && (RING_REJECT_STOP_TIME..RING_REJECT_RESTART_TIME)
                        .contains(&reject_time.elapsed())
                {
                    self.motor.brake(BrakeMode::Brake).context(MotorSnafu)?;
                } else {
                    self.motor
                        .set_voltage(self.motor.max_voltage())
                        .context(MotorSnafu)?;
                }
                if let Some(accept_color) = accept
                    && reject_time.is_none()
                {
                    match self.vision.objects() {
                        Ok(objects) => {
                            for object in objects {
                                if let DetectionSource::Signature(sig) = object.source {
                                    log::debug!("ring: {:?}", object.source);
                                    if sig == (!accept_color).signature() {
                                        log::debug!("intake rejected ring: {:?}", object.source);
                                        *reject_time = Some(Instant::now());
                                    }
                                }
                            }
                        }
                        Err(err) => {
                            log::warn!("vision error: {}", err);
                        }
                    }
                }
            }
            IntakeState::Reverse => {
                self.motor
                    .set_voltage(-self.motor.max_voltage())
                    .context(MotorSnafu)?;
            }
            IntakeState::Stop => {
                self.motor.brake(BrakeMode::Coast).context(MotorSnafu)?;
            }
            IntakeState::StopHold => {
                self.motor.brake(BrakeMode::Hold).context(MotorSnafu)?;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct Intake(Arc<Mutex<IntakeInner>>);

impl Intake {
    pub fn new(motor: Motor, mut vision: VisionSensor) -> Self {
        vision
            .set_mode(VisionMode::ColorDetection)
            .expect("failed to set vision mode");
        vision
            .set_signature(
                RED_SIGNATURE,
                VisionSignature {
                    range: 5.0,
                    u_threshold: (10581, 11817, 11199),
                    v_threshold: (-1377, -1001, -1189),
                    flags: 0,
                },
            )
            .expect("failed to initialize vision red signature");
        vision
            .set_signature(
                BLUE_SIGNATURE,
                VisionSignature {
                    range: 5.0,
                    u_threshold: (-4097, -3559, -3828),
                    v_threshold: (5605, 7797, 6701),
                    flags: 0,
                },
            )
            .expect("failed to initialize vision blue signature");
        Self(Arc::new(Mutex::new(IntakeInner {
            motor,
            vision,
            state: IntakeState::Stop,
        })))
    }

    pub fn temperature(&self) -> f64 {
        match self.0.try_lock() {
            Some(inner) => inner.motor.temperature().unwrap_or(0.0),
            None => 0.0,
        }
    }

    pub fn task(&self) {
        let inner = self.0.clone();
        spawn(async move {
            loop {
                let mut inner = inner.lock().await;
                if let Err(err) = inner.update().await {
                    error!("intake update error: {}", err);
                }
                drop(inner);

                sleep(super::SUBSYSTEM_UPDATE_PERIOD).await;
            }
        })
        .detach();
    }

    pub async fn stop(&self) {
        let mut inner = self.0.lock().await;
        inner.state = IntakeState::Stop;
    }

    pub async fn stop_hold(&self) {
        let mut inner = self.0.lock().await;
        inner.state = IntakeState::StopHold;
    }

    pub async fn run(&self, direction: Direction) {
        let mut inner = self.0.lock().await;
        inner.state = match direction {
            Direction::Forward => IntakeState::Forward {
                accept: None,
                reject_time: None,
            },
            Direction::Reverse => IntakeState::Reverse,
        };
    }

    pub async fn run_forward_accept(&self, color: RingColor) {
        let mut inner = self.0.lock().await;
        inner.state = IntakeState::Forward {
            accept: Some(color),
            reject_time: None,
        };
    }
}
