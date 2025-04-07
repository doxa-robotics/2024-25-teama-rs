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
        sleep, spawn, BrakeMode, Direction, DistanceSensor, Motor, VisionMode, VisionSensor,
        VisionSignature,
    },
    sync::Mutex,
    time::Instant,
};

const RED_SIGNATURE: u8 = 1;
const BLUE_SIGNATURE: u8 = 2;
const JAM_CURRENT: f64 = 2.6;
const JAM_OVERCURRENT_TIME: Duration = Duration::from_millis(1000);
const JAM_REVERSE_TIME: Duration = Duration::from_millis(200);

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
        jam_time: Option<Instant>,
        overcurrent_time: Option<Instant>,
    },
    Reverse,
    Stop,
    StopHold,
}

const TIMEOUT: u128 = 3000;
const RING_REJECT_STOP_TIME: Duration = Duration::from_millis(100);
const RING_REJECT_RESTART_TIME: Duration = Duration::from_millis(1000);

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

#[derive(Debug, Clone)]
pub struct Intake {
    state: Arc<Mutex<IntakeState>>,
    _task: Arc<vexide::task::Task<()>>,
}

impl Intake {
    pub fn new(mut motor: Motor, mut vision: VisionSensor, mut distance: DistanceSensor) -> Self {
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

        let state = Arc::new(Mutex::new(IntakeState::Stop));
        Self {
            state: state.clone(),
            _task: Arc::new(spawn({
                async move {
                    loop {
                        let mut state = state.lock().await;
                        if let Err(err) =
                            Intake::update(&mut motor, &mut vision, &mut distance, &mut state).await
                        {
                            error!("intake update error: {}", err);
                        }
                        drop(state);

                        sleep(super::SUBSYSTEM_UPDATE_PERIOD).await;
                    }
                }
            })),
        }
    }

    async fn update(
        motor: &mut Motor,
        vision: &mut VisionSensor,
        distance: &mut DistanceSensor,
        state: &mut IntakeState,
    ) -> Result<(), IntakeError> {
        match state {
            IntakeState::Forward {
                accept,
                ref mut reject_time,
                ref mut jam_time,
                ref mut overcurrent_time,
            } => {
                if let Ok(current) = motor.current() {
                    log::debug!("intake current: {}", current);
                    if current > JAM_CURRENT {
                        if let Some(overcurrent_time) = overcurrent_time {
                            if overcurrent_time.elapsed() > JAM_OVERCURRENT_TIME {
                                log::warn!("intake jammed, reversing");
                                *jam_time = Some(Instant::now());
                            }
                        } else {
                            *jam_time = Some(Instant::now());
                        }
                    }
                } else {
                    log::warn!("failed to get motor current");
                }
                if let Some(jammed_time) = jam_time {
                    if jammed_time.elapsed() > JAM_REVERSE_TIME {
                        *jam_time = None;
                        *overcurrent_time = None;
                    }
                }
                if let Some(reject_time) = reject_time
                    && (RING_REJECT_STOP_TIME..RING_REJECT_RESTART_TIME)
                        .contains(&reject_time.elapsed())
                {
                    motor.brake(BrakeMode::Brake).context(MotorSnafu)?;
                } else if jam_time.is_some() {
                    motor
                        .set_voltage(-motor.max_voltage())
                        .context(MotorSnafu)?;
                } else {
                    motor.set_voltage(motor.max_voltage()).context(MotorSnafu)?;
                }
                if let Some(accept_color) = accept
                    && reject_time.is_none()
                {
                    match vision.objects() {
                        Ok(objects) => {
                            for object in objects {
                                if let DetectionSource::Signature(sig) = object.source {
                                    log::debug!("ring: {:?}", object.source);
                                    if sig == (!*accept_color).signature() {
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
                motor
                    .set_voltage(-motor.max_voltage())
                    .context(MotorSnafu)?;
            }
            IntakeState::Stop => {
                motor.brake(BrakeMode::Coast).context(MotorSnafu)?;
            }
            IntakeState::StopHold => {
                motor.brake(BrakeMode::Hold).context(MotorSnafu)?;
            }
        }
        Ok(())
    }

    pub async fn stop(&self) {
        let mut state = self.state.lock().await;
        *state = IntakeState::Stop;
    }

    pub async fn stop_hold(&self) {
        let mut state = self.state.lock().await;
        *state = IntakeState::StopHold;
    }

    pub async fn run(&self, direction: Direction) {
        let mut state = self.state.lock().await;
        *state = match direction {
            Direction::Forward => IntakeState::Forward {
                accept: None,
                reject_time: None,
                jam_time: None,
                overcurrent_time: None,
            },
            Direction::Reverse => IntakeState::Reverse,
        };
    }

    pub async fn run_forward_accept(&self, color: RingColor) {
        let mut state = self.state.lock().await;
        *state = IntakeState::Forward {
            accept: Some(color),
            reject_time: None,
            jam_time: None,
            overcurrent_time: None,
        };
    }
}
