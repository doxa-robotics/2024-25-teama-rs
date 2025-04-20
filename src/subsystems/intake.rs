use alloc::sync::Arc;
use core::{ops::Not, time::Duration};

use colorsys::{ColorAlpha, Hsl, Rgb};
use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    devices::smart::{motor::MotorError, vision::DetectionSource},
    float::Float as _,
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
const JAM_REVERSE_TIME: Duration = Duration::from_millis(500);
const RING_REJECT_STOP_TIME: Duration = Duration::from_millis(000);
const RING_REJECT_RESTART_TIME: Duration = Duration::from_millis(600);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
        current_ring: Option<RingColor>,
        reject_time: Option<Instant>,
        jam_time: Option<Instant>,
        overcurrent_time: Option<Instant>,
    },
    Reverse,
    Stop,
    StopHold,
}

#[derive(Debug, Snafu)]
pub enum IntakeError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
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
            .set_brightness(0.5)
            .expect("failed to set vision brightness");
        vision
            .set_signature(
                RingColor::Red.signature(),
                VisionSignature {
                    range: 2.5,
                    u_threshold: (10027, 10981, 10504),
                    v_threshold: (-1707, -1307, -1507),
                    flags: 0,
                },
            )
            .expect("failed to initialize vision red signature");
        vision
            .set_signature(
                RingColor::Blue.signature(),
                VisionSignature {
                    range: 2.5,
                    u_threshold: (-4539, -3981, -4260),
                    v_threshold: (6895, 8859, 7877),
                    flags: 0,
                },
            )
            .expect("failed to initialize vision blue signature");

        let state = Arc::new(Mutex::new(IntakeState::Stop));
        Self {
            state: state.clone(),
            _task: Arc::new(spawn({
                async move {
                    let start = Instant::now();
                    loop {
                        let mut state = state.lock().await;
                        if let Err(err) =
                            Intake::update(&mut motor, &mut vision, &mut distance, &mut state).await
                        {
                            error!("intake update error: {}", err);
                        }
                        drop(state);
                        // Fade the LED!
                        let color = Rgb::from(Hsl::new(
                            (start.elapsed().as_millis() as f64 / 20.0).rem_euclid(360.0),
                            100.0,
                            50.0,
                            Some(
                                ((start.elapsed().as_millis() as f64 / 200.0).sin() + 1.0) / 2.0
                                    * 0.8
                                    + 0.2,
                            ),
                        ));
                        _ = vision.set_led_mode(vexide::prelude::LedMode::Manual(
                            (color.red() as u8, color.green() as u8, color.blue() as u8).into(),
                            color.alpha(),
                        ));

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
                ref mut current_ring,
            } => {
                if let Ok(current) = motor.current() {
                    if current > JAM_CURRENT {
                        if let Some(overcurrent_time) = overcurrent_time {
                            if overcurrent_time.elapsed() > JAM_OVERCURRENT_TIME {
                                log::warn!("intake jammed, reversing");
                                *jam_time = Some(Instant::now());
                            }
                        } else {
                            *overcurrent_time = Some(Instant::now());
                        }
                    } else {
                        *overcurrent_time = None;
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
                if let Ok(objects) = vision.objects() {
                    for object in objects {
                        if let DetectionSource::Signature(id) = object.source {
                            match id {
                                RED_SIGNATURE => {
                                    *current_ring = Some(RingColor::Red);
                                }
                                BLUE_SIGNATURE => {
                                    *current_ring = Some(RingColor::Blue);
                                }
                                _ => {}
                            }
                        }
                    }
                } else {
                    log::warn!("failed to get vision detections");
                }
                if let Some(accept_color) = accept {
                    if reject_time.is_none()
                        && let Ok(Some(object)) = distance.object()
                        && object.distance < 50
                        && let Some(current_ring) = current_ring
                        && *current_ring != *accept_color
                    {
                        log::debug!("intake rejected ring: {:?}", current_ring);
                        *reject_time = Some(Instant::now());
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
                current_ring: None,
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
            current_ring: None,
        };
    }
}
