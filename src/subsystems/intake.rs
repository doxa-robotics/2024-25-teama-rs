use alloc::rc::Rc;
use core::{cell::RefCell, ops::Not, time::Duration};

use colorsys::{ColorAlpha, Hsl, Rgb};
use log::error;
use snafu::{ResultExt, Snafu};
use vexide::{
    devices::smart::{motor::MotorError, vision::DetectionSource},
    float::Float as _,
    prelude::{
        sleep, spawn, AdiLineTracker, BrakeMode, Direction, Motor, VisionMode, VisionSensor,
        VisionSignature,
    },
    time::Instant,
};

const JAM_CURRENT: f64 = 2.6;
const JAM_OVERCURRENT_TIME: Duration = Duration::from_millis(1000);
const JAM_REVERSE_TIME: Duration = Duration::from_millis(500);
const RING_REJECT_STOP_TIME: Duration = Duration::from_millis(100);
const RING_REJECT_RESTART_TIME: Duration = Duration::from_millis(300);
const LINE_TRACKER_THRESHOLD: f64 = 0.1;
const RPM: i32 = 600;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(unused)]
pub enum RingColor {
    Red,
    Blue,
}

impl RingColor {
    const fn signatures(&self) -> &[(u8, VisionSignature)] {
        match self {
            RingColor::Red => &[
                (
                    3,
                    VisionSignature {
                        range: 4.5,
                        u_threshold: (10785, 11829, 11307),
                        v_threshold: (-1813, -961, -1387),
                        flags: 0,
                    },
                ),
                (
                    2,
                    VisionSignature {
                        range: 1.1,
                        u_threshold: (4681, 10923, 7802),
                        v_threshold: (-3059, -1129, -2094),
                        flags: 0,
                    },
                ),
            ],
            RingColor::Blue => &[
                (
                    4,
                    VisionSignature {
                        range: 2.5,
                        u_threshold: (-4539, -3981, -4260),
                        v_threshold: (6895, 8859, 7877),
                        flags: 0,
                    },
                ),
                (
                    1,
                    VisionSignature {
                        range: 2.5,
                        u_threshold: (-3859, -2711, -3285),
                        v_threshold: (3375, 8813, 6094),
                        flags: 0,
                    },
                ),
            ],
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
        current_ring: Option<RingColor>,
        red_time: Option<Instant>,
        reject_time: Option<Instant>,
        jam_time: Option<Instant>,
        overcurrent_time: Option<Instant>,
    },
    PartialIntake,
    Reverse,
    Stop,
    StopHold,
}

#[derive(Debug, Clone, Copy)]
pub struct IntakeInner {
    accept: Option<RingColor>,
    state: IntakeState,
}

#[derive(Debug, Snafu)]
pub enum IntakeError {
    #[snafu(display("motor error: {}", source))]
    Motor { source: MotorError },
}

#[derive(Debug, Clone)]
pub struct Intake {
    state: Rc<RefCell<IntakeInner>>,
    _task: Rc<vexide::task::Task<()>>,
}

impl Intake {
    pub fn new(
        mut motor: Motor,
        mut vision: VisionSensor,
        mut line_tracker: AdiLineTracker,
    ) -> Self {
        vision
            .set_mode(VisionMode::MixedDetection)
            .expect("failed to set vision mode");
        vision
            .set_brightness(1.2)
            .expect("failed to set vision brightness");
        for (id, signature) in RingColor::Blue
            .signatures()
            .iter()
            .chain(RingColor::Red.signatures())
        {
            vision
                .set_signature(*id, *signature)
                .expect("failed to initialize vision signature");
        }
        let mut line_tracker_zero = line_tracker
            .reflectivity()
            .expect("couldn't zero the line tracker");

        let mut sleeper = Some(Instant::now());

        let state = Rc::new(RefCell::new(IntakeInner {
            accept: None,
            state: IntakeState::Stop,
        }));
        Self {
            state: state.clone(),
            _task: Rc::new(spawn({
                async move {
                    let start = Instant::now();
                    loop {
                        if let Some(time) = sleeper
                            && time.elapsed() > Duration::from_millis(100)
                        {
                            line_tracker_zero = line_tracker
                                .reflectivity()
                                .expect("couldn't zero the line tracker");
                            sleeper = None;
                        }
                        {
                            let mut state = state.borrow_mut();
                            if let Err(err) = Intake::update(
                                &mut motor,
                                &mut vision,
                                &mut line_tracker,
                                line_tracker_zero,
                                &mut state,
                            ) {
                                error!("intake update error: {}", err);
                            }
                        }
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

    fn update(
        motor: &mut Motor,
        vision: &mut VisionSensor,
        line_tracker: &mut AdiLineTracker,
        line_tracker_zero: f64,
        state: &mut IntakeInner,
    ) -> Result<(), IntakeError> {
        match &mut state.state {
            IntakeState::Forward {
                ref mut reject_time,
                ref mut jam_time,
                ref mut overcurrent_time,
                ref mut current_ring,
                ref mut red_time,
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
                    motor.set_velocity(-RPM).context(MotorSnafu)?;
                } else {
                    motor.set_velocity(RPM).context(MotorSnafu)?;
                }
                if let Some(time) = reject_time
                    && time.elapsed() > RING_REJECT_RESTART_TIME
                {
                    *reject_time = None;
                }
                match vision.objects() {
                    Ok(objects) => {
                        for object in objects {
                            if let DetectionSource::Signature(id) = object.source {
                                if object.width <= 10 {
                                    // log::debug!("intake vision detected small object");
                                    continue;
                                }
                                // log::debug!("intake vision detected ring: {:?}", id);
                                for (signature_id, ..) in RingColor::Red.signatures() {
                                    if id == *signature_id {
                                        *current_ring = Some(RingColor::Red);
                                        *red_time = Some(Instant::now());
                                    }
                                }
                                for (signature_id, ..) in RingColor::Blue.signatures() {
                                    if id == *signature_id {
                                        *current_ring = Some(RingColor::Blue);
                                    }
                                }
                            }
                        }
                    }
                    Err(err) => {
                        log::warn!("failed to get vision detections: {}", err);
                    }
                }
                if let Some(time) = red_time {
                    if time.elapsed() > Duration::from_millis(800) {
                        *current_ring = Some(RingColor::Blue);
                        *red_time = None;
                    }
                }
                if let Some(accept_color) = state.accept {
                    if reject_time.is_none()
                        && line_tracker.reflectivity().unwrap_or(0.0)
                            > line_tracker_zero + LINE_TRACKER_THRESHOLD
                        && let Some(current_ring) = current_ring
                        && *current_ring != accept_color
                    {
                        log::debug!("intake rejected ring: {:?}", current_ring);
                        *reject_time = Some(Instant::now());
                    }
                }
            }
            IntakeState::PartialIntake => {
                if line_tracker.reflectivity().unwrap_or(0.0)
                    < line_tracker_zero + LINE_TRACKER_THRESHOLD
                {
                    motor.set_velocity(RPM).context(MotorSnafu)?;
                } else {
                    state.state = IntakeState::Stop;
                    motor.brake(BrakeMode::Brake).context(MotorSnafu)?;
                }
            }
            IntakeState::Reverse => {
                motor.set_velocity(-RPM).context(MotorSnafu)?;
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

    pub fn stop(&self) {
        let mut state = self.state.borrow_mut();
        state.state = IntakeState::Stop;
    }

    pub fn stop_hold(&self) {
        let mut state = self.state.borrow_mut();
        state.state = IntakeState::StopHold;
    }

    pub fn run(&self, direction: Direction) {
        let mut state = self.state.borrow_mut();
        state.state = match direction {
            Direction::Forward => IntakeState::Forward {
                reject_time: None,
                jam_time: None,
                overcurrent_time: None,
                current_ring: None,
                red_time: None,
            },
            Direction::Reverse => IntakeState::Reverse,
        };
    }

    pub fn partial_intake(&self) {
        let mut state = self.state.borrow_mut();
        state.state = IntakeState::PartialIntake;
    }

    pub fn set_accept(&self, color: Option<RingColor>) {
        let mut state = self.state.borrow_mut();
        state.accept = color;
    }

    pub fn accept(&self) -> Option<RingColor> {
        self.state.borrow().accept
    }
}
