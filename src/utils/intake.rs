use vexide::{
    devices::smart::motor::MotorError,
    prelude::{AdiAnalogIn, BrakeMode, Motor},
};

pub struct Intake {
    motor: Motor,
    line_tracker: AdiAnalogIn,
}

impl Intake {
    pub fn new(motor: Motor, line_tracker: AdiAnalogIn) -> Self {
        Self {
            motor,
            line_tracker,
        }
    }

    pub fn run(&mut self) -> Result<(), MotorError> {
        self.motor.set_velocity(450)
    }

    pub fn partial_intake(&mut self) -> Result<(), MotorError> {
        self.motor.set_velocity(450)
    }

    pub fn stop(&mut self) -> Result<(), MotorError> {
        self.motor.brake(BrakeMode::Brake)
    }
}
