use snafu::ResultExt;

pub struct Clamp {
    adi_out: vexide::devices::adi::AdiDigitalOut,
}

#[derive(Debug, snafu::Snafu)]
pub enum ClampError {
    #[snafu(display("digital output port error: {}", source))]
    DigitalPort { source: vexide::devices::PortError },
}

impl Clamp {
    pub fn new(adi_out: vexide::devices::adi::AdiDigitalOut) -> Self {
        Self { adi_out }
    }

    pub fn toggle(&mut self) -> Result<(), ClampError> {
        match self.adi_out.level().context(DigitalPortSnafu)? {
            vexide::devices::adi::digital::LogicLevel::High => self.clamp(),
            vexide::devices::adi::digital::LogicLevel::Low => self.unclamp(),
        }
    }

    pub fn unclamp(&mut self) -> Result<(), ClampError> {
        self.adi_out
            .set_level(vexide::devices::adi::digital::LogicLevel::High)
            .context(DigitalPortSnafu)?;
        Ok(())
    }

    pub fn clamp(&mut self) -> Result<(), ClampError> {
        self.adi_out
            .set_level(vexide::devices::adi::digital::LogicLevel::Low)
            .context(DigitalPortSnafu)?;
        Ok(())
    }
}
