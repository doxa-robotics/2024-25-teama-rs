use snafu::ResultExt;

pub struct Doinker {
    adi_out: vexide::devices::adi::AdiDigitalOut,
}

#[derive(Debug, snafu::Snafu)]
pub enum DoinkerError {
    #[snafu(display("digital output port error: {}", source))]
    DigitalPort { source: vexide::devices::PortError },
}

impl Doinker {
    pub fn new(adi_out: vexide::devices::adi::AdiDigitalOut) -> Self {
        Self { adi_out }
    }

    pub fn toggle(&mut self) -> Result<(), DoinkerError> {
        match self.adi_out.level().context(DigitalPortSnafu)? {
            vexide::devices::adi::digital::LogicLevel::Low => self.undoink(),
            vexide::devices::adi::digital::LogicLevel::High => self.doink(),
        }
    }

    /// Don't ask about the name of the function. That was GitHub Copilot's idea.
    pub fn doink(&mut self) -> Result<(), DoinkerError> {
        self.adi_out
            .set_level(vexide::devices::adi::digital::LogicLevel::Low)
            .context(DigitalPortSnafu)?;
        Ok(())
    }

    /// Don't ask about the name of the function. That was GitHub Copilot's idea.
    pub fn undoink(&mut self) -> Result<(), DoinkerError> {
        self.adi_out
            .set_level(vexide::devices::adi::digital::LogicLevel::High)
            .context(DigitalPortSnafu)?;
        Ok(())
    }
}
