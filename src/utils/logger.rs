use alloc::{boxed::Box, sync::Arc};
use core::time::Duration;

use log::{error, Level, Metadata, Record, SetLoggerError};
use vexide::{
    core::{println, sync::Mutex},
    prelude::{spawn, Write},
};

struct SimpleLogger {
    file: Arc<Mutex<vexide::core::fs::File>>,
}
// Safety: The brain is single-threaded. Hopefully.
unsafe impl core::marker::Sync for SimpleLogger {}
unsafe impl core::marker::Send for SimpleLogger {}

impl SimpleLogger {
    pub fn new() -> Self {
        Self {
            // We're being very unsafe here.
            #[allow(clippy::arc_with_non_send_sync)]
            file: Arc::new(Mutex::new(
                vexide::core::fs::File::options()
                    .write(true)
                    .append(true)
                    .open("log.txt")
                    .expect("Failed to create log file"),
            )),
        }
    }
}

impl log::Log for SimpleLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Debug
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            println!("{} - {}", record.level(), record.args());
            if let Some(mut file) = self.file.try_lock() {
                _ = writeln!(file, "{} - {}", record.level(), record.args());
            } else {
                spawn(async {
                    vexide::async_runtime::time::sleep(Duration::from_millis(100)).await;
                    error!("Failed to lock log file (slept 100ms). 1 message was ignored.");
                })
                .detach();
            }
        }
    }

    fn flush(&self) {
        vexide::core::io::stdout().flush().ok();
    }
}

pub fn init() -> Result<(), SetLoggerError> {
    let logger = SimpleLogger::new();
    let logger_ref = Box::leak(Box::new(logger));
    log::set_logger(logger_ref).map(|()| log::set_max_level(log::LevelFilter::Debug))
}
