use alloc::{boxed::Box, sync::Arc};
use core::time::Duration;

use log::{error, Level, Metadata, Record, SetLoggerError};
use vexide::{
    io::println,
    prelude::{spawn, Write},
    sync::Mutex,
};

struct SimpleLogger {
    file: Arc<Mutex<Option<vexide::fs::File>>>,
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
                vexide::fs::File::options()
                    .write(true)
                    .append(true)
                    .open("log.txt")
                    .ok(),
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
            // Only write to stdout if we're not connected to the competition field control
            // If we're connected to the field control, writing to stdout doesn't go
            // anywhere and is a waste of time.
            if !matches!(
                vexide::competition::system(),
                Some(vexide::competition::CompetitionSystem::FieldControl)
            ) {
                println!("{} - {}", record.level(), record.args());
            }
            if let Some(mut guard) = self.file.try_lock() {
                if let Some(file) = guard.as_mut() {
                    _ = writeln!(file, "{} - {}", record.level(), record.args());
                }
            } else {
                spawn(async {
                    vexide::time::sleep(Duration::from_millis(100)).await;
                    error!("Failed to lock log file (slept 100ms). 1 message was ignored.");
                })
                .detach();
            }
            self.flush();
        }
    }

    fn flush(&self) {
        let file = self.file.clone();
        spawn(async move {
            vexide::io::stdout().lock().await.flush().ok();
            if let Some(file) = file.lock().await.as_mut() {
                file.flush().ok();
            }
        })
        .detach();
    }
}

pub fn init() -> Result<(), SetLoggerError> {
    let logger = SimpleLogger::new();
    let logger_ref = Box::leak(Box::new(logger));
    log::set_logger(logger_ref).map(|()| log::set_max_level(log::LevelFilter::Debug))
}
