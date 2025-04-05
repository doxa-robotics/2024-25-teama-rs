mod normal;
mod simple;

#[cfg(not(feature = "simplified_controls"))]
pub use normal::opcontrol;
#[cfg(feature = "simplified_controls")]
pub use simple::opcontrol;
