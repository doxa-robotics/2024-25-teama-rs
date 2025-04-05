#[cfg(not(feature = "simplified_controls"))]
mod normal;
#[cfg(feature = "simplified_controls")]
mod simple;

#[cfg(not(feature = "simplified_controls"))]
pub use normal::opcontrol;
#[cfg(feature = "simplified_controls")]
pub use simple::opcontrol;
