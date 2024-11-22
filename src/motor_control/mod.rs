mod actuator;
pub use actuator::Actuator;
pub mod driver;
pub use driver::{DriverDRV8316, DriverTMC6200};
pub(crate) mod foc;
pub use foc::Foc;
mod motors_io;
pub use motors_io::{Pid, RawMotorsIO};

pub mod task;
pub mod ventouse;
