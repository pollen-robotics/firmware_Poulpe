mod actuator;
pub mod sensors;
pub use actuator::Actuator;
mod axis_sensor;
pub mod driver;
pub use driver::{DriverDRV8316, DriverTMC6200};
pub(crate) mod foc;
pub use foc::Foc;
mod motors_io;
pub use motors_io::{Pid, RawMotorsIO, Result};
mod sensors_io;
pub use sensors_io::RawSensorsIO;

pub mod analog;
pub mod task;
pub mod ventouse;
