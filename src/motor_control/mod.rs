mod actuator;
pub mod sensors;
pub use actuator::Actuator;
mod axis_sensor;
mod driver;
pub use driver::Driver;
pub(crate) mod foc;
pub use foc::Foc;
mod motors_io;
pub use motors_io::{Pid, RawMotorsIO, Result};
mod sensors_io;
pub use sensors_io::RawSensorsIO;

pub mod task;
pub mod ventouse;
pub mod analog;

#[derive(PartialEq)] 
#[derive(Clone, Copy,defmt::Format)]
pub enum BoardStatus{
    Ok = 0,
    InitError = 1,
    SensorError = 2,
    IndexError = 3,
    ZeroingError = 4,
    OverTemperatureError = 5,
    OverCurrentError = 6,
    BusVoltageError = 7,
}