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

pub mod analog;
pub mod task;
pub mod ventouse;

#[derive(PartialEq, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum BoardStatus {
    Ok = 0,
    InitError = 1,
    SensorError = 2,
    IndexError = 3,
    ZeroingError = 4,
    OverTemperatureError = 5,
    OverCurrentError = 6,
    BusVoltageError = 7,
    HighTemperatureState = 100,
    Init = 20,
    Unknown = 255,
}

impl BoardStatus {
    pub fn from_u8(value: u8) -> BoardStatus {
        match value {
            0 => BoardStatus::Ok,
            1 => BoardStatus::InitError,
            2 => BoardStatus::SensorError,
            3 => BoardStatus::IndexError,
            4 => BoardStatus::ZeroingError,
            5 => BoardStatus::OverTemperatureError,
            6 => BoardStatus::OverCurrentError,
            7 => BoardStatus::BusVoltageError,
            100 => BoardStatus::HighTemperatureState,
            20 => BoardStatus::Init,
            255 => BoardStatus::Unknown,
            _ => BoardStatus::Unknown,
        }
    }
}
