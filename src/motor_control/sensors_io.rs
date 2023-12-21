use embassy_stm32::spi;

// pub type Result<T> = core::result::Result<T, IOError>;
use super::motors_io::Result;


#[derive(Debug)]
pub enum IOError {
    SpiError(spi::Error),
}

pub trait RawSensorsIO<const N: usize> {
    /// Get sensors value
    fn get_axis_sensors(&mut self) -> Result<[f32; N]>;
}
