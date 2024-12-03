use defmt::Format;
use embassy_stm32::i2c;
use embassy_stm32::spi;

use crate::utils::errors::Result;

pub trait RawSensorsIO<const N: usize> {
    /// Get sensors value
    fn get_axis_sensors(&mut self) -> Result<[f32; N]>;
    // fn get_index_sensors(&mut self) -> Result<[u16; N]>;
}
