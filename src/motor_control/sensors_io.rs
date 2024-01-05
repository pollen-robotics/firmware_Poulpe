
use defmt::Format;
use embassy_stm32::spi;
use embassy_stm32::i2c;

// pub type Result<T> = core::result::Result<T, IOError>;
use super::motors_io::Result;


pub trait RawSensorsIO<const N: usize> {
    /// Get sensors value
    fn get_axis_sensors(&mut self) -> Result<[f32; N]>;
    // fn get_index_sensors(&mut self) -> Result<[u16; N]>;

}
