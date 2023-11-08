use embassy_stm32::spi::Error;

pub trait Axis {
    async fn init(&mut self);

    fn get_actual_position(&mut self) -> Result<i32, Error>;
    fn set_target_position(&mut self, position: i32) -> Result<u32, Error>;
}
