use defmt::info;
use embassy_stm32::spi::Error;

use super::{ventouse::Ventouse, MotionMode};

pub struct Actuator {
    ventouse: Ventouse,
}

impl Actuator {
    pub fn new(ventouse: Ventouse) -> Self {
        Self { ventouse }
    }

    pub async fn init(&mut self) {
        self.ventouse.tmc4671_init_registers().await.unwrap();
        info!("TMC4671 init done");
        self.ventouse.tmc4671_align_motor().await.unwrap();
        info!("Motor align done");
    }

    pub fn set_mode(&mut self, mode: MotionMode) -> Result<u32, Error> {
        self.ventouse.tmc4671_set_mode(mode)
    }

    pub fn get_actual_position(&mut self) -> Result<i32, Error> {
        self.ventouse.tmc4671_get_actual_position()
    }
    pub fn set_target_position(&mut self, position: i32) -> Result<u32, Error> {
        self.ventouse.tmc4671_set_target_position(position)
    }
}
