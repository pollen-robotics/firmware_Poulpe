// use crate::motor_control::Ventouse;

use embassy_stm32::peripherals as p;
use embassy_stm32::usart::Uart;

#[cfg(feature = "orbita2d")]
pub const N_AXIS: usize = 2;
#[cfg(feature = "orbita3d")]
pub const N_AXIS: usize = 3;

pub static DXL_ID: u8 = 42;

pub type DynamixelUart = Uart<'static, p::USART1, p::DMA1_CH0, p::DMA1_CH1>;

use crate::motor_control::ventouse::{Ventouse, VentouseConfig};

pub type VentouseA<'d> = Ventouse<'d, p::SPI1, p::PA3, p::PC0, p::PA2>;
pub type VentouseB<'d> = Ventouse<'d, p::SPI4, p::PE3, p::PE0, p::PC15>;
pub type VentouseC<'d> = Ventouse<'d, p::SPI6, p::PD7, p::PD5, p::PD6>;

pub type VentouseAConfig = VentouseConfig<p::SPI1, p::PA5, p::PA7, p::PA6, p::PA3, p::PC0, p::PA2>;
pub type VentouseBConfig =
    VentouseConfig<p::SPI4, p::PE12, p::PE6, p::PE5, p::PE3, p::PE0, p::PC15>;
pub type VentouseCConfig = VentouseConfig<p::SPI6, p::PB3, p::PB5, p::PB4, p::PD7, p::PD5, p::PD6>;

pub struct ActuatorConfig {
    #[cfg(feature = "orbita3d")]
    pub a: VentouseAConfig,
    pub b: VentouseBConfig,
    pub c: VentouseCConfig,
}

mod motor;
pub use motor::BrushlessMotor;
