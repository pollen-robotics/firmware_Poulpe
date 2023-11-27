// use crate::motor_control::Ventouse;

use embassy_stm32::peripherals as p;
use embassy_stm32::usart::Uart;

#[cfg(feature = "orbita2d")]
pub const N_AXIS: usize = 2;
#[cfg(feature = "orbita3d")]
pub const N_AXIS: usize = 3;

pub static DXL_ID: u8 = 42;

pub type DynamixelUart = Uart<'static, p::USART1, p::DMA1_CH0, p::DMA1_CH1>;

use crate::motor_control::ventouse::Ventouse;

pub type VentouseA<'d> = Ventouse<'d, 'static, 'static, 'static, p::SPI1, p::PA3, p::PC0, p::PA2>;
pub type VentouseB<'d> = Ventouse<'d, 'static, 'static, 'static, p::SPI4, p::PE3, p::PE0, p::PC15>;
pub type VentouseC<'d> = Ventouse<'d, 'static, 'static, 'static, p::SPI6, p::PD7, p::PD5, p::PD6>;

mod motor;
pub use motor::BrushlessMotor;
