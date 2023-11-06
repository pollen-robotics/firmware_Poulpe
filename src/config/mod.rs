use crate::motor_control::Ventouse;

use embassy_stm32::peripherals as p;
use embassy_stm32::usart::Uart;

pub static DXL_ID: u8 = 42;

pub type DynamixelUart = Uart<'static, p::USART1, p::DMA1_CH0, p::DMA1_CH1>;

pub type VentouseA = Ventouse<'static, p::SPI4, p::PE3, p::PC15, p::PE0, p::PC13, p::PC14>;
pub type VentouseB = Ventouse<'static, p::SPI6, p::PD7, p::PD6, p::PD5, p::PD4, p::PD3>;

mod motor;
pub use motor::BrushlessMotor;
