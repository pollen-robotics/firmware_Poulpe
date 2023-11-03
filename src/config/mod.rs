use crate::motor_control::Ventouse;

use embassy_stm32::dma::NoDma;
use embassy_stm32::peripherals as p;
use embassy_stm32::usart::Uart;

pub static DXL_ID: u8 = 42;

pub type DynamixelUart = Uart<'static, p::USART1, p::DMA1_CH0, p::DMA1_CH1>;

pub type VentouseA =
    Ventouse<'static, p::SPI4, NoDma, NoDma, p::PE3, p::PC15, p::PE0, p::PC13, p::PC14>;
pub type VentouseB =
    Ventouse<'static, p::SPI6, NoDma, NoDma, p::PD7, p::PD6, p::PD5, p::PD4, p::PD3>;

#[cfg(feature = "ecx22")]
pub mod motor {
    pub const PID_FLUX_P_FLUX_I: u32 = 0x03200080;
    pub const PID_TORQUE_P_TORQUE_I: u32 = 0x03200000;
    pub const PID_VELOCITY_P_VELOCITY_I: u32 = 0x01000080;
    pub const PID_POSITION_P_POSITION_I: u32 = 0x00400010;
}

#[cfg(feature = "ec60")]
pub mod motor {
    pub const PID_FLUX_P_FLUX_I: u32 = 0x03200000;
    pub const PID_TORQUE_P_TORQUE_I: u32 = 0x03200000;
    pub const PID_VELOCITY_P_VELOCITY_I: u32 = 0x01F401C2;
    pub const PID_POSITION_P_POSITION_I: u32 = 0x00500000;
}
