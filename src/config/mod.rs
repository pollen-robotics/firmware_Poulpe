use crate::motor_control::Ventouse;
use crate::motor_control::AksimSensor;
use crate::motor_control::AD5047Sensor;

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_stm32::peripherals as p;
use embassy_stm32::usart::Uart;

#[cfg(feature = "orbita2d")]
pub const N_AXIS: usize = 2;
#[cfg(feature = "orbita3d")]
pub const N_AXIS: usize = 3;
#[cfg(feature = "bob")]
pub const N_AXIS: usize = 1;

pub static DXL_ID: u8 = 42;

pub type DynamixelUart = Uart<'static, p::USART1, p::DMA1_CH0, p::DMA1_CH1>;

pub type VentouseA = Ventouse<'static, p::SPI1, p::PA3, p::PA2, p::PC0, p::PA0, p::PA1>;
pub type VentouseB = Ventouse<'static, p::SPI4, p::PE3, p::PC15, p::PE0, p::PC13, p::PC14>;
pub type VentouseC = Ventouse<'static, p::SPI6, p::PD7, p::PD6, p::PD5, p::PD4, p::PD3>;

// pub type RingSensor = AksimSensor<'static, p::SPI4, p::PE4>;
pub type RingSensor = AksimSensor<SpiDevice<'static,NoopRawMutex, p::SPI4, p::PE4>>;


// pub type CenterSensor = AD5047Sensor<'static, p::SPI6, p::PB3>;


mod motor;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
pub use motor::BrushlessMotor;
