// use crate::motor_control::Ventouse;

use embassy_stm32::dma::NoDma;
use embassy_stm32::peripherals as p;
use embassy_stm32::usart::Uart;

#[cfg(feature = "orbita2d")]
pub const N_AXIS: usize = 2;
#[cfg(feature = "orbita3d")]
pub const N_AXIS: usize = 3;

// maximal temperature limits for the motor and the boards
// high temeperature state - only warning
pub const HIGH_TEMP: f32 = 65.0;
// maximal temperature limit - error state
pub const MAX_TEMP: f32 = 75.0;

// pub static DXL_ID: u8 = 42;


pub type DynamixelUart = Uart<'static, p::USART1, p::DMA1_CH0, p::DMA1_CH1>;

use crate::motor_control::{
    sensors::{AD5047Sensor, AksimSensor, I2cHallSensor},
    sensors::{I2cHallConfig, SensorConfig},
    ventouse::{Ventouse, VentouseConfig},
    analog::AnalogInputConfig,
};

use crate::motor_control::driver::{DriverDRV8316, DriverTMC6200};

// Ventouse A
#[cfg(any(feature = "beta", all(feature="orbita2d", feature="gamma")))] // any beta or 2d gamma
pub type VentouseA<'d> = Ventouse<'d, p::SPI1, p::PA3, p::PC0, DriverTMC6200<'d, p::SPI1, p::PA2>>;
#[cfg(any(all(feature="gamma", feature="orbita3d")))] // 3d gamma
pub type VentouseA<'d> = Ventouse<'d, p::SPI1, p::PA3, p::PC0, DriverDRV8316<'d, p::SPI1, p::PA2>>;

// Ventouse B
#[cfg(any(feature = "beta", all(feature="orbita2d", feature="gamma")))] // any beta or 2d gamma
pub type VentouseB<'d> = Ventouse<'d, p::SPI4, p::PE3, p::PE0, DriverTMC6200<'d, p::SPI4, p::PC15>>;
#[cfg(any(all(feature="gamma", feature="orbita3d")))] // 3d gamma
pub type VentouseB<'d> = Ventouse<'d, p::SPI4, p::PE3, p::PE0, DriverDRV8316<'d, p::SPI4, p::PC15>>;

// Ventouse C
#[cfg(any(feature = "beta", all(feature="orbita2d", feature="gamma")))] // any beta or 2d gamma
pub type VentouseC<'d> = Ventouse<'d, p::SPI6, p::PD7, p::PD5, DriverTMC6200<'d, p::SPI6, p::PD6>>;
#[cfg(any(all(feature="gamma", feature="orbita3d")))] // 3d gamma
pub type VentouseC<'d> = Ventouse<'d, p::SPI6, p::PD7, p::PD5,  DriverDRV8316<'d, p::SPI6, p::PD6>>;



#[cfg(feature = "orbita3d")]
pub type VentouseAConfig = VentouseConfig<p::SPI1, p::PA5, p::PA7, p::PA6, p::PA3, p::PC0, p::PA2>;
pub type VentouseBConfig =
    VentouseConfig<p::SPI4, p::PE12, p::PE6, p::PE5, p::PE3, p::PE0, p::PC15>;
pub type VentouseCConfig = VentouseConfig<p::SPI6, p::PB3, p::PB5, p::PB4, p::PD7, p::PD5, p::PD6>;

pub type AksimConfig = SensorConfig<p::PA15>;
pub type AD5047Config = SensorConfig<p::PE4>;

pub type AD5047ConfigTop = SensorConfig<p::PA4>;
pub type AD5047ConfigMid = SensorConfig<p::PE4>;
pub type AD5047ConfigBot = SensorConfig<p::PA15>;

pub type DonutHallConfig = I2cHallConfig<p::I2C1, p::PB6, p::PB7>;

pub type Aksim<'d> = AksimSensor<'d, p::SPI6, p::PA15>;
pub type AD5047<'d> = AD5047Sensor<'d, p::SPI4, p::PE4>;

pub type AD5047Top<'d> = AD5047Sensor<'d, p::SPI4, p::PA4>;
pub type AD5047Mid<'d> = AD5047Sensor<'d, p::SPI4, p::PE4>;
pub type AD5047Bot<'d> = AD5047Sensor<'d, p::SPI4, p::PA15>;


pub type TemperatureSensorConfig = AnalogInputConfig<p::ADC1, p::PB1>;


// pub type DonutHall<'d> = I2cHallSensor<'d, p::I2C1, p::PB6, p::PB7>;
pub type DonutHall<'d> = I2cHallSensor<p::I2C1>;

// from build.rs (should contain DXL_ID, HARDWARE_ZEROS and GIT_HASH)
include!(concat!(env!("OUT_DIR"), "/constants.rs"));

pub struct ActuatorConfig {
    #[cfg(feature = "orbita3d")]
    pub a: VentouseAConfig,

    pub b: VentouseBConfig,
    pub c: VentouseCConfig,
    #[cfg(feature = "orbita2d")]
    pub aksim: AksimConfig,
    #[cfg(feature = "orbita2d")]
    pub ad5047: AD5047Config,

    #[cfg(feature = "orbita3d")]
    pub ad5047top: AD5047ConfigTop,
    #[cfg(feature = "orbita3d")]
    pub ad5047mid: AD5047ConfigMid,
    #[cfg(feature = "orbita3d")]
    pub ad5047bot: AD5047ConfigBot,
    #[cfg(feature = "orbita3d")]
    pub donut_hall: DonutHallConfig,
    #[cfg(not(feature = "no_temperture_sensor"))]
    pub temperature_sensor: TemperatureSensorConfig
}

mod motor;
pub use motor::BrushlessMotor;
mod current_sense;
pub use current_sense::CurrentSensing;
pub mod flash;