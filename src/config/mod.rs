// use crate::motor_control::Ventouse;

use embassy_stm32::dma::NoDma;
use embassy_stm32::peripherals as p;
use embassy_stm32::usart::Uart;

#[cfg(feature = "orbita2d")]
pub const N_AXIS: usize = 2;
#[cfg(feature = "orbita3d")]
pub const N_AXIS: usize = 3;

// maximal temperature limits for the motor and the boards
// high temeperature state (boards and motors) - only warning
pub const HIGH_TEMP: f32 = 65.0;
// maximal motor temperature - error state if above
pub const MAX_MOTOR_TEMP: f32 = 75.0;
// maximal board temperature - error state if above
pub const MAX_BOARD_TEMP: f32 = 100.0;
// minimal measureable temperature - error state if below (sensor malfunction)
pub const MIN_TEMP: f32 = -40.0;

// minimal bus voltage - error state if below
pub const MIN_BUS_VOLTAGE: f32 = 10.0;

// maximal time without the watchdog update - error state if above
pub const MAX_WATCHDOG_DOWN_TIME_MS: u64 = 100; // in milliseconds

// maximal timeout that is allowed for communication with the motor controller
// after this time, the communication is considered as failed and the operation is stopped
pub const MAX_COMMUNICATION_DOWN_TIME: u32 = 3; // in sec

// pub static DXL_ID: u8 = 42;

pub type DynamixelUart = Uart<'static, p::USART1, p::DMA1_CH0, p::DMA1_CH1>;

pub type LAN9252Config = EthercatConfig<p::SPI3, p::PC10, p::PB2, p::PC11, p::PD0>;

use crate::{
    ethercat::EthercatConfig,
    motor_control::ventouse::{Ventouse, VentouseConfig},
    sensors::sensors::{AD5047Sensor, AksimSensor, I2cHallConfig, I2cHallSensor, SensorConfig},
};

#[cfg(not(feature = "no_temperature_sensor"))]
use crate::sensors::analog::{
    AnalogInputConfig, Orbita2dTemperatureConfig, Orbita3dTemperatureConfig,
};

use crate::motor_control::driver::{DriverDRV8316, DriverTMC6200};

// Ventouse A
#[cfg(any(
    feature = "beta",
    all(feature = "orbita2d", any(feature = "gamma", feature = "pvt"))
))] // any beta or 2d gamma/pvt
pub type VentouseA<'d> =
    Ventouse<'d, p::SPI1, p::PA3, p::PC0, DriverTMC6200<'d, p::SPI1, p::PA2, p::PA1>>;
#[cfg(any(all(any(feature = "gamma", feature = "pvt"), feature = "orbita3d")))] // 3d gamma/pvt
pub type VentouseA<'d> =
    Ventouse<'d, p::SPI1, p::PA3, p::PC0, DriverDRV8316<'d, p::SPI1, p::PA2, p::PA1>>;

// Ventouse B
#[cfg(any(
    feature = "beta",
    all(feature = "orbita2d", any(feature = "gamma", feature = "pvt"))
))] // any beta or 2d gamma/pvt
pub type VentouseB<'d> =
    Ventouse<'d, p::SPI4, p::PE3, p::PE0, DriverTMC6200<'d, p::SPI4, p::PC15, p::PC14>>;
#[cfg(any(all(any(feature = "gamma", feature = "pvt"), feature = "orbita3d")))] // 3d gamma/pvt
pub type VentouseB<'d> =
    Ventouse<'d, p::SPI4, p::PE3, p::PE0, DriverDRV8316<'d, p::SPI4, p::PC15, p::PC14>>;

// Ventouse C
#[cfg(any(
    feature = "beta",
    all(feature = "orbita2d", any(feature = "gamma", feature = "pvt"))
))] // any beta or 2d gamma/pvt
pub type VentouseC<'d> =
    Ventouse<'d, p::SPI6, p::PD7, p::PD5, DriverTMC6200<'d, p::SPI6, p::PD6, p::PD3>>;
#[cfg(any(all(any(feature = "gamma", feature = "pvt"), feature = "orbita3d")))] // 3d gamma/pvt
pub type VentouseC<'d> =
    Ventouse<'d, p::SPI6, p::PD7, p::PD5, DriverDRV8316<'d, p::SPI6, p::PD6, p::PD3>>;

#[cfg(feature = "orbita3d")]
pub type VentouseAConfig =
    VentouseConfig<p::SPI1, p::PA5, p::PA7, p::PA6, p::PA3, p::PC0, p::PA2, p::PA1>;
pub type VentouseBConfig =
    VentouseConfig<p::SPI4, p::PE12, p::PE6, p::PE5, p::PE3, p::PE0, p::PC15, p::PC14>;
pub type VentouseCConfig =
    VentouseConfig<p::SPI6, p::PB3, p::PB5, p::PB4, p::PD7, p::PD5, p::PD6, p::PD3>;

pub type AksimConfig = SensorConfig<p::PA15>;
pub type AD5047Config = SensorConfig<p::PE4>;

pub type AD5047ConfigTop = SensorConfig<p::PA4>;
pub type AD5047ConfigMid = SensorConfig<p::PE4>;
pub type AD5047ConfigBot = SensorConfig<p::PA15>;

pub type LTC4332DonutConfig = SensorConfig<p::PA12>;
pub type LTC4332CenterConfig = SensorConfig<p::PB9>;
pub type LTC4332RingConfig = SensorConfig<p::PD1>;

pub type DonutHallConfig = I2cHallConfig<p::I2C1, p::PB6, p::PB7>;

pub type Aksim<'d> = AksimSensor<'d, p::SPI6, p::PA15>;
pub type AD5047<'d> = AD5047Sensor<'d, p::SPI4, p::PE4>;

pub type AD5047Top<'d> = AD5047Sensor<'d, p::SPI4, p::PA4>;
pub type AD5047Mid<'d> = AD5047Sensor<'d, p::SPI4, p::PE4>;
pub type AD5047Bot<'d> = AD5047Sensor<'d, p::SPI4, p::PA15>;

#[cfg(all(
    not(feature = "no_temperature_sensor"),
    feature = "orbita3d",
    feature = "pvt"
))]
pub type TemperatureSensingConfig = Orbita3dTemperatureConfig<p::ADC1, p::PB1, p::PC5, p::PB0>;
#[cfg(all(
    not(feature = "no_temperature_sensor"),
    feature = "orbita3d",
    not(feature = "pvt")
))]
pub type TemperatureSensingConfig = AnalogInputConfig<p::ADC1, p::PB1>;
#[cfg(all(
    not(feature = "no_temperature_sensor"),
    feature = "orbita2d",
    feature = "pvt"
))]
pub type TemperatureSensingConfig = Orbita2dTemperatureConfig<p::ADC1, p::PC5, p::PB0>;
#[cfg(all(
    not(feature = "no_temperature_sensor"),
    feature = "orbita2d",
    not(feature = "pvt")
))]
pub type TemperatureSensingConfig = AnalogInputConfig<p::ADC1, p::PB1>;

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
    #[cfg(not(feature = "no_temperature_sensor"))]
    pub temperature_sensing: TemperatureSensingConfig,
    #[cfg(all(feature = "pvt", feature = "orbita3d"))]
    pub ltc4332donut: LTC4332DonutConfig,
    #[cfg(all(feature = "pvt", feature = "orbita2d"))]
    pub ltc4332center: LTC4332CenterConfig,
    #[cfg(all(feature = "pvt", feature = "orbita2d"))]
    pub ltc4332ring: LTC4332RingConfig,
}

#[cfg(feature = "orbita3d_zero_pre_dvt")]
pub const ORBITA3D_HALL_ZERO_IDX: [u8; 3] = [0, 5, 10]; // This is the expected configuration with a "standard" zero as it was done until Reachy2 DVT
#[cfg(not(feature = "orbita3d_zero_pre_dvt"))]
pub const ORBITA3D_HALL_ZERO_IDX: [u8; 3] = [8, 13, 2]; // This is the expected configuration with a zero as it is done from Reachy2 DVT

mod motor;
pub use motor::BrushlessMotor;
mod current_sense;
pub use current_sense::CurrentSensing;
