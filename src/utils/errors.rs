use defmt::Format;
use embassy_stm32::{i2c, spi};

pub type Result<T> = core::result::Result<T, IOError>;

#[derive(Debug, Format)]
pub enum IOError {
    SpiError(spi::Error),
    I2cError,
    InvalidData,
    InvalidState,
    Unavailable,
    InitError,
    DriverError,
    CommunicationError,
    SensorError,
}

#[derive(Debug, Format)]
pub enum ConversionError {
    InvalidDataLength,
    NanReceived,
}

#[derive(Debug, Format)]
pub enum DriverError {
    SpiError(spi::Error),
    ConfigError,
    FaultState,
}
