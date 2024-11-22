

use crate::utils::errors::{Result, IOError};


use defmt::*;
use embassy_stm32::spi::{Instance, Spi};
use embassy_stm32::gpio::{Output, Pin};
use embassy_sync::blocking_mutex::Mutex;
use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal_1::spi::SpiDevice;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Speed};


#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum LTC4332Register{ 
    CONFIG  = 0x00,
    STATUS = 0x01,
    EVENT = 0x02,
    INT = 0x03,
    FAULT = 0x04,
    WORD_LENGTH = 0x05,
    SCRATCH  = 0x06,
}


#[derive(Debug, Format, Copy, Clone)]
pub enum LTC4332Config {
    Ring,
    Center,
    Donut,
}

impl LTC4332Config {
    pub fn to_u8(&self) -> u8 {
        match self {
            LTC4332Config::Ring => 0b00000001, // MODE1 slave 1
            LTC4332Config::Center => 0b00000001, // MODE1 slave 1
            LTC4332Config::Donut => 0b00010101, // MODE1 to slaves 1,2 and 3
        }
    }
}


pub struct LTC4332<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    pub spi: SpiDeviceWithConfig<
        'd,
        NoopRawMutex,
        Spi<'static, T, NoDma, NoDma>,
        Output<'static, Cs>,
    >
}


impl<'d, T, Cs> LTC4332<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    pub fn new( 
        spi: SpiDeviceWithConfig<
        'd,
        NoopRawMutex,
        Spi<'static, T, NoDma, NoDma>,
        Output<'static, Cs>,
        >) -> Self {
        Self {
            spi
        }
    }

    pub  fn setup(&mut self, config: LTC4332Config) -> Result<()> {
        self.write_config(config.to_u8())
    }


    pub  fn read_reg(&mut self, reg: LTC4332Register) ->  Result<u8> {
        let mut data_ltc_reg = [(reg as u8) << 1 | 1, 0, 0]; // reg_addr
        match self.spi.transfer_in_place(&mut data_ltc_reg){
            Ok(_) => {},
            Err(e) => {
                error!("Error: error reading LTC4332 (reg {=u8:#x})", reg as u8);
                return Err(IOError::CommunicationError);
            }
        }
        Ok(data_ltc_reg[1])
    }

    pub  fn read_status(&mut self) -> Result<u8> {
        self.read_reg(LTC4332Register::STATUS)
    }

    pub  fn read_config(&mut self) -> Result<u8> {
        self.read_reg(LTC4332Register::CONFIG)
    }

    pub  fn read_int(&mut self) -> Result<u8> {
        self.read_reg(LTC4332Register::INT)
    }

    pub  fn read_fault(&mut self) -> Result<u8> {
        self.read_reg(LTC4332Register::FAULT)
    }

    pub  fn read_word_length(&mut self) -> Result<u8> {
        self.read_reg(LTC4332Register::WORD_LENGTH)
    }

    pub  fn read_scratch(&mut self) -> Result<u8> {
        self.read_reg(LTC4332Register::SCRATCH)
    }


    pub  fn write_reg(&mut self, reg: LTC4332Register, data: u8) -> Result<()> {
        let mut data_ltc = [(reg as u8), data]; // reg_addr, data, crc (opt.)
        match self.spi.transfer_in_place(&mut data_ltc){
            Ok(_) => {},
            Err(_) => {
                error!("Error: error writing LTC4332 (reg {=u8:#x})", reg as u8);
                return Err(IOError::CommunicationError);
            }
        }
        let val = self.read_reg(reg)?;
        // check that the value is the same
        if val != data {
            error!("Error: error verifying LTC4332 configuration");
            return Err(IOError::InitError);
        }
        Ok(())
    }

    pub  fn write_scratch(&mut self, data: u8) -> Result<()> {
        self.write_reg(LTC4332Register::SCRATCH, data)
    }

    pub  fn write_word_length(&mut self, data: u8) -> Result<()> {
        self.write_reg(LTC4332Register::WORD_LENGTH, data)
    }

    pub  fn write_event(&mut self, data: u8) -> Result<()> {
        self.write_reg(LTC4332Register::EVENT, data)
    }

    pub  fn write_config(&mut self, data: u8) -> Result<()> {
        self.write_reg(LTC4332Register::CONFIG, data)
    }

}
