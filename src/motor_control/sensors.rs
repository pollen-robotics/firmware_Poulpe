use core::cell::RefCell;

use crate::config;

use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_embedded_hal::SetConfig;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Level, Output, Pin, Pull, Speed};

use embassy_stm32::i2c::{Error, I2c, Instance as I2cInstance};
use embassy_stm32::peripherals::SPI4;
use embassy_stm32::spi::{Config, Instance, MisoPin, MosiPin, SckPin, Spi};
use embassy_stm32::{i2c, spi};
use embassy_time::*;
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
// use embedded_hal_1::spi::{Operation, SpiDevice};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, NoopMutex};

use embedded_hal_1::spi::SpiDevice;
// use embedded_hal_1::i2c::I2cDevice as embedded_hal_I2cDevice;
// use embedded_hal_1::i2c::I2c::BlockingRead;
// use cortex_m::prelude::_embedded_hal_blocking_i2c_Read;

use static_cell::StaticCell;

use super::motors_io::IOError;
use super::RawSensorsIO;

const ADDRESS_A: u8 = 0x38;
const ADDRESS_B: u8 = 0x39;

// CRC Code from: https://www.rls.si/media/custom/upload/CRCD01_03.pdf
//Lookup table for polynome = 0x97
// P(x) = x8 + x7 + x4 + x2 + x1 + 1
const ab_CRC8_LUT: [u8; 256] = [
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
    0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
    0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
    0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
    0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
    0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
    0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
    0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
    0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
    0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
    0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
    0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
    0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
    0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
    0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C,
];
/* CRC 0x97 Polynomial, 64-bit input data, right alignment, calculation over 64 bits */

pub fn CRC_SPI_97_64bit(dw_InputData: u64) -> u8 {
    let mut b_Index: u8 = 0;
    let mut b_CRC: u8 = 0;
    b_Index = ((dw_InputData >> 56) & 0x000000FF) as u8;
    b_CRC = ((dw_InputData >> 48) & 0x000000FF) as u8;
    b_Index = b_CRC ^ ab_CRC8_LUT[b_Index as usize];
    b_CRC = ((dw_InputData >> 40) & 0x000000FF) as u8;
    b_Index = b_CRC ^ ab_CRC8_LUT[b_Index as usize];
    b_CRC = ((dw_InputData >> 32) & 0x000000FF) as u8;
    b_Index = b_CRC ^ ab_CRC8_LUT[b_Index as usize];
    b_CRC = ((dw_InputData >> 24) & 0x000000FF) as u8;
    b_Index = b_CRC ^ ab_CRC8_LUT[b_Index as usize];
    b_CRC = ((dw_InputData >> 16) & 0x000000FF) as u8;
    b_Index = b_CRC ^ ab_CRC8_LUT[b_Index as usize];
    b_CRC = ((dw_InputData >> 8) & 0x000000FF) as u8;
    b_Index = b_CRC ^ ab_CRC8_LUT[b_Index as usize];
    b_CRC = (dw_InputData & 0x000000FF) as u8;
    b_Index = b_CRC ^ ab_CRC8_LUT[b_Index as usize];
    b_CRC = ab_CRC8_LUT[b_Index as usize];
    b_CRC
}

pub enum SensorKind<'d> {
    #[allow(dead_code)]
    Ring(config::Aksim<'d>),
    Center(config::AD5047<'d>),
    DonutTop(config::AD5047Top<'d>),
    DonutMid(config::AD5047Mid<'d>),
    DonutBot(config::AD5047Bot<'d>),

    DonutHall(config::DonutHall<'d>),
}

pub struct SensorConfig<Cs>
where
    Cs: Pin,
{
    pub cs: Cs,
}

pub struct I2cHallConfig<T, Scl, Sda>
where
    T: I2cInstance,
    Scl: Pin,
    Sda: Pin,
{
    pub peri: T,
    pub scl: Scl,
    pub sda: Sda,
}

pub struct I2cHallSensor<T>
where
    T: I2cInstance,
    // Scl: Pin,
    // Sda: Pin,
    // embassy_stm32::i2c::I2c<'static, T, Scl, Sda>: SetConfig
{
    i2c: I2c<'static, T>,
}

impl<T> I2cHallSensor<T>
where
    T: I2cInstance,
    // Scl: Pin,
    // Sda: Pin,
    // embassy_stm32::i2c::I2c<'static, T, Scl, Sda>: _embedded_hal_blocking_i2c_Read
{
    pub fn new(i2c: I2c<'static, T>) -> Self {
        Self { i2c }
    }
    pub fn read(&mut self) -> Result<u16, IOError> {
        let mut data = [0u8; 1];
        let mut hall_detected = 0u16;
        // debug!("Reading Hall sensor");

        match self.i2c.blocking_read(ADDRESS_A, &mut data) {
            Ok(()) => {
                // debug!("Inputs_A: {:#010b}", data[0]);
                //            hall_detected = (data[0] as u16) << 8;
                hall_detected = data[0] as u16;
            }
            // Err(Error::Timeout) => info!("Operation timed out"),
            //Why is this so complicated!!! I cannot return the original error thanks to a dozain levels of abstraction...
            Err(e) => {
                error!("Input A error: {:?}", e);
                return Err(IOError::I2cError);
            }
        }

        match self.i2c.blocking_read(ADDRESS_B, &mut data) {
            Ok(()) => {
                // debug!("Inputs_B: {:#010b}", data[0]);
                //            hall_detected = hall_detected | (data[0] as u16);
                hall_detected |= ((data[0] as u16) << 8);
            }
            // Err(Error::Timeout) => info!("Operation timed out"),
            // Err(e) => info!("I2c Error: {:?}", e),
            Err(e) => {
                error!("Input B error: {:?}", e);
                return Err(IOError::I2cError);
            }
        }
        Ok(hall_detected)
    }

    pub fn get_index(&mut self) -> Result<[u16; 1], IOError> {
        match self.read() {
            Ok(hall_detected) => Ok([hall_detected]),
            Err(e) => Err(e),
        }
    }
}

// I can't get this to work, so I'm using the "standard" blocking version above

// pub struct I2cHallSensor<'d,T, Scl, Sda>
// where
//     T: I2cInstance,
//     Scl: Pin,
//     Sda: Pin,
//     // embassy_stm32::i2c::I2c<'static, T, Scl, Sda>: SetConfig

// {
// 	i2c: I2cDevice<'d,NoopRawMutex, I2c<'static, T, Scl, Sda >>,
// }

// impl<'d, T, Scl, Sda>I2cHallSensor<'d,T,Scl,Sda>
// where
// 	  T: I2cInstance,
// 	  Scl: Pin,
// 	  Sda: Pin,
//     // embassy_stm32::i2c::I2c<'static, T, Scl, Sda>: _embedded_hal_blocking_i2c_Read
// {
//     pub fn new(i2c: I2cDevice<'d,NoopRawMutex, I2c<'static, T, Scl, Sda >>,) -> Self {
// 		Self {
// 			i2c,
// 		}
// 	}
//     pub fn read(&mut self) -> Result<u16,IOError>
//     {
// 	let mut data = [0u8; 1];
// 	let mut hall_detected = 0u16;
// 	match I2cDevice::read(&mut self.i2c, ADDRESS_A, &mut data) {
//             Ok(()) => {
// 		//info!("Inputs_A: {:#010b}", data[0]);
// 		//            hall_detected = (data[0] as u16) << 8;
// 		hall_detected = data[0] as u16;
//             },
//             // Err(Error::Timeout) => info!("Operation timed out"),
// 	    //Why is this so complicated!!! I cannot return the original error thanks to a dozain levels of abstraction...
//             Err(e) => {return Err(IOError::I2cError)},
// 	}

// 	match I2cDevice::read(&mut self.i2c, ADDRESS_B, &mut data) {
//             Ok(()) => {
// 		//info!("Inputs_B: {:#010b}", data[0]);
// 		//            hall_detected = hall_detected | (data[0] as u16);
// 		hall_detected |= ((data[0] as u16) << 8);
//             },
//             // Err(Error::Timeout) => info!("Operation timed out"),
//             // Err(e) => info!("I2c Error: {:?}", e),
//             Err(e) => {return Err(IOError::I2cError)},
// 	}
// 	Ok(hall_detected)
//     }

// }

pub struct AksimSensor<'d, T, P>
where
    T: Instance,
    P: Pin,
{
    spi: SpiDeviceWithConfig<'d, NoopRawMutex, Spi<'static, T, NoDma, NoDma>, Output<'static, P>>,
}

#[allow(dead_code)]
impl<'d, T, P> AksimSensor<'d, T, P>
where
    T: Instance,
    P: Pin,
{
    const ANGLE_RANGE: f64 = 2.0 * 3.14159265;

    pub fn new(
        spi: SpiDeviceWithConfig<
            'd,
            NoopRawMutex,
            Spi<'static, T, NoDma, NoDma>,
            Output<'static, P>,
        >,
    ) -> Self {
        // let spi_config = Config::default();
        // spi_config.mode = embassy_stm32::spi::MODE_3; //Ring=MODE0? 1MHz
        // Ring sensor is 3V3-powered and runs on SPI4 (J3)

        // let spi=spi;

        Self { spi }
    }

    pub fn init(&mut self) -> Result<(), IOError> {
        // self.spi.cs.set_high();

        Ok(())
    }
    pub fn read_angle(&mut self) -> Result<[f32; 1], IOError> {
        // RLS Aksim2
        let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8];
        // block_for(Duration::from_micros(10000));
        let _ = SpiDevice::transfer_in_place(&mut self.spi, &mut data_read).map_err(|e| {
            error!("!!! Error SPI {:?}!!!", e);
            IOError::SpiError
        });

        // let _ = SpiDevice::read(&mut self.spi, &mut data_read).map_err(|e| {
        //         error!("!!! Error SPI {:?}!!!", e);
        //         IOError::SpiError
        //     });

        // debug!("read via spi: {:#02x}  {:#02x}  {:#02x} {:#02x}.", &data_read[0], &data_read[1], &data_read[2], &data_read[3]);

        let encoder_data: u64 = ((data_read[0] as u64) << 24)
            | ((data_read[1] as u64) << 16)
            | ((data_read[2] as u64) << 8)
            | (data_read[3] as u64);
        // For single-turn
        // b31:b10 - Encoder position + zero padding bits. Left aligned, MSB first. b12:b10 are zero padding bits.
        // b9      - Error: if low, the position data is not valid.
        // b8      - Warning: if low, the position data is valid, but
        //                    some operating conditions are close to limits.
        // b7:b0   - Inverted CRC, 0x97 polynomial

        let encoder_position: u32 = ((encoder_data & 0x00000000ffffe000) >> 13) as u32; // 19 bits on MB049
                                                                                        // encoder_position >>= 13; // 19 + 13 = 31 (MSB position of data)
                                                                                        // Nota: 2^19 = 524288
        let error: bool = ((encoder_data & 0x0000000000000200) >> 9) != 0x1; // 9th bit, active low
                                                                             // error >>= 9;
        let _warning: bool = ((encoder_data & 0x0000000000000100) >> 8) != 0x1; // 8th bit, active low
        if error {
            error!("Ring sensor error",);
            Err(IOError::InvalidData)
        } else {
            let crc: u8 = (encoder_data & 0x00000000000000ff) as u8; // 7-0 bits //TODO
            let datapacket: u64 = (encoder_data >> 8) & 0x0000000000ffffff;
            let calculated_crc = !CRC_SPI_97_64bit(datapacket);

            if calculated_crc != crc {
                error!("Ring sensor CRC error. crc: {:#02x} computed: {:#02x} data: {:#x} datapacket: {:#x}", crc, calculated_crc, encoder_data, datapacket);
                Err(IOError::InvalidData)
            } else {
                // debug!("CRC {:#02x} computed {:#02x} data {:#x} datapacket {:#x}", crc, calculated_crc, encoder_data, datapacket);
                let angle = ((encoder_position as f64 / 524288.0) * Self::ANGLE_RANGE) as f32;
                // debug!("encoder position: {:?} angle {:?} ]warn {:?}",encoder_position,angle, _warning);
                //debug
                /*
                if error{
                error!("Ring Angle: raw: {}  deg: {} error: {} warn: {}", encoder_position,angle, error, warning);
                }else{
                debug!("Ring Angle: raw: {}  deg: {} error: {} warn: {}", encoder_position,angle, error, warning);
                }
                 */
                //Return result
                Ok([angle])
            }
        }
    }

    pub fn get_axis_sensor(&mut self) -> Result<[f32; 1], IOError> {
        self.read_angle()
    }
}

pub struct AD5047Sensor<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    spi: SpiDeviceWithConfig<'d, NoopRawMutex, Spi<'static, T, NoDma, NoDma>, Output<'static, Cs>>,
}

// pub struct AD5047SensorConfig<
//     T: Instance,
//     Cs: Pin,
//     Sck: SckPin<T>,
//     Mosi: MosiPin<T>,
//     Miso: MisoPin<T>,

// > {
//     pub cs: Cs,
//     pub peri: T,
//     pub sck: Sck,
//     pub mosi: Mosi,
//     pub miso: Miso,
// }

#[allow(dead_code)]
impl<'d, T, Cs> AD5047Sensor<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    const ANGLE_RANGE: f64 = 2.0 * 3.14159265;

    pub fn new(
        spi: SpiDeviceWithConfig<
            'd,
            NoopRawMutex,
            Spi<'static, T, NoDma, NoDma>,
            Output<'static, Cs>,
        >,
    ) -> Self {
        Self { spi }
    }

    pub fn init(&mut self) -> Result<(), IOError> {
        Ok(())
    }
    pub fn read_angle(&mut self) -> Result<[f32; 1], IOError> {
        // Command: 16-bit frame with bit 15 as even parity, bit 14 as read(1)/write(0), [13-0] as data
        // Answer:  16-bit frame with bit 15 as even parity, bit 14 as error(1)/no_error(0), [13-0] as data
        //---- AD5047 commands ------------------------------------------------------------------------------
        // addr   type     default   comment
        // 0x0000 NOP      0x0000    No operation
        // 0x0001 ERRFL    0x0000    Error register
        // 0x0003 PROG     0x0000    Programming register
        // 0x3FFC DIAAGC   0x0180    Diagnostic and AGC
        // 0x3FFD MAG      0x0000    CORDIC magnitude
        // 0x3FFE ANGLEUNC 0x0000    Measured angle without dynamic angle error compensation
        // 0x3FFF ANGLECOM 0x0000    Measured angle

        let mut data_write = [0x7fu8, 0xfeu8]; // read angle

        // let result = self.spi.blocking_write(&data_write);
        // if result.is_err() {
        //     defmt::error!("Error writing to Center sensor");
        //     return Err(result.err().unwrap());
        // }

        // block_for(Duration::from_micros(10));

        let _ = SpiDevice::transfer_in_place(&mut self.spi, &mut data_write).map_err(|e| {
            error!("!!! Error SPI {:?}!!!", e);
            IOError::SpiError
        });
        // block_for(Duration::from_micros(10));

        // block_for(Duration::from_micros(1));

        // block_for(Duration::from_micros(1));

        let mut data_read = [0x00u8, 0x00u8];

        let _ = SpiDevice::transfer_in_place(&mut self.spi, &mut data_read).map_err(|e| {
            error!("!!! Error SPI {:?}!!!", e);
            IOError::SpiError
        });

        // let result = self.spi.blocking_read(&mut data_read);
        // if result.is_err() {
        //     defmt::error!("Error reading Center sensor");
        //     return Err(result.err().unwrap());
        // }

        // Combine the two u8 values into a 16-bit integer
        let mut combined_value: u16 = ((data_read[0] as u16) << 8) | (data_read[1] as u16);
        // check if data has good parity
        if (combined_value.count_ones() % 2) != 0 {
            // error!("Parity error reading Center sensor");
            return Err(IOError::InvalidData);
        }
        combined_value &= 0x3FFF;

        let angle = ((combined_value as f64 / 16383.0) * Self::ANGLE_RANGE) as f32;
        // debug!("Center Angle: {} degrees", angle);

        Ok([angle])
    }
    pub fn get_axis_sensor(&mut self) -> Result<[f32; 1], IOError> {
        self.read_angle()
    }
}

impl<'d> RawSensorsIO<1> for SensorKind<'d> {
    /// Check if the motors are ON or OFF
    fn get_axis_sensors(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            SensorKind::Ring(ring) => ring.get_axis_sensor(),
            SensorKind::Center(center) => center.get_axis_sensor(),
            SensorKind::DonutTop(top) => top.get_axis_sensor(),
            SensorKind::DonutMid(mid) => mid.get_axis_sensor(),
            SensorKind::DonutBot(bot) => bot.get_axis_sensor(),

            SensorKind::DonutHall(hall) => Err(IOError::Unavailable),
        }
    }
    // fn get_index_sensors(&mut self) -> Result<[u16;1], IOError>
    // {
    // 	match self{
    // 	    SensorKind::DonutHall(hall) => hall.get_index(),
    // 	    _ => Err(IOError::Unavailable),

    // 	}
    // }
}
