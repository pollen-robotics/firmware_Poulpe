use core::cell::RefCell;

use crate::config;

use defmt::*;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::peripherals::SPI4;
use embassy_stm32::spi::{Config, Instance, MisoPin, MosiPin, SckPin, Spi};
use embassy_stm32::{spi};
use embassy_time::*;
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
// use embedded_hal_1::spi::{Operation, SpiDevice};
use embassy_sync::blocking_mutex::{NoopMutex, raw::NoopRawMutex};



use embedded_hal_1::spi::SpiDevice;
use static_cell::StaticCell;

use super::motors_io::IOError;


/*
pub struct AksimSensor<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    spi: Spi<'d, T, NoDma, NoDma>,
    cs: Output<'d, Cs>,

}

pub struct AksimSensorConfig<
    T: Instance,
    Cs: Pin,
    Sck: SckPin<T>,
    Mosi: MosiPin<T>,
    Miso: MisoPin<T>,

> {
    pub cs: Cs,
    pub peri: T,
    pub sck: Sck,
    pub mosi: Mosi,
    pub miso: Miso,
}

#[allow(dead_code)]
impl<'d, T, Cs>
    AksimSensor<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    const ANGLE_RANGE: f64 = 360.0;

    pub fn new(
        ring_sensor_config: AksimSensorConfig<
            T,
            Cs,
            impl SckPin<T>,
            impl MosiPin<T>,
            impl MisoPin<T>,
        >,
    ) -> Self {
        let spi_config = Config::default();
        // spi_config.mode = embassy_stm32::spi::MODE_3; //Ring=MODE0? 1MHz
	// Ring sensor is 3V3-powered and runs on SPI4 (J3)
        let spi = Spi::new(
            ring_sensor_config.peri,
            ring_sensor_config.sck,
            ring_sensor_config.mosi,
            ring_sensor_config.miso,
            NoDma,
            NoDma,
            spi_config,
        );

        // IOs
        let cs = Output::new(ring_sensor_config.cs, Level::High, Speed::Medium);

        Self {
            cs,
            spi,
        }
    }

    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
	self.cs.set_high();

        Ok(())
    }
    pub async fn read_angle(&mut self) -> Result<f64, embassy_stm32::spi::Error>
    {
	 // RLS Aksim2
        let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8];
        self.cs.set_low();
	let result = self.spi.blocking_read(&mut data_read);
        if result.is_err() {
            defmt::error!("Bad spi read (Ring sensor)");
	    return Err(result.err().unwrap());
        }
        self.cs.set_high();
        info!("read via spi: {:#02x}  {:#02x}  {:#02x} {:#02x}.", &data_read[0], &data_read[1], &data_read[2], &data_read[3]);

        let encoder_data: u64 = ((data_read[0] as u64) << 24) |
                                ((data_read[1] as u64) << 16) |
                                ((data_read[2] as u64) << 8)  |
                                 (data_read[3] as u64);
        // For single-turn
        // b31:b10 - Encoder position + zero padding bits. Left aligned, MSB first.
        // b9      - Error: if low, the position data is not valid.
        // b8      - Warning: if low, the position data is valid, but
        //                    some operating conditions are close to limits.
        // b7:b0   - Inverted CRC, 0x97 polynomial

        let mut encoder_position = encoder_data & 0x00000000ffffe000; // 19 bits on MB049
        encoder_position >>= 13; // 19 + 13 = 31 (MSB position of data)
        // Nota: 2^19 = 524288

        let angle = (encoder_position as f64 / 524288.0) * Self::ANGLE_RANGE;
        info!("Angle: {} degrees", angle);

	Ok(angle)


    }


}






pub struct AD5047Sensor<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    spi: Spi<'d, T, NoDma, NoDma>,
    cs: Output<'d, Cs>,

}

pub struct AD5047SensorConfig<
    T: Instance,
    Cs: Pin,
    Sck: SckPin<T>,
    Mosi: MosiPin<T>,
    Miso: MisoPin<T>,

> {
    pub cs: Cs,
    pub peri: T,
    pub sck: Sck,
    pub mosi: Mosi,
    pub miso: Miso,
}

#[allow(dead_code)]
impl<'d, T, Cs>
    AD5047Sensor<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    const ANGLE_RANGE: f64 = 360.0;

    pub fn new(
        center_sensor_config: AD5047SensorConfig<
            T,
            Cs,
            impl SckPin<T>,
            impl MosiPin<T>,
            impl MisoPin<T>,
        >,
    ) -> Self {
        let mut spi_config = Config::default();
        spi_config.mode = embassy_stm32::spi::MODE_1; //Center=MODE1? 1MHz
	// Center sensor is 3V3-powered and runs on SPI6 (J4)
        let spi = Spi::new(
            center_sensor_config.peri,
            center_sensor_config.sck,
            center_sensor_config.mosi,
            center_sensor_config.miso,
            NoDma,
            NoDma,
            spi_config,
        );

        // IOs
        let cs = Output::new(center_sensor_config.cs, Level::High, Speed::Medium);

        Self {
            cs,
            spi,
        }
    }

    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
	self.cs.set_high();

        Ok(())
    }
    pub async fn read_angle(&mut self) -> Result<f64, embassy_stm32::spi::Error>
    {
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


        self.cs.set_low();
	let data_write = [0x7fu8, 0xfeu8]; // read angle
        let result = self.spi.blocking_write(&data_write);
        if result.is_err() {
            defmt::error!("Error writing to Center sensor");
	    return Err(result.err().unwrap());
        }
        self.cs.set_high();
        Timer::after(Duration::from_micros(1)).await; // actually > 350 ns
        self.cs.set_low();
        let mut data_read = [0x00u8, 0x00u8];
        let result = self.spi.blocking_read(&mut data_read);
        if result.is_err() {
            defmt::error!("Error reading Center sensor");
	    return Err(result.err().unwrap());
        }
        self.cs.set_high();

	  // Combine the two u8 values into a 16-bit integer
        let mut combined_value: u16 = ((data_read[0] as u16) << 8) | (data_read[1] as u16);
        combined_value &= 0x3FFF;

        let angle = (combined_value as f64 / 16383.0) * Self::ANGLE_RANGE;
        info!("Angle: {} degrees", angle);

	Ok(angle)

    }


}
*/



//https://github.com/embassy-rs/embassy/blob/main/embassy-embedded-hal/src/shared_bus/blocking/spi.rs
//ex https://github.com/embassy-rs/embassy/blob/e0727fe1f6bd1b8c9c356f33991b522956251ba1/examples/rp/src/bin/spi_display.rs#L140


// pub struct SensorConfig<T, SCK, MOSI, MISO, Cs,>
// where
//     T: spi::Instance,
//     SCK: spi::SckPin<T>,
//     MOSI: spi::MosiPin<T>,
//     MISO: spi::MisoPin<T>,
//     Cs: Pin,

// {
//     pub peri: T,
//     pub sck: SCK,
//     pub mosi: MOSI,
//     pub miso: MISO,

//     pub cs: Cs,

// }
pub struct SensorConfig<Cs>
where

    Cs: Pin,

{

    pub cs: Cs,

}

pub struct AksimSensor<'d,T,P>
    where
    T: Instance,
    P: Pin,
{
    spi: SpiDeviceWithConfig<'d,
	    NoopRawMutex,
        Spi<'static, T, NoDma, NoDma>,
        Output<'static, P>,
        >,
}

#[allow(dead_code)]
impl<'d,T,P>
    AksimSensor<'d,T,P>
where T:Instance,
	  P:Pin,

{
    const ANGLE_RANGE: f64 = 360.0;

    pub fn new(
	spi: SpiDeviceWithConfig<'d,
		NoopRawMutex,
            Spi<'static, T, NoDma, NoDma>,
            Output<'static, P>,
        >,
    ) -> Self {
        // let spi_config = Config::default();
        // spi_config.mode = embassy_stm32::spi::MODE_3; //Ring=MODE0? 1MHz
	// Ring sensor is 3V3-powered and runs on SPI4 (J3)


	// let spi=spi;


        Self {
            spi,
        }
    }

    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
	// self.spi.cs.set_high();

        Ok(())
    }
    pub async fn read_angle(&mut self) -> Result<f64, embassy_stm32::spi::Error>
    {
	 // RLS Aksim2
        let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8];
        // self.cs.set_low();

	// let result = self.spi.blocking_read(&mut data_read);
	SpiDevice::transfer_in_place(&mut self.spi, &mut data_read)
	    .map_err(|e| {
                error!("!!! Error SPI {:?}!!!", e);
                embassy_stm32::spi::Error::Framing
            })?;





        // if result.is_err() {
        //     defmt::error!("Bad spi read (Ring sensor)");
	//     return Err(result.err().unwrap());
        // }
        // self.cs.set_high();
        info!("read via spi: {:#02x}  {:#02x}  {:#02x} {:#02x}.", &data_read[0], &data_read[1], &data_read[2], &data_read[3]);

        let encoder_data: u64 = ((data_read[0] as u64) << 24) |
                                ((data_read[1] as u64) << 16) |
                                ((data_read[2] as u64) << 8)  |
                                 (data_read[3] as u64);
        // For single-turn
        // b31:b10 - Encoder position + zero padding bits. Left aligned, MSB first.
        // b9      - Error: if low, the position data is not valid.
        // b8      - Warning: if low, the position data is valid, but
        //                    some operating conditions are close to limits.
        // b7:b0   - Inverted CRC, 0x97 polynomial

        let mut encoder_position = encoder_data & 0x00000000ffffe000; // 19 bits on MB049
        encoder_position >>= 13; // 19 + 13 = 31 (MSB position of data)
        // Nota: 2^19 = 524288

        let angle = (encoder_position as f64 / 524288.0) * Self::ANGLE_RANGE;
        info!("Angle: {} degrees", angle);

	Ok(angle)


    }


}






pub struct AD5047Sensor<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    spi: Spi<'d, T, NoDma, NoDma>,
    cs: Output<'d, Cs>,

}

pub struct AD5047SensorConfig<
    T: Instance,
    Cs: Pin,
    Sck: SckPin<T>,
    Mosi: MosiPin<T>,
    Miso: MisoPin<T>,

> {
    pub cs: Cs,
    pub peri: T,
    pub sck: Sck,
    pub mosi: Mosi,
    pub miso: Miso,
}

#[allow(dead_code)]
impl<'d, T, Cs>
    AD5047Sensor<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    const ANGLE_RANGE: f64 = 360.0;

    pub fn new(
        center_sensor_config: AD5047SensorConfig<
            T,
            Cs,
            impl SckPin<T>,
            impl MosiPin<T>,
            impl MisoPin<T>,
        >,
    ) -> Self {
        let mut spi_config = Config::default();
        spi_config.mode = embassy_stm32::spi::MODE_1; //Center=MODE1? 1MHz
	// Center sensor is 3V3-powered and runs on SPI6 (J4)
        let spi = Spi::new(
            center_sensor_config.peri,
            center_sensor_config.sck,
            center_sensor_config.mosi,
            center_sensor_config.miso,
            NoDma,
            NoDma,
            spi_config,
        );

        // IOs
        let cs = Output::new(center_sensor_config.cs, Level::High, Speed::Medium);

        Self {
            cs,
            spi,
        }
    }

    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
	self.cs.set_high();

        Ok(())
    }
    pub async fn read_angle(&mut self) -> Result<f64, embassy_stm32::spi::Error>
    {
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


        self.cs.set_low();
	let data_write = [0x7fu8, 0xfeu8]; // read angle
        let result = self.spi.blocking_write(&data_write);
        if result.is_err() {
            defmt::error!("Error writing to Center sensor");
	    return Err(result.err().unwrap());
        }
        self.cs.set_high();
        Timer::after(Duration::from_micros(1)).await; // actually > 350 ns
        self.cs.set_low();
        let mut data_read = [0x00u8, 0x00u8];
        let result = self.spi.blocking_read(&mut data_read);
        if result.is_err() {
            defmt::error!("Error reading Center sensor");
	    return Err(result.err().unwrap());
        }
        self.cs.set_high();

	  // Combine the two u8 values into a 16-bit integer
        let mut combined_value: u16 = ((data_read[0] as u16) << 8) | (data_read[1] as u16);
        combined_value &= 0x3FFF;

        let angle = (combined_value as f64 / 16383.0) * Self::ANGLE_RANGE;
        info!("Angle: {} degrees", angle);

	Ok(angle)

    }


}
