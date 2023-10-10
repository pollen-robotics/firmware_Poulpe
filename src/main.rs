#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use embassy_stm32::{spi, Config};
use embassy_stm32::spi::Spi;
use embassy_stm32::peripherals::SPI4;
use embassy_stm32::peripherals::PC15;
use embassy_stm32::peripherals::PE3;
use {defmt_rtt as _, panic_probe as _};

mod tmc6200;

fn tmc4671_transmit_raw_data(
    write_bit: bool,
    addr: u8,
    data: &mut u32,
    per: &mut Spi<'_, SPI4, NoDma, NoDma>,
    cs: &mut Output<'_, PE3>,
) -> Result<[u8; 4], embassy_stm32::spi::Error> {
    // Building array
    let mut msb_data = addr;
    if write_bit == true {
        msb_data = addr | 0b10000000;
    }
    let data_u8_array = data.to_le_bytes();
    let mut transfer_data = [msb_data, data_u8_array[0], data_u8_array[1], data_u8_array[2], data_u8_array[3]];

    // Sending data
    cs.set_low();
    let result = per.blocking_transfer_in_place(&mut transfer_data); // Todo: the error is not treated.
    cs.set_high();

    let mut read_data = [0x00u8; 4];
    read_data[0] = transfer_data[1];
    read_data[1] = transfer_data[2];
    read_data[2] = transfer_data[3];
    read_data[3] = transfer_data[4];

    Ok(read_data)
}

fn tmc6200_transmit_raw_data(
    write_bit: bool,
    addr: u8,
    data: &mut u32,
    per: &mut Spi<'_, SPI4, NoDma, NoDma>,
    cs: &mut Output<'_, PC15>,
) -> Result<[u8; 4], embassy_stm32::spi::Error> {
    // Building array
    let mut msb_data = addr;
    if write_bit == true {
        msb_data = addr | 0b10000000;
    }
    let data_u8_array = data.to_le_bytes();
    let mut transfer_data = [msb_data, data_u8_array[0], data_u8_array[1], data_u8_array[2], data_u8_array[3]];

    // Sending data
    cs.set_low();
    let result = per.blocking_transfer_in_place(&mut transfer_data); // Todo: the error is not treated.
    cs.set_high();

    let mut read_data = [0x00u8; 4];
    read_data[0] = transfer_data[1];
    read_data[1] = transfer_data[2];
    read_data[2] = transfer_data[3];
    read_data[3] = transfer_data[4];

    Ok(read_data)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let config = Config::default();
    let p = embassy_stm32::init(config);

    let mut led_hello = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_error = Output::new(p.PC8, Level::High, Speed::Low);
    led_error.set_low();
    led_hello.set_low();

    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_3;

    // Uses J5 / SPI4
    let mut spi_j5_ventouse = spi::Spi::new(p.SPI4, p.PE12, p.PE6, p.PE5, NoDma, NoDma, spi_config);
    let mut spi_driver_cs = Output::new(p.PC15, Level::High, Speed::Low);
    spi_driver_cs.set_high();
    let mut spi_foc_cs = Output::new(p.PE3, Level::High, Speed::Low);
    spi_foc_cs.set_high();

    let write_b = false;
    let reg_addr = 0x00u8;
    let mut data = 0x00000000u32;
    let result = tmc6200_transmit_raw_data(write_b, reg_addr, &mut data, &mut spi_j5_ventouse, &mut spi_driver_cs);
    if let Err(_) = result {
        defmt::panic!("crap_from_fn!");
    }
    info!("read GCONF from fn: {:#04x}", result.unwrap());
    Timer::after(Duration::from_millis(1000)).await;

    let write_b = true;
    let reg_addr = 0x00u8;
    let mut data = 0x00000000u32; // Switch to single line (aka 6PWM)
    let result = tmc6200_transmit_raw_data(write_b, reg_addr, &mut data, &mut spi_j5_ventouse, &mut spi_driver_cs);
    if let Err(_) = result {
        defmt::panic!("crap_from_fn!");
    }
    info!("write GCONF from fn: {:#04x}", result.unwrap());
    Timer::after(Duration::from_millis(1000)).await;

    let write_b = false;
    let reg_addr = 0x00u8;
    let mut data = 0x00000000u32;
    let result = tmc6200_transmit_raw_data(write_b, reg_addr, &mut data, &mut spi_j5_ventouse, &mut spi_driver_cs);
    if let Err(_) = result {
        defmt::panic!("crap_from_fn!");
    }
    info!("read GCONF from fn: {:#04x}", result.unwrap());
    Timer::after(Duration::from_millis(1000)).await;


    info!("Set foc_b_enable");
    let mut foc_b_enable = Output::new(p.PE0, Level::High, Speed::Low);
    foc_b_enable.set_high();

    loop {
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;

        // Check TMC4761
        let write_b = false;
        let reg_addr = 0x00u8; // Read Chip info data
        let mut data = 0x00000000u32;
        let result = tmc4671_transmit_raw_data(write_b, reg_addr, &mut data, &mut spi_j5_ventouse, &mut spi_foc_cs);
        if let Err(_) = result {
            defmt::panic!("crap_from_fn!");
        }
        info!("read from fn: {:#04x}", result.unwrap());

        led_hello.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
