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
mod tmc4671;

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

    // Configure SPI/CS (J5 / SPI4)
//    let mut driver_a = tmc6200::TMC6200::new(p.PC15, p.PE12, p.PE5, p.PE6, p.SPI4, NoDma, NoDma);
    let mut foc_a = tmc4671::TMC4761::new(p.PE3, p.PE12, p.PE5, p.PE6, p.SPI4, NoDma, NoDma);

    let write_b = false;
    let reg_addr = 0x00u8; // Read Chip info data
    let mut data = 0x00000000u32;
    let res = foc_a.transmit_raw_data(write_b, reg_addr, &mut data);
    if let Err(_) = res {
        defmt::panic!("crap_from_fn!");
    }
    info!("read GCONF from fn: {:#04x}", res.unwrap());
    Timer::after(Duration::from_millis(1000)).await;

    loop {
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;

        led_hello.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
