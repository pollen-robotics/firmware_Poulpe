#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

//use core::fmt::Write;
use core::str::from_utf8;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use embassy_stm32::dma::NoDma;
//use embassy_stm32::peripherals::SPI4;
use embassy_stm32::time::mhz;
use embassy_stm32::{spi, Config};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    config.rcc.sys_ck = Some(mhz(400));
    config.rcc.hclk = Some(mhz(200));
    config.rcc.pll1.q_ck = Some(mhz(100));
    let p = embassy_stm32::init(config);

    let mut led_hello = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_error = Output::new(p.PC8, Level::High, Speed::Low);
    led_error.set_low();
   
    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(1);

    let mut spi = spi::Spi::new(p.SPI4, p.PE12, p.PE6, p.PE5, NoDma, NoDma, spi_config);
    let mut spi_driver_cs = Output::new(p.PC15, Level::High, Speed::Low);
    spi_driver_cs.set_high();
    let mut spi_foc_cs = Output::new(p.PE3, Level::High, Speed::Low);
    spi_foc_cs.set_high();

    loop {
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led_hello.set_low();

        // 40 bits (rw + addr 7 + data 32)
        //let mut data = [0x80u8, 0x00u8, 0x00u8, 0x00u8, 0b00000100u8];
//        let mut data = [0x55u8, 0x00u8, 0x00u8, 0x00u8, 0b10101010u8];
        let mut data = [0x00u8, 0x00u8, 0x00u8, 0x00u8, 0x04u8];

        spi_driver_cs.set_low();
        let result = spi.blocking_transfer_in_place(&mut data);
        if let Err(_) = result {
            defmt::panic!("crap");
        }
        spi_driver_cs.set_high();
        info!("read via spi: {}", from_utf8(&data).unwrap());
        Timer::after(Duration::from_millis(5000)).await;

    }
}
