#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

//use core::fmt::Write;
use core::str::from_utf8;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
//use embassy_stm32::peripherals::SPI4;
use embassy_stm32::time::mhz;
use embassy_stm32::{spi, Config};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    // config.rcc.sys_ck = Some(mhz(400));
    // config.rcc.hclk = Some(mhz(200));
    // config.rcc.pll1.q_ck = Some(mhz(100));
    let p = embassy_stm32::init(config);

    let mut led_hello = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_error = Output::new(p.PC8, Level::High, Speed::Low);
    led_error.set_low();

    let mut spi_config = spi::Config::default();
    // spi_config.frequency = mhz(1);
    spi_config.mode = spi::MODE_3;

    let mut spi = spi::Spi::new(p.SPI4, p.PE12, p.PE6, p.PE5, NoDma, NoDma, spi_config);
    let mut spi_driver_cs = Output::new(p.PC15, Level::High, Speed::Low);
    spi_driver_cs.set_high();
    let mut spi_foc_cs = Output::new(p.PE3, Level::High, Speed::Low);
    spi_foc_cs.set_high();

    let mut add = 0x00u8;

    //config gconf
    let mut gconf = [0x00u8, 0x00u8, 0x00u8, 0x00u8, 0x00u8];
    // if add < 0x0A {
    //     add += 1;
    // }
    spi_driver_cs.set_low();
    // spi_foc_cs.set_low();

    let result = spi.blocking_transfer_in_place(&mut gconf);
    if let Err(_) = result {
        defmt::panic!("crap");
    }
    spi_driver_cs.set_high();
    // spi_foc_cs.set_high();

    info!("read GCONF: {:#04x}", gconf);
    Timer::after(Duration::from_millis(1000)).await;

    let mut gconf_w = [0x80u8, 0x00u8, 0x00u8, 0x00u8, 0x00u8];
    // if add < 0x0A {
    //     add += 1;
    // }
    spi_driver_cs.set_low();
    // spi_foc_cs.set_low();

    let result = spi.blocking_transfer_in_place(&mut gconf_w);
    if let Err(_) = result {
        defmt::panic!("crap");
    }
    spi_driver_cs.set_high();
    // spi_foc_cs.set_high();

    info!("write GCONF: {:#04x}", gconf_w);
    Timer::after(Duration::from_millis(1000)).await;

    spi_driver_cs.set_low();
    // spi_foc_cs.set_low();

    let result = spi.blocking_transfer_in_place(&mut gconf);
    if let Err(_) = result {
        defmt::panic!("crap");
    }
    spi_driver_cs.set_high();
    // spi_foc_cs.set_high();

    info!("read GCONF: {:#04x}", gconf);
    Timer::after(Duration::from_millis(1000)).await;

    info!("Set foc_b_enable");
    let mut foc_b_enable = Output::new(p.PE0, Level::High, Speed::Low);
    foc_b_enable.set_high();

    loop {
        // info!("high");
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;
        // info!("low");
        led_hello.set_low();
        Timer::after(Duration::from_millis(500)).await;
        // 40 bits (rw + addr 7 + data 32)
        //let mut data = [0x80u8, 0x00u8, 0x00u8, 0x00u8, 0b00000100u8];
        //        let mut data = [0x55u8, 0x00u8, 0x00u8, 0x00u8, 0b10101010u8];
        // let mut data = [0x04u8, 0x00u8, 0x00u8, 0x00u8, 0x00u8];
        // let mut data = [add, 0x00u8, 0x00u8, 0x00u8, 0x00u8];

        // let mut data = [0x00u8, 0x00u8, 0x00u8, 0x00u8, 0x00u8];
    }
}
