#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use embassy_stm32::{spi, Config};
use {defmt_rtt as _, panic_probe as _};

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

    // Config gconf
    let mut gconf = [0x00u8, 0x00u8, 0x00u8, 0x00u8, 0x00u8];
    spi_driver_cs.set_low();
    let result = spi_j5_ventouse.blocking_transfer_in_place(&mut gconf);
    if let Err(_) = result {
        defmt::panic!("crap");
    }
    spi_driver_cs.set_high();

    info!("read GCONF: {:#04x}", gconf);
    Timer::after(Duration::from_millis(1000)).await;

    let mut gconf_w = [0x80u8, 0x00u8, 0x00u8, 0x00u8, 0x00u8]; // Switch to single line (aka 6PWM)
    spi_driver_cs.set_low();

    let result = spi_j5_ventouse.blocking_transfer_in_place(&mut gconf_w);
    if let Err(_) = result {
        defmt::panic!("crap");
    }
    spi_driver_cs.set_high();

    info!("write GCONF: {:#04x}", gconf_w);
    Timer::after(Duration::from_millis(1000)).await;

    spi_driver_cs.set_low();

    let result = spi_j5_ventouse.blocking_transfer_in_place(&mut gconf);
    if let Err(_) = result {
        defmt::panic!("crap");
    }
    spi_driver_cs.set_high();

    info!("read GCONF: {:#04x}", gconf);
    Timer::after(Duration::from_millis(1000)).await;

    info!("Set foc_b_enable");
    let mut foc_b_enable = Output::new(p.PE0, Level::High, Speed::Low);
    foc_b_enable.set_high();

    loop {
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;

        // Check TMC4761
/*         let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8, 0x00u8]; // Read CHIPINFO_DATA
        spi_foc_cs.set_low();
        let result = spi_j5_ventouse.blocking_transfer_in_place(&mut data_read);
        if let Err(_) = result {
            defmt::panic!("crap");
        }
        spi_foc_cs.set_high();
        info!("read : {:#04x}  {:#04x} {:#04x} {:#04x} {:#04x}", &data_read[0], &data_read[1], &data_read[2], &data_read[3], &data_read[4]);
 */
        led_hello.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
