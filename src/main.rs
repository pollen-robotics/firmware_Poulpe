#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::default;
use core::str::from_utf8;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::Word;
use embassy_time::{Duration, Timer};
use embassy_stm32::dma::NoDma;
use embassy_stm32::time::{mhz, khz};
use embassy_stm32::{spi, Config};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    info!("----------------- Clock config -----------------");
/*     let mut config = Config::default();
    config.rcc.sys_ck = Some(mhz(400));
    config.rcc.hclk = Some(mhz(200));
    config.rcc.pll1.q_ck = Some(mhz(100));
    */
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys_ck = Some(mhz(400));
    config.rcc.hclk = Some(mhz(200));
        let p = embassy_stm32::init(config);

    info!("----------------- LEDs config -----------------");
//    let mut led_green = Output::new(p.PC9, Level::High, Speed::Low);
    
    info!("----------------- SPI config -----------------");
    let mut lan9252_spi_config = spi::Config::default();
    lan9252_spi_config.frequency = khz(50);
    lan9252_spi_config.mode = spi::MODE_0;
    // Debug SPI on J2
//    let mut lan9252_spi = spi::Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, NoDma, NoDma, lan9252_spi_config);
//    let mut lan9252_spi_cs = Output::new(p.PA4, Level::High, Speed::Low);
    // Debug SPI on J4
    let mut lan9252_spi = spi::Spi::new(p.SPI6, p.PB3, p.PB5, p.PB4, NoDma, NoDma, lan9252_spi_config);
    let mut lan9252_spi_cs = Output::new(p.PA15, Level::High, Speed::Low);
    lan9252_spi_cs.set_high();


//---- LAN9252 commands ------------------------------------------------------------------------------
// COMM_SPI_READ    0x03
// COMM_SPI_WRITE   0x02
//---- LAN9252 registers ------------------------------    
// 0x0074      // hardware configuration register
// 0x0064      // byte order test register
// 0x01F8      // reset register       
// 0x0050      // chip ID and revision
// 0x0054      // interrupt configuration
// 0x005C      // interrupt enable
//---- LAN9252 flags ------------------------------------------------------------------------------
// ECAT_CSR_BUSY     0x80
// PRAM_ABORT        0x40000000
// PRAM_BUSY         0x80
// PRAM_AVAIL        0x01
// READY             0x08
// DIGITAL_RST       0x00000001

    info!("----------------- LAN9252 reset -----------------");
    lan9252_spi_cs.set_low();
    let data_write = [0x02u8, 0x01u8, 0xF8u8, 0x00u8, 0x00u8, 0x00u8, 0x01u8]; // write command_8, add_14+2, data_32
    let result = lan9252_spi.blocking_write(&data_write);
    if let Err(_) = result {
        defmt::panic!("crap write");
    }
    lan9252_spi_cs.set_high();

    loop {
        lan9252_spi_cs.set_low();
        let mut data_write = [0x03u8, 0x01u8, 0xF8u8];
        let result = lan9252_spi.blocking_write(&data_write);
        if let Err(_) = result {
            defmt::panic!("crap write");
        }
        let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8]; // 32 bits of data
        let result = lan9252_spi.blocking_read(&mut data_read);
        if let Err(_) = result {
            defmt::panic!("crap read");
        }
        lan9252_spi_cs.set_high();
        info!("RESET_CTL_reg / DIGITAL_RST_bit: {:#02x}.", &data_read[0]);
        if (&data_read[0] & 0x01u8) == 0x00u8 {
            break;
        }
        Timer::after(Duration::from_millis(10)).await;
    }

    lan9252_spi_cs.set_low();
    let mut data_write = [0x03u8, 0x00u8, 0x74u8];
    let result = lan9252_spi.blocking_write(&data_write);
    if let Err(_) = result {
        defmt::panic!("crap write");
    }
    let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8]; // 32 bits of data
    let result = lan9252_spi.blocking_read(&mut data_read);
    if let Err(_) = result {
        defmt::panic!("crap read");
    }
    lan9252_spi_cs.set_high();
    info!("HW_CFG_reg / READY_bit {:#02x}.", &data_read[3] );
    
    lan9252_spi_cs.set_low();
    let mut data_write = [0x03u8, 0x00u8, 0x64u8];
    let result = lan9252_spi.blocking_write(&data_write);
    if let Err(_) = result {
        defmt::panic!("crap write");
    }
    let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8]; // 32 bits of data
    let result = lan9252_spi.blocking_read(&mut data_read);
    if let Err(_) = result {
        defmt::panic!("crap read");
    }
    lan9252_spi_cs.set_high();
    info!("BYTE_TEST_reg: {:#02x} {:#02x} {:#02x} {:#02x}.", &data_read[3], &data_read[2], &data_read[1], &data_read[0]);
    

    info!("----------------- Main Loop -----------------");
    loop {
//        led_green.set_high();
//        Timer::after(Duration::from_millis(500)).await;
//        led_green.set_low();

        // SPI
        lan9252_spi_cs.set_low();
//        let mut data_write = [0x03u8, 0x00u8, 0x50u8]; // 03: read, x050: chip & ID
        let mut data_write = [0x03u8, 0x00u8, 0x74u8]; // 03: read, x064: Byte order (0x87654321)
//        data_write[0] = 0x03u8; // Sent first
//        data_write[1] = 0x00u8; 
//        data_write[2] = 0x64u8; // Sent last
        let result = lan9252_spi.blocking_write(&data_write);
        if let Err(_) = result {
            defmt::panic!("crap write");
        }
        let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8]; // 32 bits of data
        let result = lan9252_spi.blocking_read(&mut data_read);
        if let Err(_) = result {
            defmt::panic!("crap read");
        }
        lan9252_spi_cs.set_high();
        info!("read via spi: {:#02x} {:#02x} {:#02x} {:#02x}.", &data_read[3], &data_read[2], &data_read[1], &data_read[0]);

        Timer::after(Duration::from_millis(1000)).await;
    }
}
