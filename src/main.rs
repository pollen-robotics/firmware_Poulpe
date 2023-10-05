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
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys_ck = Some(mhz(400));
    config.rcc.hclk = Some(mhz(200));
    let p = embassy_stm32::init(config);

    info!("----------------- LEDs config -----------------");
    let mut led_green = Output::new(p.PC9, Level::High, Speed::Low);
    
    info!("----------------- SPI config -----------------");
    // Carte Ticket (Ring sensor) is 3V3-powered and runs on SPI4 (J3)
    let mut sensor_ring_spi_config = spi::Config::default();
    sensor_ring_spi_config.frequency = mhz(1); // 4 MHz max clk
    let mut sensor_ring_spi = spi::Spi::new(p.SPI4, p.PE12, p.PE6, p.PE5, NoDma, NoDma, sensor_ring_spi_config);
    let mut sensor_ring_spi_cs = Output::new(p.PE4, Level::High, Speed::Low);
    sensor_ring_spi_cs.set_high();
/*    // Carte Ticket (Center sensor) is 3V3-powered and runs on SPI6 (J4)
    let mut sensor_ring_spi_config = spi::Config::default();
    sensor_ring_spi_config.frequency = mhz(1); // 10 MHz max clk
    sensor_ring_spi_config.mode = spi::MODE_1;
    let mut sensor_ring_spi = spi::Spi::new(p.SPI6, p.PB3, p.PB5, p.PB4, NoDma, NoDma, sensor_ring_spi_config);
    let mut sensor_ring_spi_cs = Output::new(p.PA15, Level::High, Speed::Low);
    sensor_ring_spi_cs.set_high();
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
*/
    info!("----------------- Main Loop -----------------");
    loop {
        // Blinking
        led_green.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led_green.set_low();

        // SPI
        // RLS Aksim2
        let mut data_read = [0x00u8, 0x00u8, 0x00u8, 0x00u8];
/*        let mut data_b31_b24 = 0x00u8;
        let mut data_b23_b16 = 0x00u8;
        let mut data_b15_b8  = 0x00u8;
        let mut data_b7_b0   = 0x00u8;*/
        
        sensor_ring_spi_cs.set_low();
        let result = sensor_ring_spi.blocking_read(&mut data_read);
        if let Err(_) = result {
            defmt::panic!("crap read");
        }
        sensor_ring_spi_cs.set_high();
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
        encoder_position = encoder_position >> 13; // 19 + 13 = 31 (MSB position of data)
        // Nota: 2^19 = 524288

        let angle_range = 360.0;
        let angle = (encoder_position as f64 / 524288.0) * angle_range;
        info!("Angle: {} degrees", angle);

/*        // Carte Ticket
        sensor_ring_spi_cs.set_low();
//        let data_write = [0b11000000u8, 0b00000000u8]; // read nop
//        let data_write = [0xffu8, 0xfcu8]; // read diag
        let data_write = [0x7fu8, 0xfeu8]; // read angle
        let result = sensor_ring_spi.blocking_write(&data_write);
        if let Err(_) = result {
            defmt::panic!("crap write");
        }
        sensor_ring_spi_cs.set_high();
        Timer::after(Duration::from_micros(1)).await; // actually > 350 ns
        sensor_ring_spi_cs.set_low();
        let mut data_read = [0x00u8, 0x00u8];
        let result = sensor_ring_spi.blocking_read(&mut data_read);
        if let Err(_) = result {
            defmt::panic!("crap read");
        }
        sensor_ring_spi_cs.set_high();
//        info!("read via spi: {:#02x} {:#02x}.", &data_read[0], &data_read[1]);
//        info!("read via spi: {:#010b} {:#010b}.", &data_read[0], &data_read[1]);

        // Combine the two u8 values into a 16-bit integer
        let mut combined_value: u16 = ((data_read[0] as u16) << 8) | (data_read[1] as u16);
        combined_value &= 0x3FFF;
        let angle_range = 360.0;
        let angle = (combined_value as f64 / 16383.0) * angle_range;
        info!("Angle: {} degrees", angle);
*/

        Timer::after(Duration::from_millis(1000)).await;
    }
}
