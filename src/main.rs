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

mod tmc6200;
mod tmc4671;
mod ventouse;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let config = Config::default();
    let p = embassy_stm32::init(config);

    let mut led_hello = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_error = Output::new(p.PC8, Level::High, Speed::Low);
    led_error.set_low();
    led_hello.set_low();

    let mut ventouse = ventouse::Ventouse::new(p.PE3, p.PC15, p.PE12, p.PE5, p.PE6, p.SPI4, NoDma, NoDma);

    // TMC6200 init
    let mut write_b = true;
    let mut reg_addr = 0x00u8;
    let mut data = 0x00000000u32;
    let res = ventouse.tmc6200_transmit_raw_data(write_b, reg_addr, &mut data);
    if let Err(_) = res {
        defmt::panic!("crap_from_fn!");
    }
    info!("write[{:#04x}]: {:#04x}", reg_addr, res.unwrap());
    Timer::after(Duration::from_millis(1000)).await;

    // TMC4671 init
    /*write_b = true;
    reg_addr = 0x00u8;
    data = 0x00000000u32;
    ventouse.tmc4671_transmit_raw_data(write_b, reg_addr, &mut data).unwrap();
    Timer::after(Duration::from_millis(10)).await;*/
    ventouse.tmc4671_init();
    ventouse.tmc4671_set_mode(ventouse::MotionMode::Velocity);
    ventouse.tmc4671_set_target_velocity(2000);
    info!("Velocity_target: {:?}", ventouse.tmc4671_get_target_velocity().unwrap());

    loop {
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;

        /*write_b = false;
        //reg_addr = ventouse::Tmc4671Registers::CHIPINFO_DATA as u8;
        //reg_addr = ventouse::Tmc4671Registers::ABN_DECODER_PPR as u8;
        //reg_addr = ventouse::Tmc4671Registers::ABN_DECODER_COUNT as u8;
        reg_addr = ventouse::Tmc4671Registers::MOTOR_TYPE_N_POLE_PAIRS as u8;
        let res = ventouse.tmc4671_transmit_raw_data(write_b, reg_addr, &mut data);
        if let Err(_) = res {
            defmt::panic!("crap_from_fn!");
        }
        info!("read[{:#04x}]: {:#04x}", reg_addr, res.unwrap());*/

        info!("Velocity_actual: {:?}", ventouse.tmc4671_get_actual_velocity().unwrap());

        led_hello.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
