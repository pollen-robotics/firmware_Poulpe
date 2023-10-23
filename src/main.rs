#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{spi, Config};
use embassy_time::{Duration, Timer};
use futures::TryFutureExt;
use {defmt_rtt as _, panic_probe as _};

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

    let mut ventouse = ventouse::Ventouse::new(
        p.PE3, p.PC15, p.PE12, p.PE5, p.PE6, p.SPI4, NoDma, NoDma, p.PE0, p.PC13, p.PC14,
    );

    // Tuning mode: uncomment to set Poulpe and Ventouse ready for tuning
    /*    info!("TMC6200 -> 6-PWM mode {:?}", ventouse.tmc6200_checked_write(0x00u8, 0x00000000u32));
    ventouse.tmc4671_enable();
    loop {
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led_hello.set_low();
        Timer::after(Duration::from_millis(1500)).await;
    }*/

    // TMC4671 init
    // ventouse.tmc4671_enable();
    // let _ = ventouse.tmc6200_checked_write(0x00u8, 0x00000000u32);
    ventouse.tmc4671_init_registers().await.unwrap();
    ventouse.tmc4671_align_motor().await.unwrap();

    // ventouse.tmc4671_set_mode(ventouse::MotionMode::Velocity);
    // ventouse.tmc4671_set_target_velocity(2000);

    loop {
        // info!(
        //     "Velocity_actual: {:?} [{:#04x}]",
        //     ventouse.tmc4671_get_actual_velocity().unwrap(),
        //     ventouse.tmc4671_get_mode().unwrap()
        // );

        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led_hello.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
