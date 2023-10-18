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
        p.PE3,
        p.PC15,
        p.PE12,
        p.PE5,
        p.PE6,
        p.SPI4,
        NoDma,
        NoDma,
        p.PE0,
        p.PC13,
        p.PC14
    );

    // TMC6200 init ("Single-line mode" aka 6-PWM)
    /*let write_b = true;
    let reg_addr = 0x00u8;
    let mut data = 0x00000000u32;
    let res = ventouse.tmc6200_transmit_raw_data(write_b, reg_addr, &mut data);
    if let Err(_) = res {
        defmt::panic!("crap_from_fn!");
    }
    info!("Drive_mode: {:#x}", ventouse.tmc6200_transmit_raw_data(false, reg_addr, &mut data).unwrap());
    Timer::after(Duration::from_millis(10)).await;*/
    info!("TMC6200 -> 6-PWM mode {:?}", ventouse.tmc6200_checked_write(0x00u8, 0x00000000u32));
ventouse.tmc4671_enable();
loop {
/*    info!("TMC6200 reg_0x00: {:#010x}", ventouse.tmc6200_read_register(0x00));
    info!("TMC6200 reg_0x01: {:#010x}", ventouse.tmc6200_read_register(0x01));
    info!("TMC6200 reg_0x04: {:#010x}", ventouse.tmc6200_read_register(0x04));
    info!("TMC6200 reg_0x06: {:#010x}", ventouse.tmc6200_read_register(0x06));
    info!("TMC6200 reg_0x07: {:#010x}", ventouse.tmc6200_read_register(0x07));
    info!("TMC6200 reg_0x08: {:#010x}", ventouse.tmc6200_read_register(0x08));
    info!("TMC6200 reg_0x09: {:#010x}", ventouse.tmc6200_read_register(0x09));
    info!("TMC6200 reg_0x0A: {:#010x}", ventouse.tmc6200_read_register(0x0A));
    Timer::after(Duration::from_millis(500)).await;*/
}

    // TMC4671 init
info!("UD_UQ_LIMITS: {:#x}", ventouse.tmc4671_transmit_raw_data(false, 0x5Du8, 0x00000000u32).unwrap());
info!("ADC_VM_LIMITS: {:#x}", ventouse.tmc4671_transmit_raw_data(false, 0x75u8, 0x00000000u32).unwrap());
info!("status_flags: {:#x}", ventouse.tmc4671_transmit_raw_data(false, 0x7Cu8, 0x00000000u32).unwrap());
    ventouse.tmc4671_init().await;
info!("status_flags: {:#x}", ventouse.tmc4671_transmit_raw_data(false, 0x7Cu8, 0x00000000u32).unwrap());
    ventouse.tmc4671_set_mode(ventouse::MotionMode::Stopped);
    ventouse.tmc4671_enable();

    ventouse.tmc4671_set_mode(ventouse::MotionMode::Torque);
    ventouse.tmc4671_set_torque_target(1000);
    ventouse.tmc4671_set_flux_target(1001);

//    ventouse.tmc4671_set_mode(ventouse::MotionMode::Velocity);
//    ventouse.tmc4671_set_target_velocity(20000);
//    info!("Velocity_target: {:?}", ventouse.tmc4671_get_target_velocity().unwrap());

    loop {
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;

//        info!("Velocity_actual: {:?} [{:#04x}]", ventouse.tmc4671_get_actual_velocity().unwrap(), ventouse.tmc4671_get_mode().unwrap());
//        info!("Position_actual: {:?}", ventouse.tmc4671_get_actual_position().unwrap());
//        ventouse.tmc4671_set_encoder_ppr(4096);
//        info!("encoder_ppr: {:?}", ventouse.tmc4671_get_encoder_ppr().unwrap());
//        info!("encoder_actual: {:?}", ventouse.tmc4671_get_encoder_count().unwrap());
        info!("Target/Actual: {:?}/{:?} Flux, {:?}/{:?} Torque - mode/status: [{:#04x}]/[{:#010x}]", 
            ventouse.tmc4671_get_flux_target().unwrap(), 
            ventouse.tmc4671_get_flux_actual().unwrap(), 
            ventouse.tmc4671_get_torque_target().unwrap(), 
            ventouse.tmc4671_get_torque_actual().unwrap(),
            ventouse.tmc4671_get_mode().unwrap(),
            ventouse.tmc4671_transmit_raw_data(false, 0x7Cu8, 0x00000000u32).unwrap()
        );
//        info!("torque_flux_reg: {:#x}", ventouse.tmc4671_transmit_raw_data(false, 0x64u8, 0x00000000u32));
//        info!("status_flags: {:#x}", ventouse.tmc4671_transmit_raw_data(false, 0x7Cu8, 0x00000000u32).unwrap());


        led_hello.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
