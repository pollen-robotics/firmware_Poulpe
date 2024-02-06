#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(async_fn_in_trait)]
#![feature(array_methods)]

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::Config as usart_config;
use embassy_stm32::{Config as stm32_config, i2c};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_stm32::dma::NoDma;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer, block_for};

pub mod config;
pub mod dynamixel;
pub mod motor_control;
pub mod shared_memory;
use config::{ActuatorConfig, AksimConfig, AD5047Config, AD5047ConfigTop, AD5047ConfigMid, AD5047ConfigBot};
use motor_control::sensors::I2cHallConfig;
use motor_control::ventouse::VentouseConfig;
use shared_memory::SharedMemory;


bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});
bind_interrupts!(struct IrqsI2c {
    I2C1_EV => i2c::InterruptHandler<peripherals::I2C1>;
});

use {defmt_rtt as _, panic_probe as _}; // global logger and panicking behavior

// TODO: Use a NoopMutex instead of a real mutex?
pub static SHARED_MEMORY: Mutex<ThreadModeRawMutex, SharedMemory<{ config::N_AXIS }>> =
    Mutex::new(SharedMemory::default());

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}