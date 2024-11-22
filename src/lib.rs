#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(async_fn_in_trait)]
#![feature(array_methods)]

pub mod config;
pub mod dynamixel;
pub mod ethercat;
pub mod motor_control;
pub mod state_machine;
pub mod sensors;
pub mod utils;

pub mod shared_memory;
pub use shared_memory::SharedMemory;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

// TODO: Use a NoopMutex instead of a real mutex?
pub static SHARED_MEMORY: Mutex<ThreadModeRawMutex, SharedMemory<{ config::N_AXIS }>> =
    Mutex::new(SharedMemory::default());




use embassy_stm32::{bind_interrupts, peripherals, usart, i2c};

bind_interrupts!(pub struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});
bind_interrupts!(pub struct IrqsI2c {
    I2C1_EV => i2c::InterruptHandler<peripherals::I2C1>;
});


use {defmt_rtt as _, panic_probe as _};
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
    