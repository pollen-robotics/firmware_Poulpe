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
use embassy_stm32::Config as stm32_config;
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

mod config;
mod dynamixel;
mod motor_control;
mod shared_memory;

use crate::motor_control::{Actuator, VentouseKind};
use crate::shared_memory::SharedMemory;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

// TODO: Use a NoopMutex instead of a real mutex?
static SHARED_MEMORY: Mutex<ThreadModeRawMutex, SharedMemory<{ config::N_AXIS }>> =
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let stm32_conf = stm32_config::default();
    let p = embassy_stm32::init(stm32_conf);

    // Setup the actuator with the configured ventouses
    let mut actuator = Actuator::new([
        VentouseKind::A(config::VentouseA::new(
            motor_control::VentouseConfig {
                cs_foc: p.PE3,
                cs_driver: p.PC15,
                peri: p.SPI4,
                sck: p.PE12,
                mosi: p.PE6,
                miso: p.PE5,
                foc_enable: p.PE0,
                foc_status: p.PC13,
                driver_fault: p.PC14,
            },
            config::BrushlessMotor::ecx22(),
        )),
        VentouseKind::B(config::VentouseB::new(
            motor_control::VentouseConfig {
                cs_foc: p.PD7,
                cs_driver: p.PD6,
                peri: p.SPI6,
                sck: p.PB3,
                mosi: p.PB5,
                miso: p.PB4,
                foc_enable: p.PD5,
                foc_status: p.PD4,
                driver_fault: p.PD3,
            },
            config::BrushlessMotor::ecx22(),
        )),
    ]);

    // Init SharedMemory with real values before actually running the control loop
    SHARED_MEMORY.lock().await.init(&mut actuator);

    // Spawn the control loop
    unwrap!(spawner.spawn(motor_control::task::control_loop(actuator)));

    // Prepare and spawn the DXL communication task
    let mut usart_config = usart_config::default();
    usart_config.baudrate = 1_000_000;
    usart_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    usart_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    usart_config.parity = embassy_stm32::usart::Parity::ParityNone;
    usart_config.detect_previous_overrun = false;

    let usart = config::DynamixelUart::new(
        p.USART1,
        p.PB15,
        p.PB14,
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        usart_config,
    )
    .unwrap();
    unwrap!(spawner.spawn(dynamixel::task::messsage_handler(usart, p.PD9.into())));

    // Prepare and spawn the main task
    let mut led_hello = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_error = Output::new(p.PC8, Level::High, Speed::Low);
    led_error.set_low();
    led_hello.set_low();

    loop {
        // Robots should dance, LED should blink.
        led_hello.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led_hello.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
