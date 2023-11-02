#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH1, USART1};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::Config as stm32_config;
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer};
use paste::paste;

// declare the modules
mod config;
mod dynamixel;
mod registers;
mod ventouse;

use crate::dynamixel::{InstructionPacketKind, StatusPacket};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

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

#[embassy_executor::task]
async fn dxl_serial(usart: Uart<'static, USART1, DMA1_CH0, DMA1_CH1>, dir_pin: AnyPin) {
    let id = config::DXL_ID;
    let mut dxl = dynamixel::DynamixelUsartIO::new(usart, dir_pin, id);

    let dxl_error = 0;

    loop {
        debug!("Waiting for packet...");
        match dxl.read().await {
            Ok(packet) => {
                debug!("Got packet: {:?}", packet);

                match packet {
                    InstructionPacketKind::Ping(_) => {
                        let sp = StatusPacket::ack(id, dxl_error);
                        debug!("Sending status packet: {:?}", sp);
                        if let Some(e) = dxl.write(&sp).await.err() {
                            error!("Error: {:?}", e);
                        }
                    }
                    InstructionPacketKind::ReadData(_read_data_packet) => {
                        // let value: [u8; N] = register.get_data(read_data_packet.address, read_data_packet.data_length).unwrap();
                        let value = [0, 42, 0, 10];

                        let sp = StatusPacket::with_value(id, dxl_error, value);
                        debug!("Sending status packet: {:?}", sp);
                        if let Some(e) = dxl.write(&sp).await.err() {
                            error!("Error: {:?}", e);
                        }
                    }
                    InstructionPacketKind::WriteData(_write_data_packet) => {
                        // register.set_data(write_data_packet.address, write_data_packet.data).unwrap();

                        let sp = StatusPacket::ack(id, dxl_error);
                        debug!("Sending status packet: {:?}", sp);
                        if let Some(e) = dxl.write(&sp).await.err() {
                            error!("Error: {:?}", e);
                        }
                    }
                }
            }
            Err(e) => {
                error!("Error: {:?}", e);
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let stm32_conf = stm32_config::default();
    let p = embassy_stm32::init(stm32_conf);

    // Prepare and spawn the DXL serial task
    let mut config = Config::default();
    config.baudrate = 1_000_000;
    config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    config.parity = embassy_stm32::usart::Parity::ParityNone;
    config.detect_previous_overrun = false;

    let usart = Uart::new(
        p.USART1, p.PB15, p.PB14, Irqs, p.DMA1_CH0, p.DMA1_CH1, config,
    )
    .unwrap();

    unwrap!(spawner.spawn(dxl_serial(usart, p.PD9.into())));

    // Prepare and spawn the ventouse task
    let mut ventouse = ventouse::Ventouse::new(
        p.PE3, p.PC15, p.PE12, p.PE5, p.PE6, p.SPI4, NoDma, NoDma, p.PE0, p.PC13, p.PC14,
    );
    ventouse.tmc4671_init_registers().await.unwrap();
    info!("TMC4671 init done");
    ventouse.tmc4671_align_motor().await.unwrap();
    info!("Motor align done");
    let curpos = ventouse.tmc4671_get_actual_position().unwrap();
    info!("Current position: {:?}", curpos);
    ventouse
        .tmc4671_set_mode(ventouse::MotionMode::Position)
        .unwrap();
    ventouse.tmc4671_set_target_position(curpos).unwrap();
    Timer::after(Duration::from_millis(1000)).await;

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
