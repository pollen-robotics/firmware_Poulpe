#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(async_fn_in_trait)]
#![feature(array_methods)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::usart::Config as usart_config;
use embassy_stm32::Config as stm32_config;
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer};

mod config;
mod dynamixel;
mod motor_control;

use crate::dynamixel::{InstructionPacketKind, StatusPacket};
use crate::motor_control::{Actuator, VentouseKind};

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
async fn dxl_serial(usart: config::DynamixelUart, dir_pin: AnyPin) {
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

const N_AXIS: usize = 2;

#[embassy_executor::task]
async fn control_loop(actuator: Actuator<N_AXIS>) {
    let mut actuator = actuator;

    actuator.init().await;

    loop {
        actuator.get_actual_position().unwrap();
        actuator.set_target_position([0, 0]).unwrap();
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    //440MHz (without HSE)
    let mut stm32_conf = stm32_config::default();
    {
        use embassy_stm32::rcc::*;
        stm32_conf.rcc.hsi = Some(HSIPrescaler::DIV1); //HSIState = RCC_HSI_DIV1
        stm32_conf.rcc.csi = true; //CSIState = RCC_CSI_ON;
                                   // stm32_conf.rcc.hse = Som(Hse{Hertz::mhz(48), HseMode::Oscillator}); //TODO
        stm32_conf.rcc.pll1 = Some(Pll {
            // source: PllSource::HSI
            source: PllSource::CSI,   //PLLSource = RCC_PLLSOURCE_CSI
            prediv: PllPreDiv::DIV1,  //PLLM = 1;
            mul: PllMul::MUL220,      //PLLN = 220
            divp: Some(PllDiv::DIV2), //PLLP = 2;
            divq: Some(PllDiv::DIV5), //PLLQ = 5;
            divr: Some(PllDiv::DIV5), //PLLR = 5;
        });
        stm32_conf.rcc.sys = Sysclk::PLL1_P; // 440 Mhz
        stm32_conf.rcc.ahb_pre = AHBPrescaler::DIV2; // 220 Mhz
        stm32_conf.rcc.apb1_pre = APBPrescaler::DIV2; // 110 Mhz
        stm32_conf.rcc.apb2_pre = APBPrescaler::DIV2; // 110 Mhz
        stm32_conf.rcc.apb3_pre = APBPrescaler::DIV2; // 110 Mhz
        stm32_conf.rcc.apb4_pre = APBPrescaler::DIV2; // 110 Mhz
        stm32_conf.rcc.voltage_scale = VoltageScale::Scale0;
    }

    let p = embassy_stm32::init(stm32_conf);

    // Prepare and spawn the DXL serial task
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

    unwrap!(spawner.spawn(dxl_serial(usart, p.PD9.into())));

    // Prepare and spawn the ventouse task
    let orbita_2d = Actuator::new([
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
    unwrap!(spawner.spawn(control_loop(orbita_2d)));

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
