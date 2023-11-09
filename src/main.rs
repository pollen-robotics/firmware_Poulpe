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
use storage::Storage;

mod config;
mod dynamixel;
mod motor_control;
mod shared_memory;
mod storage;

use crate::motor_control::{Actuator, VentouseKind};
use crate::shared_memory::SharedMemory;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

// TODO: Use a NoopMutex instead of a real mutex?
static SHARED_MEMORY: Mutex<ThreadModeRawMutex, SharedMemory<{ config::N_AXIS }>> =
    Mutex::new(SharedMemory::default());
static PERMANENT_STORAGE: Mutex<ThreadModeRawMutex, Storage> = Mutex::new(Storage::default());

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
    PERMANENT_STORAGE.lock().await.init(p.FLASH);

    let id = PERMANENT_STORAGE.lock().await.get_id();
    info!("Use dynamixel id: {}", id);

    // Setup the actuator with the configured ventouses
    #[cfg(feature = "orbita2d")]
    let mut actuator = Actuator::new([
        VentouseKind::B(config::VentouseB::new(
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
        VentouseKind::C(config::VentouseC::new(
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
    #[cfg(feature = "orbita3d")]
    let mut actuator = Actuator::new([
        VentouseKind::A(config::VentouseA::new(
            motor_control::VentouseConfig {
                cs_foc: p.PA3,
                cs_driver: p.PA2,
                peri: p.SPI1,
                sck: p.PA5,
                mosi: p.PA7,
                miso: p.PA6,
                foc_enable: p.PC0,
                foc_status: p.PA0,
                driver_fault: p.PA1,
            },
            config::BrushlessMotor::ecx22(),
        )),
        VentouseKind::B(config::VentouseB::new(
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
        VentouseKind::C(config::VentouseC::new(
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
