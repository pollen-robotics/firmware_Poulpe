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
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::usart::Config as usart_config;
use embassy_stm32::Config as stm32_config;
use embassy_stm32::{bind_interrupts, peripherals, i2c, usart};
use embassy_stm32::dma::NoDma;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer, block_for};
use embassy_stm32::time::Hertz;

mod config;
mod dynamixel;
mod motor_control;
mod shared_memory;

use crate::config::{ActuatorConfig, AksimConfig, AD5047Config, AD5047ConfigTop, AD5047ConfigMid, AD5047ConfigBot};
use crate::motor_control::ventouse::VentouseConfig;
use crate::shared_memory::SharedMemory;

use {defmt_rtt as _, panic_probe as _};

const ADDRESS_A: u8 = 0x38;
const ADDRESS_B: u8 = 0x39;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});
bind_interrupts!(struct IrqsI2c {
    I2C1_EV => i2c::InterruptHandler<peripherals::I2C1>;
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

    // 440MHz (without HSE)
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

    let mut led_green = Output::new(p.PC9, Level::High, Speed::Low);
    led_green.set_low();
    let mut led_red = Output::new(p.PC8, Level::High, Speed::Low);
    led_red.set_high();

    let mut i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        IrqsI2c,
        NoDma,//p.DMA1_CH4,
        NoDma,//p.DMA1_CH5,
        Hertz(100_000),
        Default::default(),
    );

    let mut data = [0u8; 1];
    let mut hall_detected = 0u16;

    led_red.set_low();

    loop {

    led_green.set_high();

    match i2c.blocking_read(ADDRESS_A, &mut data) {
        Ok(()) => {
            //info!("Inputs_A: {:#010b}", data[0]);
//            hall_detected = (data[0] as u16) << 8;
            hall_detected = data[0] as u16;
        },
        Err(Error::Timeout) => info!("Operation timed out"),
        Err(e) => info!("I2c Error: {:?}", e),
    }

    match i2c.blocking_read(ADDRESS_B, &mut data) {
        Ok(()) => {
            //info!("Inputs_B: {:#010b}", data[0]);
//            hall_detected = hall_detected | (data[0] as u16);
            hall_detected = hall_detected | ((data[0] as u16) << 8);
        },
        Err(Error::Timeout) => info!("Operation timed out"),
        Err(e) => info!("I2c Error: {:?}", e),
    }

    info!("Halls: {:#018b}", !hall_detected);

/*    match timeout_i2c.blocking_write_read(ADDRESS, &[IO_REG], &mut data) {
        Ok(()) => info!("Inputs: {}", data[0]),
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }*/

    Timer::after(Duration::from_millis(50)).await;
    //led_green.set_low();
    //Timer::after(Duration::from_millis(450)).await;
    }

    // Spawn the control loop
    /*#[cfg(feature = "orbita3d")]
    let actuator_config = ActuatorConfig {

        a: VentouseConfig {
            peri: p.SPI1,
            sck: p.PA5,
            mosi: p.PA7,
            miso: p.PA6,
            foc_cs: p.PA3,
            foc_enable: p.PC0,
            driver_cs: p.PA2,
        },
        b: VentouseConfig {
            peri: p.SPI4,
            sck: p.PE12,
            mosi: p.PE6,
            miso: p.PE5,
            foc_cs: p.PE3,
            foc_enable: p.PE0,
            driver_cs: p.PC15,
        },
        c: VentouseConfig {
            peri: p.SPI6,
            sck: p.PB3,
            mosi: p.PB5,
            miso: p.PB4,
            foc_cs: p.PD7,
            foc_enable: p.PD5,
            driver_cs: p.PD6,
        },

        ad5047top: AD5047ConfigTop {
            cs: p.PA4,
        },
        ad5047mid: AD5047ConfigMid {
            cs: p.PE4,
        },
        ad5047bot: AD5047ConfigBot {
            cs: p.PA15,
        },

    };
    #[cfg(feature = "orbita2d")]
    let actuator_config = ActuatorConfig {

        b: VentouseConfig {
            peri: p.SPI4,
            sck: p.PE12,
            mosi: p.PE6,
            miso: p.PE5,
            foc_cs: p.PE3,
            foc_enable: p.PE0,
            driver_cs: p.PC15,
        },
        c: VentouseConfig {
            peri: p.SPI6,
            sck: p.PB3,
            mosi: p.PB5,
            miso: p.PB4,
            foc_cs: p.PD7,
            foc_enable: p.PD5,
            driver_cs: p.PD6,
        },

        aksim: AksimConfig {
            cs: p.PE4,
        },
        ad5047: AD5047Config {
            cs: p.PA15,
        },

    };


    unwrap!(spawner.spawn(motor_control::task::control_loop(actuator_config)));

    // Prepare and spawn the DXL communication task
    let mut usart_config = usart_config::default();
    usart_config.baudrate = 1_000_000;
    // usart_config.baudrate = 115_200
    // usart_config.baudrate = 2_000_000;
    usart_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    usart_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    usart_config.parity = embassy_stm32::usart::Parity::ParityNone;
    usart_config.detect_previous_overrun = false;

    //Poule A1
    // let usart = config::DynamixelUart::new(
    //     p.USART1,
    //     p.PB15, //RX
    //     p.PB14, //TX
    //     Irqs,
    //     p.DMA1_CH0,
    //     p.DMA1_CH1,
    //     usart_config,
    // )
    // .unwrap();
    // unwrap!(spawner.spawn(dynamixel::task::messsage_handler(usart, p.PD9.into())));


    // Poulpe B1
    let usart = config::DynamixelUart::new(
        p.USART1,
        p.PB15, //RX
        p.PA9, //TX
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
    }*/
}
