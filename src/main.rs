#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH1, USART1};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
// use embassy_sync::channel::Channel;
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
// use embassy_sync::mutex::Mutex;
use embassy_stm32::time::mhz;
use embassy_stm32::Config as stm32_config;
use embassy_time::{Duration, Timer};
// declare the modules
mod config;
mod dynamixel;
mod registers;

mod ventouse;
use paste::paste;

use {defmt_rtt as _, panic_probe as _};
//seems ok at 115200 with LOG=Debug
// const UART_SLEEP_US_DIRLOW: u64 = 10;
// const UART_SLEEP_US_DIRHIGH: u64 = 1;

//Seems ok at 115200 with LOG=Info
const UART_SLEEP_US_DIRLOW: u64 = 200;
const UART_SLEEP_US_DIRHIGH: u64 = 300;

// const UART_SLEEP_US_DIRLOW: u64 = 400;
// const UART_SLEEP_US_DIRHIGH: u64 = 600;

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

// static RX_CHANNEL: Channel<ThreadModeRawMutex, [u8; 6], 1> = Channel::new();
// static TX_CHANNEL: Channel<ThreadModeRawMutex, [u8; 6], 1> = Channel::new();

#[embassy_executor::task]
async fn test_reg() {
    let i: u8 = 0;
    let j: u8 = 1;
    let k: u8 = 0;

    loop {
        {
            // let mut registers = registers::REGISTERS.lock().await;
            // registers.buffer[0] += 1;
            let _ = crate::config::dxl_registers_write_by_address(0, 2, &[i, j]).await;
            let _ = crate::config::dxl_registers_write_by_name(
                config::DxlRegistersEnum::SensorCenterPresentPosition,
                &[0, 0, 0, k],
            )
            .await;
        }
        let _ = i.wrapping_add(1);
        let _ = j.wrapping_add(1);
        let _ = k.wrapping_add(1);

        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn dxl_serial(mut usart: Uart<'static, USART1, DMA1_CH0, DMA1_CH1>, dir_pin: AnyPin) {
    //How can I avoid passing this specific type?
    let mut dxlcom = dynamixel::DxlCom::new(config::DXL_ID); //TODO read/write ID from flash
    let mut dir = Output::new(dir_pin, Level::Low, Speed::High);

    let mut buf: [u8; dynamixel::MAX_BUFFER_LENGTH] = [0; dynamixel::MAX_BUFFER_LENGTH];

    dir.set_low(); //reading

    loop {
        let res = usart.read_until_idle(&mut buf).await;

        match res {
            Ok(nb) => {
                trace!("received {:?} bytes: {:?}", nb, buf[0..nb]);
                let action = dxlcom.parse(&buf[0..nb]).await;
                match action {
                    Ok(dynamixel::RWAction::Ignore) => {
                        trace!("Ignoring");
                    }
                    Ok(dynamixel::RWAction::Ok) => {
                        trace!("Done");
                    }
                    Ok(dynamixel::RWAction::Tx(data)) => {
                        trace!("Sending response: {:?}", data);
                        dir.set_high(); //writing mode
                        Timer::after(Duration::from_micros(UART_SLEEP_US_DIRHIGH)).await;
                        match usart.write(data).await {
                            Ok(()) => {
                                //good
                                trace!("Sent: {:?}", data);
                            }
                            Err(_) => {
                                error!("Error sending response");
                            }
                        }
                        Timer::after(Duration::from_micros(UART_SLEEP_US_DIRLOW)).await; //Very important sleep here
                        dir.set_low(); //reading mode
                                       // Timer::after(Duration::from_micros(UART_SLEEP_US)).await;
                    }

                    Err(_err) => {
                        match _err {
                            dynamixel::Error::BadCRC => {
                                error!("Error bad crc received");

                                dir.set_high(); //writing mode
                                Timer::after(Duration::from_micros(UART_SLEEP_US_DIRHIGH)).await;
                                match usart.write(&dxlcom.get_status_packet().await).await {
                                    Ok(()) => {
                                        //good
                                        // debug!("Sent: {:?}", data);
                                    }
                                    Err(_) => {
                                        error!("Error sending response");
                                    }
                                }
                                Timer::after(Duration::from_micros(UART_SLEEP_US_DIRLOW)).await;
                                dir.set_low(); //reading mode
                                               // Timer::after(Duration::from_micros(UART_SLEEP_US)).await;
                            }
                            dynamixel::Error::BadInstruction => {
                                error!("Error bad instruction received");
                                dir.set_high(); //writing mode
                                Timer::after(Duration::from_micros(UART_SLEEP_US_DIRHIGH)).await;
                                match usart.write(&dxlcom.get_status_packet().await).await {
                                    Ok(()) => {
                                        //good
                                        // debug!("Sent: {:?}", data);
                                    }
                                    Err(_) => {
                                        error!("Error sending response");
                                    }
                                }
                                Timer::after(Duration::from_micros(UART_SLEEP_US_DIRLOW)).await;
                                dir.set_low(); //reading mode
                                               // Timer::after(Duration::from_micros(UART_SLEEP_US)).await;
                            }
                            dynamixel::Error::BadPacket => {
                                error!("Error bad packet received");
                                dir.set_high(); //writing mode
                                Timer::after(Duration::from_micros(UART_SLEEP_US_DIRHIGH)).await;
                                match usart.write(&dxlcom.get_status_packet().await).await {
                                    Ok(()) => {
                                        //good
                                        // debug!("Sent: {:?}", data);
                                    }
                                    Err(_) => {
                                        error!("Error sending response");
                                    }
                                }
                                Timer::after(Duration::from_micros(UART_SLEEP_US_DIRLOW)).await;
                                dir.set_low(); //reading mode
                                               // Timer::after(Duration::from_micros(UART_SLEEP_US)).await;
                            }
                        }
                        // error!("Action Error??"); //TODO
                    }
                }
            }
            Err(err) => {
                error!("ERROR {:?}", err);
            }
        }
    }
}

// static EXECUTOR: StaticCell<Executor> = StaticCell::new();

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

    let mut led_hello = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_error = Output::new(p.PC8, Level::High, Speed::Low);
    led_error.set_low();
    led_hello.set_low();

    let mut ventouse = ventouse::Ventouse::new(
        p.PE3, p.PC15, p.PE12, p.PE5, p.PE6, p.SPI4, NoDma, NoDma, p.PE0, p.PC13, p.PC14,
    );

    let mut config = Config::default();
    config.baudrate = 1_000_000;
    // config.baudrate = 115200;
    // config.baudrate = 9600;

    config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    config.parity = embassy_stm32::usart::Parity::ParityNone;

    config.detect_previous_overrun = false;

    //more "recent" version of embassy returns a Result here
    // let usart = Uart::new(
    //     p.USART1, p.PB15, p.PB14, Irqs, p.DMA1_CH0, p.DMA1_CH1, config,
    // )
    // .unwrap();

    let usart = Uart::new(
        p.USART1, p.PB15, p.PB14, Irqs, p.DMA1_CH0, p.DMA1_CH1, config,
    );

    unwrap!(spawner.spawn(dxl_serial(usart, p.PD9.into())));
    // TMC4671 init
    // ventouse.tmc4671_enable();
    // let _ = ventouse.tmc6200_checked_write(0x00u8, 0x00000000u32);
    ventouse.tmc4671_init_registers().await.unwrap();
    info!("TMC4671 init done");
    ventouse.tmc4671_align_motor().await.unwrap();
    info!("Motor align done");

    // ventouse.tmc4671_set_mode(ventouse::MotionMode::Velocity);
    // ventouse.tmc4671_set_target_velocity(2000);
    // unwrap!(spawner.spawn(test_reg()));

    let curpos = ventouse.tmc4671_get_actual_position().unwrap();
    info!("Current position: {:?}", curpos);
    /*
    ventouse.tmc4671_set_mode(ventouse::MotionMode::Position);
    ventouse.tmc4671_set_target_position(curpos);
    Timer::after(Duration::from_millis(1000)).await;
    ventouse.tmc4671_set_target_position(curpos + 1000000);
    Timer::after(Duration::from_millis(1000)).await;
    let curpos = ventouse.tmc4671_get_actual_position().unwrap();
    info!("Current position: {:?}", curpos);
     */

    ventouse.tmc4671_set_mode(ventouse::MotionMode::Position);
    ventouse.tmc4671_set_target_position(curpos);
    Timer::after(Duration::from_millis(1000)).await;

    loop {
        /*
            led_hello.set_high();
            Timer::after(Duration::from_millis(500)).await;
            led_hello.set_low();
            Timer::after(Duration::from_millis(500)).await;
        */

        //TEST DXL COM
        //Get register value

        let mut gpos: &mut [u8; 4] = &mut [0, 0, 0, 0];
        let _ = crate::config::dxl_registers_read_by_name(
            config::DxlRegistersEnum::MotorAGoalPosition,
            gpos,
        )
        .await;

        let gposf: i32 = i32::from_le_bytes(*gpos);
        ventouse.tmc4671_set_target_position(curpos + gposf);
        let curpos = ventouse.tmc4671_get_actual_position().unwrap();
        let _ = crate::config::dxl_registers_write_by_name(
            config::DxlRegistersEnum::MotorAPresentPosition,
            &curpos.to_le_bytes(),
        )
        .await;
        // info!("goal pos: {:?} actual pos: {:?}", gposf, curpos);

        // Timer::after(Duration::from_millis(1)).await;
        Timer::after(Duration::from_micros(1000)).await;
    }
}

/*
#[cfg(test)]
#[defmt_test::tests]
mod tests {
    use super::*;
    // #[init]
    // fn init() {}

    use defmt::{assert, assert_eq};

    #[test]
    fn test_it_works() {
        info!("TEST");
        assert!(true)
    }
    #[test]
    fn test_42() {
        let a: u8 = 42;
        assert_eq!(a, 42)
    }
}
*/
