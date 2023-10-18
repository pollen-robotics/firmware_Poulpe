#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
// use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH1, USART1};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
// use embassy_sync::channel::Channel;
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
// use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

// declare the modules
mod config;
mod dynamixel;
mod registers;

use paste::paste;

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
                debug!("received {:?} bytes: {:?}", nb, buf[0..nb]);
                let action = dxlcom.parse(&buf[0..nb]).await;
                match action {
                    Ok(dynamixel::RWAction::Ignore) => {
                        debug!("Ignoring");
                    }
                    Ok(dynamixel::RWAction::Ok) => {
                        debug!("Done");
                    }
                    Ok(dynamixel::RWAction::Tx(data)) => {
                        debug!("Sending response: {:?}", data);
                        dir.set_high(); //writing mode
                                        // Timer::after(Duration::from_micros(500)).await;
                        match usart.write(data).await {
                            Ok(()) => {
                                //good
                            }
                            Err(_) => {
                                error!("Error sending response");
                            }
                        }
                        dir.set_low(); //reading mode
                    }

                    Err(_err) => {
                        error!("Error"); //TODO
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
async fn main(spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 1_000_000;
    config.detect_previous_overrun = false;
    let usart = Uart::new(
        p.USART1, p.PB15, p.PB14, Irqs, p.DMA1_CH0, p.DMA1_CH1, config,
    );

    // let usart1 = Uart::new(
    //     p.USART1, p.PB15, p.PB14, Irqs, p.DMA1_CH0, p.DMA1_CH1, config,
    // );

    unwrap!(spawner.spawn(dxl_serial(usart, p.PD9.into())));

    unwrap!(spawner.spawn(test_reg()));

    let mut led = Output::new(p.PC9, Level::High, Speed::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;

        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
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
