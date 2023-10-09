#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
// use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH1, USART1};
use embassy_stm32::usart::{BufferedUart, Config, Error, Uart, UartRx, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, usart};
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
// use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

// use rustypot::protocol::V1; //unfortunately, we are not ready for that yet ;(

mod dynamixel;
mod registers;
use {defmt_rtt as _, panic_probe as _};

// static RX_CHANNEL: Channel<ThreadModeRawMutex, [u8; 6], 1> = Channel::new();
// static TX_CHANNEL: Channel<ThreadModeRawMutex, [u8; 6], 1> = Channel::new();

static DXL_ID: u8 = 42;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::task]
async fn test_reg() {
    loop {
        {
            let mut registers = registers::REGISTERS.lock().await;
            registers.buffer[0] += 1;
        }

        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn DxlSerial(mut usart: Uart<'static, USART1, DMA1_CH0, DMA1_CH1>, dir_pin: AnyPin) {
    let mut dxlcom = dynamixel::DxlCom::new(DXL_ID);
    let mut dir = Output::new(dir_pin, Level::Low, Speed::High);

    let mut buf: [u8; 255] = [0; 255];

    dir.set_low(); //reading

    loop {
        let res = usart.read_until_idle(&mut buf).await;

        match res {
            Ok(nb) => {
                debug!("read {:?} bytes: {:?}", nb, buf[0..nb]);
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
                        dir.set_high();
                        // Timer::after(Duration::from_micros(500)).await;
                        // let _ = usart.blocking_write(data); //this doesn't work
                        match usart.write(data).await //this works
			{
			    Ok(())=>{
				//good
			    }
			    Err(_) =>{error!("Error sending response");}
			}
                        dir.set_low(); //reading
                    }

                    Err(err) => {
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

    unwrap!(spawner.spawn(DxlSerial(usart, p.PD9.into())));

    unwrap!(spawner.spawn(test_reg()));

    let mut led = Output::new(p.PC9, Level::High, Speed::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;

        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
