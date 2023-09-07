#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
// use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH1, USART1};
use embassy_stm32::usart::{Config, Uart, UartRx, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
// use embassy_time::{Duration, Timer};
// use rustypot::protocol::V1; //unfortunately, we are not ready for that yet ;(

use {defmt_rtt as _, panic_probe as _};

static RX_CHANNEL: Channel<ThreadModeRawMutex, [u8; 6], 1> = Channel::new();
static TX_CHANNEL: Channel<ThreadModeRawMutex, [u8; 6], 1> = Channel::new();

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::task]
async fn writer(mut usart: UartTx<'static, USART1, DMA1_CH0>, dir_pin: AnyPin) {
    let mut dir = Output::new(dir_pin, Level::Low, Speed::High);

    loop {
        let buf = TX_CHANNEL.receive().await;
        dir.set_high(); //TODO: high or low?
        usart.write(&buf).await.ok();
        dir.set_low();
    }
}

#[embassy_executor::task]
async fn reader(mut rx: UartRx<'static, USART1, DMA1_CH1>) {
    let mut buf = [0; 6];
    loop {
        info!("reading...");
        unwrap!(rx.read(&mut buf).await);
        RX_CHANNEL.send(buf).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 1_000_000;
    let usart = Uart::new(
        p.USART1, p.PB15, p.PB14, Irqs, p.DMA1_CH0, p.DMA1_CH1, config,
    );

    // unwrap!(usart.blocking_write(b"Type 8 chars to echo!\r\n"));

    let (tx, rx) = usart.split();

    unwrap!(spawner.spawn(reader(rx)));
    unwrap!(spawner.spawn(writer(tx, p.PD9.into())));

    loop {
        let buf = RX_CHANNEL.receive().await;
        let mut pp = [0xff, 0xff, 0x01, 0x02, 0x01, 0xfb];

        /////////TODO
        // let bytes = vec![0x00];
        // let sp = StatusPacketV1::from_bytes(&bytes, 42);

        if pp == buf {
            info!("Answering to ping");
            let mut sp = [0xff, 0xff, 0x01, 0x02, 0x00, 0xfc];

            // TX_CHANNEL.send(buf).await;
            TX_CHANNEL.send(sp).await;
            // unwrap!(tx.write(&buf).await);
        } else {
            info!("Received: {:?}", buf);
        }
    }
}
