#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use embassy_stm32::{
    i2c, 
    peripherals,
    dma::NoDma,
};

use firmware_poulpe::{
    sensors::sensors::I2cHallSensor,
    IrqsI2c,
};


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello world!");
    let p = embassy_stm32::init(Default::default());

    info!("----------------- LEDs config -----------------");
    let mut led_green = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_red   = Output::new(p.PC8, Level::High, Speed::Low);
    led_green.set_low();
    led_red.set_high();



    info!("----------------- HALL sensor config ------------------");
    let i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        IrqsI2c,
        NoDma,
        NoDma,
        Hertz(100_000),
        Default::default(),
    );
    let mut hall_sensors = I2cHallSensor::new(i2c);


    led_red.set_low();
    
    loop {
        led_green.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led_green.set_low();

        match hall_sensors.read() {
            Ok(hall_detected) => {
                info!("Halls: {:#018b}", hall_detected);
            },
            Err(e) => {
                info!("Error: {:?}", e);
            }
        }

        Timer::after(Duration::from_millis(100)).await;
    }

}
