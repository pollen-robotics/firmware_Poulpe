use core::cell::RefCell;

use defmt::{debug, error, info, warn};
use embassy_embedded_hal::shared_bus::blocking::{spi::SpiDeviceWithConfig};
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
    spi,
};

use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{ Duration, Instant, Ticker, Timer};
use micromath::F32Ext;

const SPI_FREQ: u32 = 2_000_000;

use crate::{
    config::{self, ActuatorConfig},
};
use crate::motor_control::motors_io::RawMotorsIO;
use crate::motor_control::Foc;
use super::{
    ventouse::{Ventouse, VentouseKind},
};

use super::driver::{DriverTMC6200};


#[embassy_executor::task]
pub async fn control_loop(config: ActuatorConfig) {
    
    
    // wait 5 secs
    Timer::after(Duration::from_secs(5)).await;

    let mut spi_config = spi::Config::default();
    spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);
    spi_config.bit_order = spi::BitOrder::MsbFirst;

    spi_config.mode = spi::MODE_1;

    let mut foc_spi_config = spi::Config::default();
    foc_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);
    foc_spi_config.mode = spi::MODE_3;
    foc_spi_config.bit_order = spi::BitOrder::MsbFirst;
    let mut driver_spi_config = spi::Config::default();
    driver_spi_config.mode = spi::MODE_3;
    

    // Ventouse B
    let spi = spi::Spi::new(
        config.b.peri,
        config.b.sck,
        config.b.mosi,
        config.b.miso,
        NoDma,
        NoDma,
        spi::Config::default(),
        // spi_config,
    );
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));

    let mut ventouse_b = {
        let foc_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.b.foc_cs, Level::High, Speed::Medium),
            foc_spi_config,
        );
        let foc = Foc::new(
            foc_spi,
            config.b.foc_enable,
            #[cfg(feature = "ec45")]
            config::BrushlessMotor::ec45(),
            #[cfg(feature = "ec60")]
            config::BrushlessMotor::ec60(),
            #[cfg(feature = "ecx22")]
            config::BrushlessMotor::ecx22(),
            config::CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
        );

        let driver_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.b.driver_cs, Level::High, Speed::Medium),
            driver_spi_config,
        );

        let driver = DriverTMC6200::new(driver_spi, config.b.driver_status_pin);
        let ventouse_b = Ventouse::new(foc, driver);
        VentouseKind::B(ventouse_b)
    };



    // configure the motors of the actuator
    let res = ventouse_b.init().await;

    // verify that the motors are correctly configured
    match res {
        Ok(_) => {
            info!("Actuator init ok");
        }
        Err(e) => {
            error!("Actuator init error: {:?}", e);
        }
    }

    info!("torque control loop started");
    ventouse_b.set_torque([true]);
    // ventouse_b.set_control_mode(super::foc::MotionMode::Velocity);
    ventouse_b.set_control_mode(super::foc::MotionMode::Torque);

    info!("torque control loop started");

    let mut ticker = Ticker::every(Duration::from_micros(1000));
    let mut t0: Instant = Instant::now();
    let mut t1: Instant = Instant::now();


    let mut torque_target = 1000.0;
    let mut velocity_mean = 0.0; // rad/s
    let mut velocity_variance = 0.0; // rad/s
    loop {
        
        // ventouse_b.set_target_velocity([10.0]);
        ventouse_b.set_target_torque([torque_target]);


        match ventouse_b.get_current_velocity(){
            Ok(v) => {
                velocity_mean = 0.99*velocity_mean + 0.01*v[0];
                velocity_variance = 0.99*velocity_variance + 0.01*(v[0]-velocity_mean)*(v[0]-velocity_mean);
            },
            Err(e) => {
                error!("Error getting velocity: {:?}", e);
            }
        };

        if t1.elapsed().as_secs() > 10 {
            t1 = Instant::now();
            torque_target = -torque_target;
            info!("velocity: \tmean: {} rad/s,\tstddev: {} rad/s",  velocity_mean, velocity_variance.sqrt());
            info!("new torque target: {}", torque_target);
        }

        if t0.elapsed().as_secs() > 100 {
            info!("torque control loop stopped");
            ventouse_b.set_target_torque([0.0]);
            break;
        }

        // let elapsed=t0.elapsed().as_micros();
        // info!("Motor control loop elapsed: {} us",elapsed);
        // Timer::after(Duration::from_micros(1000-elapsed)).await;
        // Timer::after(Duration::from_millis(1)).await;
        ticker.next().await;
    }
}
