use core::cell::RefCell;

use defmt::info;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
    spi,
};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Duration, Timer, block_for};

use crate::{
    config::{self, ActuatorConfig},
    SHARED_MEMORY,
};

use super::{
    ventouse::{Ventouse, VentouseKind},
    Actuator, Driver, Foc, RawMotorsIO, sensors::AksimSensor,
};

#[embassy_executor::task]
pub async fn control_loop(config: ActuatorConfig) {
    let mut spi_config = spi::Config::default();
    spi_config.frequency = embassy_stm32::time::Hertz(1_000_000);
    spi_config.mode = spi::MODE_3;

    let mut foc_spi_config = spi::Config::default();
    foc_spi_config.frequency = embassy_stm32::time::Hertz(1_000_000);
    foc_spi_config.mode = spi::MODE_3;
    let mut driver_spi_config = spi::Config::default();
    driver_spi_config.mode = spi::MODE_3;
    driver_spi_config.frequency = embassy_stm32::time::Hertz(1_000_000);
    /*
    // Ventouse A
    #[cfg(feature = "orbita3d")]
    let spi = spi::Spi::new(
        config.a.peri,
        config.a.sck,
        config.a.mosi,
        config.a.miso,
        NoDma,
        NoDma,
        spi_config,
    );
    #[cfg(feature = "orbita3d")]
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));
    #[cfg(feature = "orbita3d")]
    let foc_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.a.foc_cs, Level::High, Speed::Medium),
        foc_spi_config,
    );
    #[cfg(feature = "orbita3d")]
    let foc = Foc::new(
        foc_spi,
        config.a.foc_enable,
        config::BrushlessMotor::ecx22(),
    );
    #[cfg(feature = "orbita3d")]
    let driver_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.a.driver_cs, Level::High, Speed::Medium),
        driver_spi_config,
    );
    #[cfg(feature = "orbita3d")]
    let driver = Driver::new(driver_spi);
    #[cfg(feature = "orbita3d")]
    let ventouse_a = Ventouse::new(foc, driver);
    #[cfg(feature = "orbita3d")]
    let ventouse_a = VentouseKind::A(ventouse_a);
    */

    // Ventouse B
    let spi = spi::Spi::new(
        config.b.peri,
        config.b.sck,
        config.b.mosi,
        config.b.miso,
        NoDma,
        NoDma,
        // spi::Config::default(),
	spi_config,
    );
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));

    let foc_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.b.foc_cs, Level::High, Speed::Medium),
        foc_spi_config,
    );
    let foc = Foc::new(
        foc_spi,
        config.b.foc_enable,
        config::BrushlessMotor::ec45(),
    );

    let driver_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.b.driver_cs, Level::High, Speed::Medium),
        driver_spi_config,
    );
    let driver = Driver::new(driver_spi);

    let ventouse_b = Ventouse::new(foc, driver);
    let ventouse_b = VentouseKind::B(ventouse_b);

    // Ventouse C
    let spi = spi::Spi::new(
        config.c.peri,
        config.c.sck,
        config.c.mosi,
        config.c.miso,
        NoDma,
        NoDma,
        // spi::Config::default(),
	spi_config,
    );
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));

    let foc_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.c.foc_cs, Level::High, Speed::Medium),
        foc_spi_config,
    );
    let foc = Foc::new(
        foc_spi,
        config.c.foc_enable,
        config::BrushlessMotor::ec45(),
    );

    let driver_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.c.driver_cs, Level::High, Speed::Medium),
        driver_spi_config,
    );
    let driver = Driver::new(driver_spi);

    let ventouse_c = Ventouse::new(foc, driver);
    let ventouse_c = VentouseKind::C(ventouse_c);


    //Aksim Ring sensor
    let mut aksim_spi_config = spi::Config::default();
    aksim_spi_config.frequency = embassy_stm32::time::Hertz(1_000_000);
    aksim_spi_config.mode = spi::MODE_1;
    let aksim_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.aksim.cs, Level::High, Speed::Medium),
        aksim_spi_config,
    );

    let mut aksim=AksimSensor::new(aksim_spi);


    // Setup the actuator with the configured ventouses
    #[cfg(feature = "orbita2d")]
    let mut actuator = Actuator::new([ventouse_b, ventouse_c]);
    #[cfg(feature = "orbita3d")]
    let mut actuator = Actuator::new([ventouse_a, ventouse_b, ventouse_c]);

    actuator.init().await;
    block_for(Duration::from_secs(1));
    info!("init done");
    // Init SharedMemory with real values before actually running the control loop
    SHARED_MEMORY.lock().await.init(&mut actuator);



    loop {

        let pos = actuator.get_current_position().unwrap();
        {
            SHARED_MEMORY.lock().await.set_current_position(pos)
        }

        let torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };
        actuator.set_torque(torque_on).unwrap();

        let target = { SHARED_MEMORY.lock().await.get_target_position() };
        actuator.set_target_position(target).unwrap();

	/*
	let aksim_angle=aksim.read_angle().await;
	match aksim_angle {
	    Ok(angle) => {
		info!("aksim angle: {}", angle);

	    },
	    Err(e) => {
		info!("aksim error: {:?}", e);
	    }
	}
	 */



        Timer::after(Duration::from_millis(1)).await;
    }
}
