use core::cell::RefCell;

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Pin, Speed},
    peripherals as p,
    spi::{self, SckPin},
};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Duration, Timer};

use crate::{config, SHARED_MEMORY};

use super::{
    ventouse::{Ventouse, VentouseKind},
    Actuator, Driver, Foc, RawMotorsIO,
};

pub struct ActuatorConfig<
    TB,
    SCKB,
    MOSIB,
    MISOB,
    FocCsB,
    FocEnbB,
    DrvCsB,
    TC,
    SCKC,
    MOSIC,
    MISOC,
    FocCsC,
    FocEnbC,
    DrvCsC,
> where
    TB: spi::Instance,
    SCKB: SckPin<TB>,
    MOSIB: spi::MosiPin<TB>,
    MISOB: spi::MisoPin<TB>,
    FocCsB: Pin,
    FocEnbB: Pin,
    DrvCsB: Pin,
    TC: spi::Instance,
    SCKC: SckPin<TC>,
    MOSIC: spi::MosiPin<TC>,
    MISOC: spi::MisoPin<TC>,
    FocCsC: Pin,
    FocEnbC: Pin,
    DrvCsC: Pin,
{
    pub b: VentouseConfig<TB, SCKB, MOSIB, MISOB, FocCsB, FocEnbB, DrvCsB>,
    pub c: VentouseConfig<TC, SCKC, MOSIC, MISOC, FocCsC, FocEnbC, DrvCsC>,
}

pub struct VentouseConfig<T, SCK, MOSI, MISO, FocCs, FocEnb, DrvCs>
where
    T: spi::Instance,
    SCK: SckPin<T>,
    MOSI: spi::MosiPin<T>,
    MISO: spi::MisoPin<T>,
    FocCs: Pin,
    FocEnb: Pin,
    DrvCs: Pin,
{
    pub peri: T,
    pub sck: SCK,
    pub mosi: MOSI,
    pub miso: MISO,

    pub foc_cs: FocCs,
    pub foc_enable: FocEnb,

    pub driver_cs: DrvCs,
}

#[embassy_executor::task]
pub async fn control_loop(
    config: ActuatorConfig<
        p::SPI4,
        p::PE12,
        p::PE6,
        p::PE5,
        p::PE3,
        p::PE0,
        p::PC15,
        p::SPI6,
        p::PB3,
        p::PB5,
        p::PB4,
        p::PD7,
        p::PD5,
        p::PD6,
    >,
) {
    let spi_config = spi::Config::default();
    let mut foc_spi_config = spi::Config::default();
    foc_spi_config.mode = spi::MODE_3;
    let driver_spi_config = spi::Config::default();

    // Ventouse B
    let spi = spi::Spi::new(
        config.b.peri,
        config.b.sck,
        config.b.mosi,
        config.b.miso,
        NoDma,
        NoDma,
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
        config::BrushlessMotor::ecx22(),
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
        config::BrushlessMotor::ecx22(),
    );

    let driver_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.c.driver_cs, Level::High, Speed::Medium),
        driver_spi_config,
    );
    let driver = Driver::new(driver_spi);

    let ventouse_c = Ventouse::new(foc, driver);
    let ventouse_c = VentouseKind::C(ventouse_c);

    // Setup the actuator with the configured ventouses
    #[cfg(feature = "orbita2d")]
    let mut actuator = Actuator::new([ventouse_b, ventouse_c]);
    #[cfg(feature = "orbita3d")]
    let mut actuator = Actuator::new([ventouse_a, ventouse_b, ventouse_c]);

    // Init SharedMemory with real values before actually running the control loop
    SHARED_MEMORY.lock().await.init(&mut actuator);

    actuator.init().await;

    loop {
        let pos = actuator.get_current_position().unwrap();
        {
            SHARED_MEMORY.lock().await.set_current_position(pos)
        }

        let torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };
        actuator.set_torque(torque_on).unwrap();

        let target = { SHARED_MEMORY.lock().await.get_target_position() };
        actuator.set_target_position(target).unwrap();

        Timer::after(Duration::from_millis(1)).await;
    }
}
