#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(stmt_expr_attributes)]

use cortex_m::register::control;
use embassy_executor::Spawner;
use defmt::*;
use embassy_stm32::time::mhz;
use embassy_stm32::{spi, Config};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Timer, Duration,Ticker, Instant};
use embassy_stm32::dma::{NoDma};
use embassy_stm32::spi::Spi;

use core::cell::RefCell;
use embassy_sync::blocking_mutex::{Mutex};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embedded_hal_1::spi::SpiDevice;

use firmware_poulpe::motor_control::ventouse::*;    
use firmware_poulpe::motor_control::*;
use firmware_poulpe::config::{CurrentSensing, BrushlessMotor};

#[embassy_executor::main]
pub async fn main(_spawner: Spawner) {
    info!("Hello World!");

    info!("----------------- Clock config -----------------");
    // 440MHz (without HSE)
    let mut stm32_conf = Config::default();
    {
        use embassy_stm32::rcc::*;
        stm32_conf.rcc.hsi = Some(HSIPrescaler::DIV1); //HSIState = RCC_HSI_DIV1
        stm32_conf.rcc.csi = true; //CSIState = RCC_CSI_ON;
                                   // stm32_conf.rcc.hse = Som(Hse{Hertz::mhz(48), HseMode::Oscillator}); //TODO hse external clock might be more accurate
        stm32_conf.rcc.pll1 = Some(Pll {
            // source: PllSource::HSI
            source: PllSource::CSI, //PLLSource = RCC_PLLSOURCE_CSI

            prediv: PllPreDiv::DIV1,  //PLLM = 1;
            mul: PllMul::MUL220,      //PLLN = 220
            divp: Some(PllDiv::DIV2), //PLLP = 2;
            divq: Some(PllDiv::DIV5), //PLLQ = 5;
            divr: Some(PllDiv::DIV5), //PLLR = 5;
        });
        stm32_conf.rcc.pll2 = Some(Pll {
            source: PllSource::CSI,
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

    info!("----------------- LEDs config -----------------");
    let mut led_green = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_red   = Output::new(p.PC8, Level::High, Speed::Low);


    info!("----------------- SPI config -----------------");
    let mut foc_spi_config = spi::Config::default();
    foc_spi_config.frequency = mhz(2);
    foc_spi_config.mode = spi::MODE_3;
    foc_spi_config.bit_order = spi::BitOrder::MsbFirst;
    let mut driver_spi_config = spi::Config::default();
    driver_spi_config.mode = spi::MODE_3;
    #[cfg(all(any(feature = "gamma", feature="pvt"), feature = "orbita3d"))]
    { driver_spi_config.mode = spi::MODE_1; }
    driver_spi_config.frequency =mhz(2);
    driver_spi_config.bit_order = spi::BitOrder::MsbFirst;

    let config = VentouseConfig{
        peri: p.SPI4,
        sck: p.PE12,
        mosi: p.PE6,
        miso: p.PE5,
        foc_cs: p.PE3,
        foc_enable: p.PE0,
        driver_cs: p.PC15,
        driver_status_pin: p.PC14,
        #[cfg(feature = "orbita3d")]
        motor_config: BrushlessMotor::ecx22(), // motor config for the ECX22
        #[cfg(feature = "orbita2d")] 
        motor_config: BrushlessMotor::ec45(), // motor config for the EC45
        #[cfg(feature = "beta")]
        current_sense_config: CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
        #[cfg(all(any(feature = "gamma", feature="pvt"), feature = "orbita2d"))]
        current_sense_config: CurrentSensing::ventouse_2d(), // current sense for gamma elec ventouse 2d
        #[cfg(all(any(feature = "gamma", feature="pvt"), feature = "orbita3d"))]
        current_sense_config: CurrentSensing::ventouse_3d(), // current sense for gamma elec ventouse 2d
    };

    // SPI4 - J3 - 3V3 powered
    let mut spi4 = spi::Spi::new(
        config.peri,
        config.sck,
        config.mosi,
        config.miso,
        NoDma, 
        NoDma, 
        spi::Config::default());
    // create the shared mutex
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi4));   

    info!("----------------- FOC config -----------------");
    let foc = Foc::new(
        SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.foc_cs, Level::High, Speed::Medium),
            foc_spi_config,
        ),
        config.foc_enable,
        config.motor_config,
        config.current_sense_config
    );

    info!("----------------- Driver config -----------------");
    let driver_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.driver_cs, Level::High, Speed::Medium),
        driver_spi_config,
    );
    #[cfg(all(feature = "orbita3d", any(feature = "gamma", feature="pvt")))]
    let driver = DriverDRV8316::new(driver_spi, config.driver_status_pin);
    #[cfg(any(feature = "beta", all(feature = "orbita2d", any(feature = "gamma", feature="pvt"))))]
    let driver = DriverTMC6200::new(driver_spi, config.driver_status_pin);


    info!("----------------- Ventouse config -----------------");
    let mut controller: Ventouse<'_, embassy_stm32::peripherals::SPI4, embassy_stm32::peripherals::PE3, embassy_stm32::peripherals::PE0, _> = Ventouse::new(foc, driver);
    match controller.init('A').await{
        Ok(_) => info!("Ventouse initialized"),
        Err(e) => error!("Ventouse initialization failed: {:?}", e),
    }
    Timer::after(Duration::from_millis(500)).await;
    match controller.check_motors_1().await{
        Ok(_) => info!("Check 1 passed"),
        Err(e) => error!("Check 1 failed: {:?}", e),
    }
    Timer::after(Duration::from_millis(500)).await;
    match controller.check_motors_2().await{
        Ok(_) => info!("Check 2 passed"),
        Err(e) => error!("Check 2 failed: {:?}", e),
    }

    let t0 = Instant::now();
    let initial_position = controller.get_current_position().unwrap_or_default()[0];

    info!("----------------- Ventouse loop -----------------");
    let mut ticker = Ticker::every(Duration::from_millis(10));
    // set the green led
    led_green.set_high();
    loop {
        let angle = libm::sin(t0.elapsed().as_millis() as f64/1000.0) as f32;
        controller.set_target_position([angle + initial_position]);

        let current_position = controller.get_current_position().unwrap_or_default()[0];
        ticker.next().await;
        info!("Target position: {} \t Current position: {}", angle + initial_position, current_position);
    }

}
