#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(stmt_expr_attributes)]

use cortex_m::register::control;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::mhz;
use embassy_stm32::{spi, Config};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Ticker, Timer};

use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_sync::blocking_mutex::Mutex;
use embedded_hal_1::spi::SpiDevice;

use firmware_poulpe::config::{BrushlessMotor, CurrentSensing};
use firmware_poulpe::motor_control::ventouse::*;
use firmware_poulpe::motor_control::*;
use firmware_poulpe::motor_control::foc::MotionMode;

#[embassy_executor::main]
pub async fn main(_spawner: Spawner) {
    info!("BENCH: Test Orbita2d!");

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


    Timer::after(Duration::from_millis(1000)).await;

    info!("----------------- LEDs config -----------------");
    let mut led_green = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_red = Output::new(p.PC8, Level::High, Speed::Low);

    info!("----------------- SPI config -----------------");
    let mut foc_spi_config = spi::Config::default();
    foc_spi_config.frequency = mhz(2);
    foc_spi_config.mode = spi::MODE_3;
    foc_spi_config.bit_order = spi::BitOrder::MsbFirst;
    let mut driver_spi_config = spi::Config::default();
    driver_spi_config.mode = spi::MODE_3;
    driver_spi_config.frequency = mhz(2);
    driver_spi_config.bit_order = spi::BitOrder::MsbFirst;

    let config_a = VentouseConfig {
        peri: p.SPI4,
        sck: p.PE12,
        mosi: p.PE6,
        miso: p.PE5,
        foc_cs: p.PE3,
        foc_enable: p.PE0,
        driver_cs: p.PC15,
        driver_status_pin: p.PC14,
        motor_config: BrushlessMotor::ec45(), // motor config for the EC45
        #[cfg(feature = "beta")]
        current_sense_config: CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
        #[cfg(not(feature = "beta"))]
        current_sense_config: CurrentSensing::ventouse_2d(), // current sense for gamma elec ventouse 2d
    };

    let config_b = VentouseConfig {
        peri: p.SPI6,
        sck: p.PB3,
        mosi: p.PB5,
        miso: p.PB4,
        foc_cs: p.PD7,
        foc_enable: p.PD5,
        driver_cs: p.PD6,
        driver_status_pin: p.PD3,
        motor_config: BrushlessMotor::ec45(),
        #[cfg(feature = "beta")]
        current_sense_config: CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
        #[cfg(not(feature = "beta"))]
        current_sense_config: CurrentSensing::ventouse_2d(), // current sense for gamma elec ventouse 2d
    };

    // SPI4 - J3 - 3V3 powered
    let mut spi_a = spi::Spi::new(
        config_a.peri,
        config_a.sck,
        config_a.mosi,
        config_a.miso,
        NoDma,
        NoDma,
        spi::Config::default(),
    );

    // SPI4 - J3 - 3V3 powered
    let mut spi_b = spi::Spi::new(
        config_b.peri,
        config_b.sck,
        config_b.mosi,
        config_b.miso,
        NoDma,
        NoDma,
        spi::Config::default(),
    );
    // create the shared mutex
    let spi_bus_a: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi_a));
    let spi_bus_b: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi_b));

    info!("----------------- FOC config -----------------");
    let foc_a = Foc::new(
        SpiDeviceWithConfig::new(
            &spi_bus_a,
            Output::new(config_a.foc_cs, Level::High, Speed::Medium),
            foc_spi_config,
        ),
        config_a.foc_enable,
        config_a.motor_config,
        config_a.current_sense_config,
    );
    let foc_b = Foc::new(
        SpiDeviceWithConfig::new(
            &spi_bus_b,
            Output::new(config_b.foc_cs, Level::High, Speed::Medium),
            foc_spi_config,
        ),
        config_b.foc_enable,
        config_b.motor_config,
        config_b.current_sense_config,
    );

    info!("----------------- Driver config -----------------");
    let driver_spi_a = SpiDeviceWithConfig::new(
        &spi_bus_a,
        Output::new(config_a.driver_cs, Level::High, Speed::Medium),
        driver_spi_config
    );
    let driver_a = DriverTMC6200::new(driver_spi_a, config_a.driver_status_pin);

    let driver_spi_b = SpiDeviceWithConfig::new(
        &spi_bus_b,
        Output::new(config_b.driver_cs, Level::High, Speed::Medium),
        driver_spi_config,
    );
    let driver_b = DriverTMC6200::new(driver_spi_b, config_b.driver_status_pin);

    info!("----------------- Ventouse config -----------------");
    let mut controller_a: Ventouse<
        '_,
        embassy_stm32::peripherals::SPI4,
        embassy_stm32::peripherals::PE3,
        embassy_stm32::peripherals::PE0,
        _,
    > = Ventouse::new(foc_a, driver_a);
    match controller_a.init('A').await {
        Ok(_) => info!("Ventouse initialized"),
        Err(e) => error!("Ventouse initialization failed: {:?}", e),
    }

    let mut controller_b: Ventouse<
        '_,
        embassy_stm32::peripherals::SPI6,
        embassy_stm32::peripherals::PD7,
        embassy_stm32::peripherals::PD5,
        _,
    > = Ventouse::new(foc_b, driver_b);
    match controller_b.init('B').await {
        Ok(_) => info!("Ventouse initialized"),
        Err(e) => error!("Ventouse initialization failed: {:?}", e),
    }

    Timer::after(Duration::from_millis(500)).await;

    match controller_a.set_control_mode(MotionMode::Torque){
        Ok(_) => info!("Control mode set to Torque"),
        Err(e) => error!("Control mode set to Torque failed: {:?}", e),
    }
    Timer::after(Duration::from_millis(500)).await;

    match controller_b.set_control_mode(MotionMode::Torque){
        Ok(_) => info!("Control mode set to Torque"),
        Err(e) => error!("Control mode set to Torque failed: {:?}", e),
    }
    Timer::after(Duration::from_millis(500)).await;

    controller_a.set_torque([true]);

    info!("----------------- Ventouse loop -----------------");
    let mut ticker = Ticker::every(Duration::from_millis(10));
    // set the green led
    led_green.set_high();

    let mut torque_target = 400.0;
    let mut current_dir = 1;    // 1 - ring axis positive, 
                                // 2 - ring axis negative, 
                                // 3 - center axis positive, 
                                // 4 - center axis negative 
    let mut torque_a = torque_target;
    let mut torque_b = torque_target;
    let mut t0 = Instant::now();
    loop {

        // do 5 seconds of torque: 
        //  - positive, positive  - ring axis rotation positive
        //  - negative, negative  - ring axis rotation negative
        //  - positive, negative  - center axis rotation positive
        //  - negative, positive  - center axis rotation negative
        // repeat for 1 minute
        if t0.ellapsed() > Duration::from_secs(5) {
            t0 = Instant::now();
            match current_dir {
                1 => {
                    current_dir = 2;
                    info!("Switching to ring axis negative");
                    torque_a = -torque_target;
                    torque_b = -torque_target;
                },
                2 => {
                    current_dir = 3;
                    info!("Switching to center axis positive");
                    torque_a = torque_target;
                    torque_b = -torque_target;
                },
                3 => {
                    current_dir = 4;
                    info!("Switching to center axis negative");
                    torque_a = -torque_target;
                    torque_b = torque_target;
                },
                4 => {
                    current_dir = 1;
                    info!("Switching to ring axis positive");
                    torque_a = torque_target;
                    torque_b = torque_target;
                },
                _ => {
                    current_dir = 1;
                    info!("Switching to ring axis positive");
                    torque_a = torque_target;
                    torque_b = torque_target;
                }
            }
        }
        if t0.ellapsed() > Duration::from_secs(60) {
            info!("Test finished");
            break;
        }
        
        controller_a.set_target_torque([torque_a]);
        controller_b.set_target_torque([torque_b]);

        let current_torque_a = controller_a.get_current_torque().unwrap_or_default()[0];
        let current_torque_b = controller_b.get_current_torque().unwrap_or_default()[0];
        ticker.next().await;
        info!(
            "Target torque : [{},{}] mA\t Current torque: [{},{}] mA",
            torque_a, torque_b,
            current_torque_a,
            current_torque_b
        );
    }
}
