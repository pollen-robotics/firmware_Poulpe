#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(stmt_expr_attributes)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::mhz;
use embassy_stm32::{spi, Config};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_sync::blocking_mutex::Mutex;
use embedded_hal_1::spi::SpiDevice;
use firmware_poulpe::sensors::sensors::*;
use firmware_poulpe::sensors::*;
use firmware_poulpe::sensors::sensors_io;

use firmware_poulpe::utils::flash::*;
use firmware_poulpe::config::*;


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Orbita2d Zero to flash program!");

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
    let mut led_red = Output::new(p.PC8, Level::High, Speed::Low);
    led_red.set_low();
    led_green.set_low();

    #[cfg(not(feature = "orbita2d"))]
    {
        error!("This program is only for Orbita2d hardware!");
        led_red.set_high();
        return;
    }

    info!("----------------- SPI config -----------------");
    // Configure SPI
    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_1; // Aksim uses MODE1
    spi_config.frequency = mhz(1); // 10 MHz max clk
    #[cfg(feature = "pvt")]
    {
        spi_config.mode = spi::MODE_0;
    } // For LTC4332 internal interface

    // SPI4 - J3 - 3V3 powered
    let mut spi6 = spi::Spi::new(p.SPI6, p.PB3, p.PB5, p.PB4, NoDma, NoDma, spi_config);
    // create the shared mutex
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi6));

    // if pvt electronics we need to configure the LTC4332
    #[cfg(feature = "pvt")]
    {
        info!("----------------- LTC4332 config -----------------");
        let mut ltc4332_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(p.PD1, Level::High, Speed::Medium),
            spi_config,
        );

        let mut ltc4332 = ltc4332::LTC4332::new(ltc4332_spi);
        ltc4332.setup(ltc4332::LTC4332Config::Ring);
        info!(
            "LTC4332 configured, status: {=u8:#x},  config: {=u8:#b} ",
            ltc4332.read_status().unwrap_or_default(),
            ltc4332.read_config().unwrap_or_default()
        );
    }

    info!("----------------- RING config -----------------");
    let mut ring_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(p.PA15, Level::High, Speed::Medium),
        spi_config,
    );
    let mut ring_sensor = AksimSensor::new(ring_spi);
    let mut ring = SensorKind::Ring(ring_sensor);

    info!("----------------- SPI config -----------------");
    // Configure SPI
    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_1; // For AS5047 directly internal interface
    spi_config.frequency = mhz(1); // 10 MHz max clk
    spi_config.bit_order = spi::BitOrder::MsbFirst;
    #[cfg(feature = "pvt")]
    {
        spi_config.mode = spi::MODE_0;
    } // For LTC4332 internal interface

    // SPI4 - J3 - 3V3 powered
    let mut spi4 = spi::Spi::new(p.SPI4, p.PE12, p.PE6, p.PE5, NoDma, NoDma, spi_config);
    // create the shared mutex
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi4));

    // if pvt electronics we need to configure the LTC4332
    #[cfg(feature = "pvt")]
    {
        info!("----------------- LTC4332 config -----------------");
        let mut ltc4332_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(p.PB9, Level::High, Speed::Medium),
            spi_config,
        );

        let mut ltc4332 = ltc4332::LTC4332::new(ltc4332_spi);
        ltc4332.setup(ltc4332::LTC4332Config::Center);
        info!(
            "LTC4332 configured, status: {=u8:#x},  config: {=u8:#b} ",
            ltc4332.read_status().unwrap_or_default(),
            ltc4332.read_config().unwrap_or_default()
        );
    }

    info!("----------------- CENTER config -----------------");
    let mut center = AD5047Sensor::new(SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(p.PE4, Level::High, Speed::Medium),
        spi_config,
    ));
    let mut center = SensorKind::Center(center);


    info!("----------------- Flash config -----------------");
    let mut flash_manager = FlashManager::new(p.FLASH).await;

    info!("Reading from flash");
    match flash_manager.read() {
        Ok(b) => {
            // check if empty data
            if b.is_valid() {
                warn!(
                    "Data in flash is empty or corrupted, bord id:{}, hardware_zeros: {:?}",
                    b.board_id,  b.sensor_offsets
                );
            } else {
                info!("Data in flash valid, bord id:{}, hardware_zeros: {:?}",
                    b.board_id,  b.sensor_offsets
                );
            }
        }
        Err(e) => {
            error!(
                "Error reading from flash! Stopping execution! {:?}", 
                e
            );
            led_red.set_high();
            return;
        }
    }

    info!("----------------- Reading sensors -----------------");

    let angle_ring = sensors_io::get_axis_sensors_robust(&mut ring, 10, 1000).await.unwrap()[0];
    let angle_center = sensors_io::get_axis_sensors_robust(&mut center, 10, 1000).await.unwrap()[0];

    info!(
        "Angles: ring: {}, center: {}",
        angle_ring, angle_center
    );

    info!("----------------- Writing zeros to flash -----------------");
    let sensor_offsets = FlashData {
        board_id: DXL_ID,
        sensor_offsets: [0.0; N_AXIS],  
    };
    sensor_offsets.sensor_offsets[0] = angle_ring;
    sensor_offsets.sensor_offsets[1] = angle_center;
    
    info!("Zeros to be written to flash: {:?}", sensor_offsets);

    match flash_manager.lazy_checked_write(sensor_offsets   , 5).await {
        Ok(()) => info!("Write to flash OK"),
        Err(e) => error!("Error writing to flash: {:?}", e),
    }

    info!("----------------- Verifying zeros in flash -----------------");
    match flash_manager.read() {
        Ok(b) => {
            // check if empty data
            if b.is_valid() {
                error!(
                    "Data in flash is empty or corrupted, bord id:{}, hardware_zeros: {:?}",
                    b.board_id,  b.sensor_offsets
                );
            } else {
                info!("Data in flash valid, bord id:{}, hardware_zeros: {:?}",
                    b.board_id,  b.sensor_offsets
                );
            }
        }
        Err(e) => {
            error!(
                "Error reading from flash! Stopping execution! {:?}", 
                e
            );
            led_red.set_high();
            return;
        }
    }
    

    info!("----------------- Done : SUCCESS -----------------");
    loop {
        // Led
        led_green.set_high();
    }
}
