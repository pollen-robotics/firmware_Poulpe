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
use firmware_poulpe::sensors::ads124s0x::ADS124S0x;
use {defmt_rtt as _, panic_probe as _};

use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_sync::blocking_mutex::Mutex;
use embedded_hal_1::spi::SpiDevice;
//use firmware_poulpe::sensors::sensors::AD5047Sensor;
use firmware_poulpe::sensors::ads124s0x;
use firmware_poulpe::sensors::*;

#[embassy_executor::main]
pub async fn main(_spawner: Spawner) {
    info!("Force test!");

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

    info!("----------------- SPI config -----------------");
    // Configure SPI
    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_1; // For ADS124S0x
    spi_config.frequency = mhz(1); // 10 MHz max clk
    spi_config.bit_order = spi::BitOrder::MsbFirst;

    // SPI4 - J3 - 3V3 powered
    let mut spi4 = spi::Spi::new(p.SPI4, p.PE12, p.PE6, p.PE5, NoDma, NoDma, spi_config);
    // create the shared mutex
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi4));

    info!("-------------- Forcathon config --------------");
    /*let mut forces = ADS124S0x::new(SpiDeviceWithConfig::new(
        &spi_bus, 
        Output::new(p.PE4, Level::High, Speed::Medium),
        spi_config)
    );
    forces.reset();
    let dev = forces.read_reg_id().unwrap();
    match dev {
        0 => info!("ADC is ADS124S08 (12 chan.)"),
        1 => info!("ADC is ADS124S06 (6 chan.)"),
        _ => info!("ADC is unknown"),
    }
    info!("Status reg: {=u8:#x}", forces.read_reg_status().unwrap());
    let status = match forces.read_status_ok() {
        Ok(a) => {},
        Err(e) => error!("ADS124S0x is not ready")
    };
    forces.clear_power_on_reset();
    forces.set_mux_channels(ads124s0x::Ads124s0xMuxChannel::AIN0, ads124s0x::Ads124s0xMuxChannel::AIN1);
    forces.set_pga_gain(ads124s0x::Ads124s0xGain::Gain1);
    forces.enable_pga();
    forces.select_reference(ads124s0x::Ads124s0xRef::Ref2V5);
    forces.config_ref(ads124s0x::Ads124s0xInternalRefConf::IntRefOn);*/

    /*Power-up so that all supplies reach minimum operating levels;
*Delay for a minimum of 2.2 ms to allow power supplies to settle and power-up reset to complete;
*Configure the SPI interface of the microcontroller to SPI mode 1 (CPOL = 0, CPHA =1);
*If the CS pin is not tied low permanently, configure the microcontroller GPIO connected to CS as an output;
*Configure the microcontroller GPIO connected to the DRDY pin as a falling edge triggered interrupt input;
*Set CS to the device low;
-Delay for a minimum of td(CSSC);
*Send the RESET command (06h) to make sure the device is properly reset after power-up; //Optional
-Delay for a minimum of 4096 · tCLK;
*Read the status register using the RREG command to check that the RDY bit is 0; //Optional
*Clear the FL_POR flag by writing 00h to the status register; //Optional
*Write the respective register configuration with the WREG command;
For verification, read back all configuration registers with the RREG command;
Send the START command (08h) to start converting in continuous conversion mode;
Delay for a minimum of td(SCCS);
Clear CS to high (resets the serial interface);
Loop
{
Wait for DRDY to transition low;
Take CS low;
Delay for a minimum of td(CSSC);
Send the RDATA command;
Send 24 SCLK rising edges to read out conversion data on DOUT/DRDY;
Delay for a minimum of td(SCCS);
Clear CS to high;
}
Take CS low;
Delay for a minimum of td(CSSC);
Send the STOP command (0Ah) to stop conversions and put the device in standby mode;
Delay for a minimu */

    /*#[cfg(not(feature = "pvt"))]
    {
        // if gamma or beta the electronics seem to be a bit more sensitive
        // so we need to set the chip select pins high
        // for tmc4671
        Output::new(p.PE3, Level::High, Speed::Low).set_high();
        // for tmc4200/drv8316
        Output::new(p.PC15, Level::High, Speed::Low).set_high();
    }*/
    info!("----------------- Loop -----------------");
    loop {
        // Led
        led_green.set_high();
        Timer::after_millis(500).await;
        led_green.set_low();
        Timer::after_millis(500).await;

        /*let angle_center = match center.read_angle() {
            Ok(a) => a,
            Err(e) => {
                error!("Error reading center angle: {:?}", e);
                continue;
            }
        };
        info!("Center angle: {}", angle_center[0]);*/
    }
}
