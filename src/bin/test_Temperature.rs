#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(stmt_expr_attributes)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{spi, Config};
use embassy_time::{Duration, Timer};
use firmware_poulpe::sensors::analog::*;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Temperature test!");

    info!("----------------- Clock config -----------------");
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
    let mut p = embassy_stm32::init(stm32_conf);

    info!("----------------- Motor Temperature sensor config -----------------");
    #[cfg(all(feature = "orbita3d", feature = "pvt"))]
    let mut sensor_config = Orbita3dTemperatureConfig {
        adc: p.ADC1,
        pin1: p.PB1,
        pin2: p.PC5,
        pin3: p.PB0,
    };

    #[cfg(all(feature = "orbita2d", feature = "pvt"))]
    let mut sensor_config = Orbita2dTemperatureConfig {
        adc: p.ADC1,
        pin1: p.PC5,
        pin2: p.PB0,
    };
    #[cfg(not(feature = "pvt"))]
    let mut sensor_config = AnalogInputConfig {
        adc: p.ADC1,
        pin1: p.PB1,
    };
    let mut adc = adc_setup(&mut sensor_config.adc);

    info!("----------------- Loop -----------------");
    loop {
        #[cfg(all(feature = "orbita3d", feature = "pvt"))]
        info!(
            "Motor Temperatures: {} C, {} C, {} C",
            adc_read_temperature(&mut adc, &mut sensor_config.pin1),
            adc_read_temperature(&mut adc, &mut sensor_config.pin2),
            adc_read_temperature(&mut adc, &mut sensor_config.pin3)
        );
        #[cfg(all(feature = "orbita2d", feature = "pvt"))]
        info!(
            "Motor Temperatures: {} C, {} C",
            adc_read_temperature(&mut adc, &mut sensor_config.pin1),
            adc_read_temperature(&mut adc, &mut sensor_config.pin2)
        );
        #[cfg(not(feature = "pvt"))]
        info!(
            "Motor Temperature: {} C",
            adc_read_temperature(&mut adc, &mut sensor_config.pin1)
        );
        Timer::after_millis(500).await;
    }
}
