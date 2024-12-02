#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(async_fn_in_trait)]
#![feature(array_methods)]

use defmt::{error, info, unwrap};
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::usart::Config as usart_config;
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_stm32::{i2c, Config as stm32_config};
use embassy_stm32::{spi, Config};
use embassy_time::{block_for, Duration, Ticker, Timer};
use rand_core::le;

use firmware_poulpe::config::{
    AD5047Config, AD5047ConfigBot, AD5047ConfigMid, AD5047ConfigTop, ActuatorConfig, AksimConfig,
    LAN9252Config, LTC4332CenterConfig, LTC4332DonutConfig, LTC4332RingConfig,
};
use firmware_poulpe::{
    config::{self, BrushlessMotor, CurrentSensing},
    dynamixel, ethercat,
    ethercat::EthercatConfig,
    motor_control,
    motor_control::ventouse::VentouseConfig,
    sensors::sensors::I2cHallConfig,
    state_machine::poulpe_state,
    utils,
    utils::flash,
    Irqs, SHARED_MEMORY,
};

#[cfg(not(feature = "no_temperature_sensor"))]
use firmware_poulpe::config::TemperatureSensingConfig;
#[cfg(feature = "use_flash")]
use firmware_poulpe::utils::flash::{FlashData, FlashManager};

// from build.rs
// include!(concat!(env!("OUT_DIR"), "/constants.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("==== Pollen Robotics ====");
    #[cfg(feature = "orbita3d")]
    info!("Poulpe: Orbita 3D");
    #[cfg(feature = "orbita2d")]
    info!("Poulpe: Orbita 2D");

    #[cfg(feature = "pvt")]
    info!("Verison: PVT");
    #[cfg(feature = "beta")]
    info!("Verison: Beta");
    #[cfg(feature = "gamma")]
    info!("Verison: Gamma");

    #[cfg(feature = "ec45")]
    info!("Motors: EC45");
    #[cfg(feature = "ec60")]
    info!("Motors: EC60");
    #[cfg(feature = "ecx22")]
    info!("Motors: ECX22");
    #[cfg(feature = "ecx22l")]
    info!("Motors: ECX22L");

    #[cfg(feature = "ethercat")]
    info!("Communication: EtherCAT");
    #[cfg(feature = "dynamixel")]
    info!("Communication: Dynamixel");
    #[cfg(not(any(feature = "ethercat", feature = "dynamixel",)))]
    warn!("No communication enabled");

    info!("Git commit: {:?}", config::GIT_HASH); //TODO: read access from a dxl msg?
                                                 // info!("Hardware_zeros: {:?}", config::HARDWARE_ZEROS); // For Orbita3d firmware zero

    // 440MHz (without HSE)
    let mut stm32_conf = stm32_config::default();
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

    // set the default values into the memory
    let mut board_id = config::DXL_ID;
    let mut hardware_zeros: [f32; config::N_AXIS] = config::HARDWARE_ZEROS;

    #[cfg(feature = "use_flash")]
    {
        let mut flash_manager = FlashManager::new(p.FLASH).await;
        #[cfg(feature = "write_flash")]
        {
            info!("Writing to flash");
            // user want to use these values
            // and write them to flash
            let mut poulpe_config = FlashData {
                board_id: board_id,
                sensor_offsets: hardware_zeros,
            };
            match flash_manager.lazy_checked_write(poulpe_config, 5).await {
                Ok(()) => info!("Write to flash OK"),
                Err(e) => error!("Error writing to flash: {:?}", e),
            }
        }
        #[cfg(not(feature = "write_flash"))]
        {
            info!("Reading from flash");
            match flash_manager.read() {
                Ok(b) => {
                    info!("Read from flash: {:?}", b);
                    // check if empty data
                    if b.is_valid() {
                        error!(
                            "Data in flash is empty or corrupted, using default values! {}, {:?}",
                            board_id, hardware_zeros
                        );
                    } else {
                        info!("Data in flash valid, using values from flash");
                        board_id = b.board_id;
                        hardware_zeros = b.sensor_offsets;
                        info!(
                            "board id: {:?} hardware_zeros: {:?}",
                            board_id, hardware_zeros
                        );
                    }
                }
                Err(e) => {
                    error!(
                        "Error reading from flash: {:?}, Using default values! {}, {:?}",
                        e, board_id, hardware_zeros
                    );
                }
            }
        }
    }

    // Spawn the control loop
    #[cfg(feature = "orbita3d")]
    let actuator_config = ActuatorConfig {
        a: VentouseConfig {
            peri: p.SPI1,
            sck: p.PA5,
            mosi: p.PA7,
            miso: p.PA6,
            foc_cs: p.PA3,
            foc_enable: p.PC0,
            driver_cs: p.PA2,
            driver_status_pin: p.PA1,
            motor_config: BrushlessMotor::ecx22(),
            #[cfg(feature = "beta")]
            current_sense_config: CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
            #[cfg(any(feature = "gamma", feature = "pvt"))]
            current_sense_config: CurrentSensing::ventouse_3d(), // current sense for gamma elec ventouse 2d
        },
        b: VentouseConfig {
            peri: p.SPI4,
            sck: p.PE12,
            mosi: p.PE6,
            miso: p.PE5,
            foc_cs: p.PE3,
            foc_enable: p.PE0,
            driver_cs: p.PC15,
            driver_status_pin: p.PC14,
            motor_config: BrushlessMotor::ecx22(),
            #[cfg(feature = "beta")]
            current_sense_config: CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
            #[cfg(any(feature = "gamma", feature = "pvt"))]
            current_sense_config: CurrentSensing::ventouse_3d(), // current sense for gamma elec ventouse 2d
        },
        c: VentouseConfig {
            peri: p.SPI6,
            sck: p.PB3,
            mosi: p.PB5,
            miso: p.PB4,
            foc_cs: p.PD7,
            foc_enable: p.PD5,
            driver_cs: p.PD6,
            driver_status_pin: p.PD3,
            motor_config: BrushlessMotor::ecx22(),
            #[cfg(feature = "beta")]
            current_sense_config: CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
            #[cfg(any(feature = "gamma", feature = "pvt"))]
            current_sense_config: CurrentSensing::ventouse_3d(), // current sense for gamma elec ventouse 2d
        },

        ad5047top: AD5047ConfigTop { cs: p.PA4 },
        ad5047mid: AD5047ConfigMid { cs: p.PE4 },
        ad5047bot: AD5047ConfigBot { cs: p.PA15 },
        #[cfg(all(not(feature = "no_temperature_sensor"), feature = "pvt"))]
        temperature_sensing: TemperatureSensingConfig {
            adc: p.ADC1,
            pin1: p.PB1,
            pin2: p.PC5,
            pin3: p.PB0,
        },
        #[cfg(all(not(feature = "no_temperature_sensor"), not(feature = "pvt")))]
        temperature_sensing: TemperatureSensingConfig {
            adc: p.ADC1,
            pin1: p.PB1,
        },

        donut_hall: I2cHallConfig {
            peri: p.I2C1,
            scl: p.PB6,
            sda: p.PB7,
        },
        #[cfg(feature = "pvt")]
        ltc4332donut: LTC4332DonutConfig { cs: p.PA12 },
    };
    #[cfg(feature = "orbita2d")]
    let actuator_config = ActuatorConfig {
        b: VentouseConfig {
            peri: p.SPI4,
            sck: p.PE12,
            mosi: p.PE6,
            miso: p.PE5,
            foc_cs: p.PE3,
            foc_enable: p.PE0,
            driver_cs: p.PC15,
            driver_status_pin: p.PC14,
            #[cfg(feature = "ec45")]
            motor_config: BrushlessMotor::ec45(),
            #[cfg(feature = "ec60")]
            otor_config: BrushlessMotor::ec60(),
            #[cfg(feature = "beta")]
            current_sense_config: CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
            #[cfg(any(feature = "gamma", feature = "pvt"))]
            current_sense_config: CurrentSensing::ventouse_2d(), // current sense for gamma elec ventouse 2d
        },
        c: VentouseConfig {
            peri: p.SPI6,
            sck: p.PB3,
            mosi: p.PB5,
            miso: p.PB4,
            foc_cs: p.PD7,
            foc_enable: p.PD5,
            driver_cs: p.PD6,
            driver_status_pin: p.PD3,
            #[cfg(feature = "ec45")]
            motor_config: BrushlessMotor::ec45(),
            #[cfg(feature = "ec60")]
            otor_config: BrushlessMotor::ec60(),
            #[cfg(feature = "beta")]
            current_sense_config: CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
            #[cfg(any(feature = "gamma", feature = "pvt"))]
            current_sense_config: CurrentSensing::ventouse_2d(), // current sense for gamma elec ventouse 2d
        },

        aksim: AksimConfig { cs: p.PA15 },
        ad5047: AD5047Config { cs: p.PE4 },
        #[cfg(all(not(feature = "no_temperature_sensor"), feature = "pvt"))]
        temperature_sensing: TemperatureSensingConfig {
            adc: p.ADC1,
            pin1: p.PC5,
            pin2: p.PB0,
        },
        #[cfg(all(not(feature = "no_temperature_sensor"), not(feature = "pvt")))]
        temperature_sensing: TemperatureSensingConfig {
            adc: p.ADC1,
            pin1: p.PB1,
        },
        #[cfg(feature = "pvt")]
        ltc4332center: LTC4332CenterConfig { cs: p.PB9 },
        #[cfg(feature = "pvt")]
        ltc4332ring: LTC4332RingConfig { cs: p.PD1 },
    };

    unwrap!(spawner.spawn(motor_control::task::control_loop(
        actuator_config,
        hardware_zeros
    )));

    #[cfg(feature = "dynamixel")]
    {
        // Prepare and spawn the DXL communication task
        let mut usart_config = usart_config::default();
        // usart_config.baudrate = 1_000_000;
        // usart_config.baudrate = 115_200;
        usart_config.baudrate = 2_000_000;
        usart_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
        usart_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
        usart_config.parity = embassy_stm32::usart::Parity::ParityNone;
        usart_config.detect_previous_overrun = false;

        //Pouple A1
        // let usart = config::DynamixelUart::new(
        //     p.USART1,
        //     p.PB15, //RX
        //     p.PB14, //TX
        //     Irqs,
        //     p.DMA1_CH0,
        //     p.DMA1_CH1,
        //     usart_config,
        // )
        // .unwrap();
        // unwrap!(spawner.spawn(dynamixel::task::messsage_handler(usart, p.PD9.into(), board_id)));

        // Poulpe B1
        let usart = config::DynamixelUart::new(
            p.USART1,
            p.PB15, //RX
            p.PA9,  //TX
            Irqs,
            p.DMA1_CH0,
            p.DMA1_CH1,
            usart_config,
        )
        .unwrap();

        unwrap!(spawner.spawn(dynamixel::task::messsage_handler(
            usart,
            p.PD9.into(),
            board_id
        )));
    }

    // SPI for Ethercat LAN9252
    #[cfg(feature = "ethercat")]
    {
        let mut lan9252_spi_config = spi::Config::default();
        lan9252_spi_config.frequency = mhz(15);
        lan9252_spi_config.mode = spi::MODE_0;

        let ethconfig: LAN9252Config = EthercatConfig {
            peri: p.SPI3,
            sck: p.PC10,
            mosi: p.PB2,
            miso: p.PC11,
            cs: p.PD0,
        };

        unwrap!(spawner.spawn(ethercat::task::messsage_handler(
            ethconfig,
            lan9252_spi_config
        )));
    }
    // Prepare and spawn the main task
    let mut led_hello = Output::new(p.PC9, Level::High, Speed::Low);
    let mut led_error = Output::new(p.PC8, Level::High, Speed::Low);
    led_error.set_low(); //TODO
    led_hello.set_low();

    // the blinking is happening each 500ms
    // this number of blinks indicates the state of the board
    // as well as the color of the blinking
    //
    // state            | green         | red
    // -----------------|---------------|------
    // init             | blinks        | blinks
    // preop            | solid         | off
    // preop  + warning | solid         | blinks
    // op               | solid         | off
    // op  + warning    | solid         | blinks
    // fault            | off           | solid
    // fault_reaction   | off           | blinks
    //
    // TODO implement the blinking patterns to indicate different error states

    enum LedState {
        Off,
        Solid,
        Blink,
    }

    let mut red = LedState::Off; // 0 = off, 1 = solid, 2 = blink
    let mut green = LedState::Off; // 0 = off, 1 = solid, 2 = blink

    loop {
        let poulpe_state = { SHARED_MEMORY.lock().await.get_poulpe_state() };

        if poulpe_state.is_fault() {
            red = LedState::Solid;
            green = LedState::Off;
        } else if poulpe_state.is_fault_reaction_state() {
            red = LedState::Blink;
            green = LedState::Off;
        } else if poulpe_state.is_init() {
            red = LedState::Blink;
            green = LedState::Blink;
        } else if poulpe_state.is_preoperation_state() || poulpe_state.is_operation_enabled() {
            if poulpe_state.is_warning() {
                red = LedState::Blink;
                green = LedState::Solid;
            } else {
                red = LedState::Off;
                green = LedState::Solid;
            }
        } else {
            red = LedState::Off;
            green = LedState::Off;
        }

        match red {
            LedState::Off => led_error.set_low(),
            LedState::Solid => led_error.set_high(),
            LedState::Blink => led_error.toggle(),
        }

        match green {
            LedState::Off => led_hello.set_low(),
            LedState::Solid => led_hello.set_high(),
            LedState::Blink => led_hello.toggle(),
        }

        Timer::after(Duration::from_millis(500)).await;
    }
}
