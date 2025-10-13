use core::cell::RefCell;

use cortex_m::register::basepri::read;
use defmt::{debug, error, info, warn};
use embassy_embedded_hal::shared_bus::blocking::{i2c::I2cDevice, spi::SpiDeviceWithConfig};
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
    spi,
};

use crate::sensors::axis_sensor;
use crate::utils::errors::IOError;

use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_sync::pipe::ReadFuture;
use embassy_time::{block_for, Duration, Instant, Ticker, Timer};
use micromath::F32Ext;
use modular_bitfield::error;

const SPI_FREQ: u32 = 2_000_000;
const SPI_FREQ_LTC: u32 = 1_000_000;

use crate::motor_control::foc::MotionMode;
use crate::state_machine::{CiA402State, CiA402StatusBit};
use crate::{
    config::{self, ActuatorConfig, DonutHall},
    sensors::{sensors::*, sensors_io::RawSensorsIO},
    state_machine::poulpe_state::{HomingErrorFlag, MotorErrorFlag, PoulpeState},
    IrqsI2c, SHARED_MEMORY,
};

#[cfg(not(feature = "no_temperature_sensor"))]
use crate::sensors::analog::{adc_read_temperature, adc_setup};

#[cfg(feature = "pvt")]
use crate::sensors::ltc4332::{LTC4332Config, LTC4332};

use super::actuator;
use super::{
    ventouse::{Ventouse, VentouseKind},
    Actuator, Foc, RawMotorsIO,
};

use super::driver::{DriverDRV8316, DriverTMC6200};

// macro that checks if the communication with the motor controller is down for too long
// or with an axis sensor
// if the communication is down for too long, set the fault state and stop the operation
macro_rules! verify_communication_timeout {
    (
        $communication_down:expr,
        $last_communication_timestamp:expr,
        $board_state:expr,
        $motor_error_flag:expr,
        $motor_id:expr,
        $homing_error_flag:expr,
        $error_message:expr
    ) => {
        if $communication_down {
            let communication_down_duration = $last_communication_timestamp.elapsed().as_secs() as u32;
            if communication_down_duration > config::MAX_COMMUNICATION_DOWN_TIME {
                $board_state.set_fault_state();
                {
                    SHARED_MEMORY.lock().await.set_poulpe_state($board_state)
                };
                if $homing_error_flag != HomingErrorFlag::None {
                    $board_state.set_homing_error_flag($homing_error_flag);
                }
                if $motor_error_flag != MotorErrorFlag::None {
                    $board_state.set_motor_error_flag($motor_id, $motor_error_flag);
                }
                error!(
                    "{} communication error for more than allowed {} secs, stopping operation!",
                    $error_message,
                    config::MAX_COMMUNICATION_DOWN_TIME
                );
            }
        }
    };
}

// macro that creates a new AD5047 sensor from the configuration
// the sensor is created with the SPI bus, the chip select pin and the SPI configuration
macro_rules! create_ad5047_from_config {
    (
        $spi_bus:expr, // the SPI bus
        $cs_pin:expr,  // the chip select pin
        $spi_config:expr  // the SPI configuration
    ) => {
        AD5047Sensor::new(SpiDeviceWithConfig::new(
            &$spi_bus,
            Output::new($cs_pin, Level::High, Speed::Medium),
            $spi_config,
        ));
    };
}

// macro that creates a new LTC4322 sensor from the configuration
// the sensor is created with the SPI bus, the chip select pin and the SPI configuration
macro_rules! create_ltc4332_from_config {
    (
        $spi_bus:expr, // the SPI bus
        $cs_pin:expr,  // the chip select pin
        $spi_config:expr  // the SPI configuration
    ) => {
        // $spi_config.mode = spi::MODE_0; // LTC4332 uses MODE0
        LTC4332::new(SpiDeviceWithConfig::new(
            &$spi_bus,
            Output::new($cs_pin, Level::High, Speed::Medium),
            $spi_config,
        ));
    };
}

// macro that creates a new Aksim2 sensor from the configuration
// the sensor is created with the SPI bus, the chip select pin and the SPI configuration
macro_rules! create_aksim_from_config {
    (
        $spi_bus:expr, // the SPI bus
        $cs_pin:expr,  // the chip select pin
        $spi_config:expr  // the SPI configuration
    ) => {
        AksimSensor::new(SpiDeviceWithConfig::new(
            &$spi_bus,
            Output::new($cs_pin, Level::High, Speed::Medium),
            $spi_config,
        ));
    };
}

// this macro checks the temperatures of the motors and the boards
// if the temperature is too high, set the error state
// if the temperature is high, set the warning state
// if temperature is too low, set the warning state (temperature sensor malfunction)
// if the temperature is back to normal, clear the warning state
// the function outputs the error and warning messages
macro_rules! verify_temperatures_and_update_state {
    (
        $board_state: ident,
        $board_temp: expr,
        $motor_temp: expr,
        $max_board_temp: expr,
        $max_motor_temp: expr,
        $high_temp: expr,
        $min_temp: expr

    ) => {// clear the warning state, it will be set in the next checks if true
        $board_state.clear_warning_state();
        // check the temperatures and set the error state if needed
        for (i, (b, m)) in $board_temp.iter().zip($motor_temp.iter()).enumerate() {
            if *b > $max_board_temp {
                // stop everything if the board temperature is too high
                // catastrophic error
                $board_state.set_motor_error_flag(i, MotorErrorFlag::OverTemperatureBoard);
                $board_state.set_fault_state();
                {SHARED_MEMORY.lock().await.set_poulpe_state($board_state)};
                error!(
                    "Max allowed board {} temperature exceeded : {}C (max {}C)!",
                    i, b, $max_board_temp
                );
            } else if *m > $max_motor_temp {
                // stop everything if the motor temperature is too high
                // catastrophic error
                $board_state.set_motor_error_flag(i, MotorErrorFlag::OverTemperatureMotor);
                $board_state.set_fault_state();
                {SHARED_MEMORY.lock().await.set_poulpe_state($board_state)};
                error!(
                    "Max allowed motor {} temperature exceeded : {}C (max {}C)!",
                    i,m, $max_motor_temp
                );
            // } else if !$board_state.is_fault() {
            }else if *b > $high_temp || *m > $high_temp {
                // if the board temperature is high, set the warning state
                $board_state.set_motor_error_flag(i, MotorErrorFlag::HighTemperatureWarning);
                $board_state.set_warning_state();
                warn!(
                    "Axis {} Temperature (motor: {}C, board: {}C) is very high (above {}C degrees)!",
                    i, m, b, $high_temp
                );
            }else if *b < $min_temp || *m < $min_temp  {
                // if the board temperature is high, set the warning state
                $board_state.set_motor_error_flag(i, MotorErrorFlag::TemperatureSensorMalfunctionWarning);
                $board_state.set_warning_state();
                warn!(
                    "Axis {} Temperature (motor: {}C, board: {}C) is very low (below -30C degrees)!",
                    i, m, b
                );
            } else {
                if $board_state.check_motor_error_flag(i, MotorErrorFlag::HighTemperatureWarning){
                    // if the motor or the board temperature is back to normal, clear the warning state
                    $board_state.clear_motor_error_flag(i, MotorErrorFlag::HighTemperatureWarning);
                    info!(
                        "Axis {} Temperature (motor: {}C, board: {}C) is back to normal!",
                        i, m, b
                    );
                   
                } 
                if $board_state.check_motor_error_flag(i, MotorErrorFlag::TemperatureSensorMalfunctionWarning){
                    // if the motor or board temperature sensor is back to normal, clear the warning state
                    $board_state.clear_motor_error_flag(i, MotorErrorFlag::TemperatureSensorMalfunctionWarning);
                    info!(
                        "Axis {} Temperature (motor: {}C, board: {}C) is back to normal!",
                        i, m, b
                    );
                } 
            }
        }
    };
}

// macro updating the commuinication error status
// if the error is true, set the communication error flag and the timestamp
// if not reset the communication error flag
macro_rules! notify_communication_status {
    (   
        $error: expr, 
        $driver_communication_down: expr, 
        $last_driver_communication_timestamp: expr
    ) => {
        let now = Instant::now();
        if $error {
            // if this is the first time set the communication error flag and the timestamp
            // this is used to track the time since the last communication error
            if !$driver_communication_down {
                $last_driver_communication_timestamp = now;
            }
        }
        $driver_communication_down = $error;
    };
}

// macro setting the actuator parameters
macro_rules! update_actuator_setting {
    (
        $actuator:ident, // orbita2d or orbita3d actuator
        $init_value:ident, // previous value
        $get_value:ident,   // shared memory function to get the value
        $set_function:ident,  // actuator function to set the value
        $error_led:ident,  // error led flag
        $error_message:expr // error message
    ) => {
        let value = { SHARED_MEMORY.lock().await.$get_value() };
        if value != $init_value {
            $actuator.$set_function(value).unwrap_or_else(|e| {
                error!($error_message, e);
                $error_led = true;
            });
            $init_value = value;
        }
    };
}
// macro setting the limit parameters
macro_rules! update_limit_setting {
    (
        $actuator:ident, // orbita2d or orbita3d actuator
        $get_limit:ident,  // shared memory function to get the limit
        $get_limit_max:ident,  // shared memory function to get the limit max
        $init_limit:ident,  // previous limit value
        $init_limit_max:ident, // previous limit max value
        $set_function:ident, // actuator function to set the limit
        $error_led:ident,  // error led flag
        $debug_message:expr,  // on set debug message
        $error_message:expr  // error message
    ) => {
        let limit = { SHARED_MEMORY.lock().await.$get_limit() };
        let limit_max = { SHARED_MEMORY.lock().await.$get_limit_max() };

        if limit != $init_limit || limit_max != $init_limit_max {
            let mut new_limit: [f32; config::N_AXIS] = [0.0; config::N_AXIS];
            limit.iter().enumerate().for_each(|(i, l)| {
                if *l <= 1.0 {
                    new_limit[i] = *l * limit_max[i] as f32;
                } else {
                    // Ensure we do not exceed the maximum limit
                    new_limit[i] = limit_max[i] as f32;
                }
            });
            //warn!($debug_message, limit, new_limit, limit_max);

            $actuator.$set_function(new_limit).unwrap_or_else(|e| {
                error!($error_message, e);
                $error_led = true;
            });

            $init_limit = limit;
            $init_limit_max = limit_max;
        }
    };
}

#[cfg(feature = "orbita3d")]
pub fn check_moved_sensors(moved_sensors: &[f32; 3], init_sensors: &[f32; 3]) -> [bool; 3] {
    let mut diff = [0.0; 3];
    // check if the sensors moved enough
    let mut moved_success: [bool; 3] = [true; 3];

    for (i, s) in moved_sensors.iter().enumerate() {
        diff[i] = *s - init_sensors[i];
        // if motor moved acors 0 the diff will be bigger around 2PI - diff
        if diff[i] > 3.141592 {
            diff[i] = diff[i] - 2.0 * 3.141592;
        } else if diff[i] < -3.141592 {
            diff[i] = diff[i] + 2.0 * 3.141592;
        }

        debug!("diff: {:?}", diff[i]);
        
        if (diff[i] <= 0.0 && diff[i] > -0.08) || (diff[i] > 0.0 || diff[i].is_nan()) {
            error!(
                "Axis sensor {:?} moved too little: {:?} Check sensor connection??",
                i, diff[i]
            );
            moved_success[i] = false;
        }
    }
    moved_success
}

#[cfg(feature = "orbita2d")]
pub fn check_moved_sensors(moved_sensors: &[f32; 2], init_sensors: &[f32; 2]) -> [bool; 2] {
    let mut diff = [0.0; 2];
    // check if the sensors moved enough
    let mut moved_success: [bool; 2] = [true; 2];

    // #[cfg(feature = "ec45")]
    let mut should_move: [f32; 2] = [-0.15, 0.05];
    #[cfg(feature = "ec60")]
    let should_move: [f32; 2] = [-0.25, 0.09];

    for (i, s) in moved_sensors.iter().enumerate() {
        diff[i] = *s - init_sensors[i];
        // if motor moved acors 0 the diff will be bigger around 2PI-diff
        if diff[i] > 3.141592 {
            diff[i] = diff[i] - 2.0 * 3.141592;
        } else if diff[i] < -3.141592 {
            diff[i] = diff[i] + 2.0 * 3.141592;
        }

        debug!("diff: {:?}", diff[i]);

        let delta = libm::fabs(should_move[i] as f64) as f32;
        if (diff[i] > should_move[i] + delta)
            || (diff[i] < should_move[i] - delta)
            || diff[i].is_nan()
        {
            error!(
                "Axis sensor {:?} moved too little: {:?} Check sensor connection??",
                i, diff[i]
            );
            moved_success[i] = false;
        }
        
    }
    moved_success
}


pub async fn set_error_led() {
    SHARED_MEMORY.lock().await.set_error_led(true);
}


#[embassy_executor::task]
pub async fn control_loop(mut config: ActuatorConfig) {


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
    #[cfg(all(any(feature = "gamma", feature = "pvt"), feature = "orbita3d"))]
    {
        driver_spi_config.mode = spi::MODE_1;
    }
    driver_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);
    driver_spi_config.bit_order = spi::BitOrder::MsbFirst;

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
    let ventouse_a = {
        let foc_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.a.foc_cs, Level::High, Speed::Medium),
            foc_spi_config,
        );
        let foc = Foc::new(
            foc_spi,
            config.a.foc_enable,
            config.a.motor_config,
            config.a.current_sense_config,
        );
        let driver_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.a.driver_cs, Level::High, Speed::Medium),
            driver_spi_config,
        );
        #[cfg(any(feature = "gamma", feature = "pvt"))]
        let driver = DriverDRV8316::new(driver_spi, config.a.driver_status_pin);
        #[cfg(feature = "beta")]
        let driver = DriverTMC6200::new(driver_spi, config.a.driver_status_pin);
        let ventouse_a = Ventouse::new(foc, driver);
        VentouseKind::A(ventouse_a)
    };

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

    //AD5047 center sensor BUS B
    /////////////
    let mut center_spi_config = spi::Config::default();
    center_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);
    center_spi_config.mode = spi::MODE_1; // AD5047 uses MODE1
    #[cfg(feature = "pvt")]
    {
        center_spi_config.mode = spi::MODE_0;
    } // override the mode - LTC4332 uses MODE0

    #[cfg(all(feature = "pvt", feature = "orbita2d"))]
    let mut center_ltc4332 =
        create_ltc4332_from_config!(spi_bus, config.ltc4332center.cs, center_spi_config);
    #[cfg(feature = "orbita2d")]
    let mut ad5047 = SensorKind::Center(create_ad5047_from_config!(
        spi_bus,
        config.ad5047.cs,
        center_spi_config
    ));

    //////////

    //Donut sensor BUS B

    let mut donut_spi_config = spi::Config::default();
    donut_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);
    donut_spi_config.bit_order = spi::BitOrder::MsbFirst;
    donut_spi_config.mode = spi::MODE_1; // AD5047 uses MODE1
    
    #[cfg(feature = "pvt")]
    {
        donut_spi_config.mode = spi::MODE_0;
    } // override the mode - LTC4332 uses MODE0

    #[cfg(all(feature = "pvt", feature = "orbita3d"))]
    let mut donut_ltc4332 =
        create_ltc4332_from_config!(spi_bus, config.ltc4332donut.cs, donut_spi_config);

    #[cfg(feature = "orbita3d")]
    let ad5047top = SensorKind::DonutTop(create_ad5047_from_config!(
        spi_bus,
        config.ad5047top.cs,
        donut_spi_config
    ));

    #[cfg(feature = "orbita3d")]
    let ad5047mid = SensorKind::DonutMid(create_ad5047_from_config!(
        spi_bus,
        config.ad5047mid.cs,
        donut_spi_config
    ));

    #[cfg(feature = "orbita3d")]
    let ad5047bot = SensorKind::DonutBot(create_ad5047_from_config!(
        spi_bus,
        config.ad5047bot.cs,
        donut_spi_config
    ));

    let ventouse_b = {
        let foc_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.b.foc_cs, Level::High, Speed::Medium),
            foc_spi_config,
        );
        let foc = Foc::new(
            foc_spi,
            config.b.foc_enable,
            config.b.motor_config,
            config.b.current_sense_config,
        );

        let driver_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.b.driver_cs, Level::High, Speed::Medium),
            driver_spi_config,
        );

        #[cfg(all(feature = "orbita3d", any(feature = "gamma", feature = "pvt")))]
        let driver = DriverDRV8316::new(driver_spi, config.b.driver_status_pin);
        #[cfg(any(
            feature = "beta",
            all(feature = "orbita2d", any(feature = "gamma", feature = "pvt"))
        ))]
        let driver = DriverTMC6200::new(driver_spi, config.b.driver_status_pin);

        let ventouse_b = Ventouse::new(foc, driver);
        VentouseKind::B(ventouse_b)
    };

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

    let ventouse_c = {
        let foc_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.c.foc_cs, Level::High, Speed::Medium),
            foc_spi_config,
        );
        let foc = Foc::new(
            foc_spi,
            config.c.foc_enable,
            config.c.motor_config,
            config.c.current_sense_config,
        );

        let driver_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.c.driver_cs, Level::High, Speed::Medium),
            driver_spi_config,
        );

        #[cfg(all(feature = "orbita3d", any(feature = "gamma", feature = "pvt")))]
        let driver = DriverDRV8316::new(driver_spi, config.c.driver_status_pin);
        #[cfg(any(
            feature = "beta",
            all(feature = "orbita2d", any(feature = "gamma", feature = "pvt"))
        ))]
        let driver = DriverTMC6200::new(driver_spi, config.c.driver_status_pin);

        let ventouse_c = Ventouse::new(foc, driver);
        VentouseKind::C(ventouse_c)
    };

    //Aksim sensor BUS C
    let mut ring_spi_config = spi::Config::default();
    ring_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ_LTC);
    ring_spi_config.bit_order = spi::BitOrder::MsbFirst;
    ring_spi_config.mode = spi::MODE_1; // Aksim2 uses MODE1
    #[cfg(feature = "pvt")]
    {
        ring_spi_config.mode = spi::MODE_0;
    } // override the mode - LTC4332 uses MODE0

    #[cfg(all(feature = "pvt", feature = "orbita2d"))]
    let mut ring_ltc4332 =
        create_ltc4332_from_config!(spi_bus, config.ltc4332ring.cs, ring_spi_config);

    #[cfg(feature = "orbita2d")]
    let aksim = SensorKind::Ring(create_aksim_from_config!(
        spi_bus,
        config.aksim.cs,
        ring_spi_config
    ));

    //Donut I2C Hall sensors
    #[cfg(feature = "orbita3d")]
    let mut donut_hall = DonutHall::new(I2c::new(
        config.donut_hall.peri,
        config.donut_hall.scl,
        config.donut_hall.sda,
        IrqsI2c,
        NoDma,
        NoDma,
        Hertz(100_000),
        Default::default(),
    ));

    // initialise the adc for motor temperature reading
    #[cfg(not(feature = "no_temperature_sensor"))]
    let mut motor_temperature_adc = adc_setup(&mut config.temperature_sensing.adc);

    // Setup the actuator with the configured ventouses
    #[cfg(all(feature = "orbita2d", any(feature = "gamma", feature = "pvt")))]
    let mut actuator = Actuator::new([ventouse_b, ventouse_c], [aksim, ad5047]);
    #[cfg(all(feature = "orbita2d", feature = "beta"))]
    // We invert motor_a and motor_b because of... mechanics
    let mut actuator = Actuator::new([ventouse_c, ventouse_b], [aksim, ad5047]);
    #[cfg(feature = "orbita3d")]
    let mut actuator = Actuator::new(
        [ventouse_a, ventouse_b, ventouse_c],
        [ad5047top, ad5047mid, ad5047bot],
    );

    // get the hardware zeros and the board id
    let hardware_zeros  = {SHARED_MEMORY.lock().await.get_hardware_zeros()};
    let board_id = {SHARED_MEMORY.lock().await.get_board_id()};

    // set the hardware zeros
    actuator.set_hardware_zeros(hardware_zeros);

    // trying to init the actuator
    // let mut init_error: BoardStatus = BoardStatus::Init;
    let mut board_state = PoulpeState::new();

    // initialization of the actuator (try two times)
    'init_loop: for try_i in 0..2 {
        info!("Init sequence try no. {:?}", try_i + 1);

        // go to the init state
        board_state.set_init_state();
        // clear previously set errors (in previous init try)
        board_state.clear_errors();

        // no error at the beginning
        {
            SHARED_MEMORY.lock().await.set_poulpe_state(board_state);
        };

        // wait for a random duration to avoid all the actuators to start at the same time
        block_for(Duration::from_millis(config::DXL_ID as u64 * 10));

        // configure the motors of the actuator
        let res_init = actuator.init().await;
        // verify that the motors are correctly configured
        res_init.iter().enumerate().for_each(|(motor_i, res)| {
            match res {
                Ok(_) => {
                    info!("Actuator {:?} init ok", motor_i);
                }
                Err(e) => {
                    // error on init
                    board_state.set_fault_state();
                    board_state.set_motor_error_flag(motor_i, MotorErrorFlag::ConfigFail);
                    error!("Actuator {:?} init error: {:?}", motor_i, e);
                }
            }
        });
        #[cfg(not(feature = "ignore_errors"))]
        if board_state.is_fault() {
            continue 'init_loop;
        }

        // configure axis sensors if PVT
        #[cfg(all(feature = "pvt", feature = "orbita3d"))]
        match donut_ltc4332.setup(LTC4332Config::Donut) {
            Ok(_) => {
                info!("Donut LTC4322 setup ok");
            }
            Err(e) => {
                board_state.set_fault_state();
                board_state.set_homing_error_flag(HomingErrorFlag::AxisSensorReadFail);
                error!("Donut LTC4322 setup error: {:?}", e);
                #[cfg(not(feature = "ignore_errors"))]
                continue 'init_loop;
            }
        }

        // read the axis sensors
        // this function makes a few tries to avoid nan values and errors
        // it disables the torque to avoid the noise (during the read - enable it after)
        // if there is an error, return an error
        let init_sensors = match actuator.robust_read_axis_sensors(10, 100).await {
            Ok(sensor_values) => {
                debug!("init sensors: {:?}", sensor_values);
                sensor_values
            }
            Err(e) => {
                board_state.set_fault_state();
                board_state.set_homing_error_flag(HomingErrorFlag::AxisSensorReadFail);
                error!("Error reading axis sensors: {:?}", e);
                [0.0; config::N_AXIS] // use the default value if ignoring errors
            }
        };
        // if there is an error, retry the init
        #[cfg(not(feature = "ignore_errors"))]
        if board_state.is_fault() {
            continue 'init_loop;
        }

        
        // motor check - move the motors and check if the sensors are moving
        let res_check1 = actuator.check_motors_1().await;

        // verify that the motors moved correctly
        res_check1
            .iter()
            .enumerate()
            .for_each(|(motor_i, res)| match res {
                Ok(_v) => {
                    info!("Motor {:?} check 1 ok", motor_i);
                }
                Err(e) => {
                    board_state.set_fault_state();
                    board_state.set_homing_error_flag(HomingErrorFlag::MotorMovementCheckFail);
                    board_state.set_motor_error_flag(motor_i, MotorErrorFlag::MotorAlignFail);
                    error!("Motor {:?} check 1 error: {:?}", motor_i, e);
                }
            });
        // if there is an error, retry the init
        #[cfg(not(feature = "ignore_errors"))]
        if board_state.is_fault() {
            continue 'init_loop;
        }

        // read the axis sensors
        // this function makes a few tries to avoid nan values and errors
        // it disables the torque to avoid the noise (during the read - enable it after)
        // if there is an error, return an error
        let moved_sensors = match actuator.robust_read_axis_sensors(10, 100).await {
            Ok(sensor_values) => {
                debug!("moved sensors: {:?}", sensor_values);
                sensor_values
            }
            Err(e) => {
                board_state.set_fault_state();
                board_state.set_homing_error_flag(HomingErrorFlag::AxisSensorReadFail);
                board_state.set_homing_error_flag(HomingErrorFlag::MotorMovementCheckFail);
                error!("Error reading axis sensors: {:?}", e);
                [0.0; config::N_AXIS] // use the default value if ignoring errors
            }
        };
        // if there is an error, retry the init
        #[cfg(not(feature = "ignore_errors"))]
        if board_state.is_fault() {
            continue 'init_loop;
        }

        {
            SHARED_MEMORY.lock().await.set_axis_sensor(moved_sensors);
        }

        // motor check - move the motors and check if the sensors are moving
        let res_check2 = actuator.check_motors_2().await;

        // verify that the motors moved correctly
        res_check2
            .iter()
            .enumerate()
            .for_each(|(motor_i, res)| match res {
                Ok(_v) => {
                    info!("Motor {:?} check 2 ok", motor_i);
                }
                Err(e) => {
                    board_state.set_fault_state();
                    board_state.set_homing_error_flag(HomingErrorFlag::MotorMovementCheckFail);
                    board_state.set_motor_error_flag(motor_i, MotorErrorFlag::MotorAlignFail);
                    error!("Motor {:?} check 2 error: {:?}", motor_i, e);
                }
            });
        // if there is an error, retry the init
        #[cfg(not(feature = "ignore_errors"))]
        if board_state.is_fault() {
            continue 'init_loop;
        }

        // verify that the sensors have moved
        // checking if the sensors are read properly and they are in the correct direction
        let move_check = check_moved_sensors(&moved_sensors, &init_sensors);
        move_check
            .iter()
            .enumerate()
            .for_each(|(motor_i, res)| match res {
                true => {
                    info!("Sensor {:?} align with motors check ok", motor_i);
                }
                false => {
                    board_state.set_fault_state();
                    board_state.set_homing_error_flag(HomingErrorFlag::AxisSensorAlignFail);
                    board_state.set_motor_error_flag(motor_i, MotorErrorFlag::MotorAlignFail);
                    error!("Sesnor {:?} align with motors check error!", motor_i);
                }
            });
        // if there is an error, retry the init
        #[cfg(not(feature = "ignore_errors"))]
        if board_state.is_fault() {
            continue 'init_loop;
        }

        // Find index for Orbita3D motors
        #[cfg(feature = "orbita3d")]
        match actuator.find_index_orbita3d(&mut donut_hall).await {
            Ok(homing_staus) => match homing_staus {
                HomingErrorFlag::None => {
                    info!("Index search and homing successfull!");
                }
                e => {
                    board_state.set_fault_state();
                    board_state.set_homing_error_flag(e);
                    error!("Error finding index: {:?}", e);
                }
            },
            Err(e) => {
                board_state.set_fault_state();
                board_state.set_homing_error_flag(HomingErrorFlag::ZeroingFail);
                error!("Error homing: {:?}", e);
            }
        }
        //Find zero for Orbita2D motors
        #[cfg(feature = "orbita2d")]
        match actuator.find_index_orbita2d().await {
            Ok(homing_staus) => match homing_staus {
                HomingErrorFlag::None => {
                    info!("Index search and homing successfull!");
                }
                e => {
                    board_state.set_fault_state();
                    board_state.set_homing_error_flag(e);
                    error!("Error finding index: {:?}", e);
                }
            },
            Err(e) => {
                board_state.set_fault_state();
                board_state.set_homing_error_flag(HomingErrorFlag::ZeroingFail);
                error!("Error homing: {:?}", e);
            }
        }
        // if there is an error, retry the init
        #[cfg(not(feature = "ignore_errors"))]
        if board_state.is_fault() {
            continue 'init_loop;
        }

        block_for(Duration::from_millis(100));
        #[cfg(feature = "orbita2d")]
        actuator.set_torque([false, false]).unwrap();

        // if no error during init, we can break the loop
        if !board_state.is_fault() {
            board_state.notify_init_success();
            break 'init_loop;
        }

        #[cfg(feature = "ignore_errors")]
        break 'init_loop; //  break the loop regardless of the error
    }

    // Print the error if there is one
    if board_state.is_fault() {
        error!("Error during init, stopping control loop!");
    } else {
        info!("Init successfull!");
    }

    let curpos = actuator.get_current_position().unwrap();
    let tarpos = actuator.get_target_position().unwrap();

    debug!(
        "Current position: {:?} target position: {:?}",
        curpos, tarpos
    );
    ////////// DEBUG

    // actuator.set_torque([true, true, true]).unwrap();
    // let axis = actuator.get_axis_sensors().unwrap();
    // let pos = actuator.get_current_position().unwrap();
    // let mut goal = pos.clone();
    // goal[0] += 1.0;
    // goal[1] += 1.0;
    // goal[2] += 1.0;
    // actuator.set_target_position(goal).unwrap();
    // Timer::after(Duration::from_millis(1000)).await;
    // let axis2 = actuator.get_axis_sensors().unwrap();
    // let pos2 = actuator.get_current_position().unwrap();
    // info!(
    //     "DEBUG: pos {:?}, axis: {:?} goal: {:?} pos2: {:?} axis2: {:?}",
    //     pos, axis, goal, pos2, axis2,
    // );
    //////////////

    // Init SharedMemory with real values before actually running the control loop
    SHARED_MEMORY.lock().await.init(&mut actuator);

    // rewrite the hardware zeros to the shared memory
    SHARED_MEMORY.lock().await.set_hardware_zeros(hardware_zeros);
    // rewrite the board id to the shared memory
    SHARED_MEMORY.lock().await.set_board_id(board_id);

    if board_state.is_fault() {
        SHARED_MEMORY.lock().await.set_error_led(true);
    }
    // set the state of the system
    {
        SHARED_MEMORY.lock().await.set_poulpe_state(board_state);
    };

    //"Slow" registers
    let mut init_fluxpid = { SHARED_MEMORY.lock().await.get_flux_pid_gains() };
    let mut init_torquepid = { SHARED_MEMORY.lock().await.get_torque_pid_gains() };
    let mut init_velocitypid = { SHARED_MEMORY.lock().await.get_velocity_pid_gains() };
    let mut init_positionpid = { SHARED_MEMORY.lock().await.get_position_pid_gains() };
    let mut init_uqudlimit = { SHARED_MEMORY.lock().await.get_uq_ud_limit() };
    let mut init_torquefluxlimit = { SHARED_MEMORY.lock().await.get_torque_flux_limit() };
    let mut init_velocitylimit = { SHARED_MEMORY.lock().await.get_velocity_limit() };

    let mut init_torquefluxlimit_max = { SHARED_MEMORY.lock().await.get_torque_flux_limit_max() };
    let mut init_velocitylimit_max = { SHARED_MEMORY.lock().await.get_velocity_limit_max() };

    let mut init_control_mode = { SHARED_MEMORY.lock().await.get_control_mode() };

    let mut init_torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };
    let mut init_target_position = { SHARED_MEMORY.lock().await.get_target_position() };

    // a variable used for the safe fault handling
    let emergency_stop_response_counter_max: usize = 7000; // 7secs (1000 loops at 1kHz)
    let mut quick_stop_response_counter  = 0;
    let mut fault_response_counter = 0; 

    // actuator.set_torque([false,false]).unwrap();
    let mut error_led = false;
    let mut prev_error_led = false;
    if board_state.is_fault() {
        error_led = true;
        prev_error_led = true;
    }

    use biquad::*;
    let f0 = 10.hz();
    let fs = 1.khz();

    // Create coefficients for the biquads
    let coeffs =
        Coefficients::<f32>::from_params(Type::LowPass, fs, f0, Q_BUTTERWORTH_F32).unwrap();

    // Create two different biquads
    // let mut biquad = DirectForm1::<f32>::new(coeffs);
    // let mut biquad = DirectForm2Transposed::<f32>::new(coeffs);
    let mut torque_filter = [DirectForm2Transposed::<f32>::new(coeffs); config::N_AXIS];
    let mut vel_filter = [DirectForm2Transposed::<f32>::new(coeffs); config::N_AXIS];

    let mut cmd_filter = [DirectForm2Transposed::<f32>::new(coeffs); config::N_AXIS];
    // velocity feedforward filter
    let f0_ff = 30.hz();
    let coeffs_vel =
        Coefficients::<f32>::from_params(Type::LowPass, fs, f0_ff, Q_BUTTERWORTH_F32).unwrap();
    let mut vel_ff_filter = [DirectForm2Transposed::<f32>::new(coeffs_vel); config::N_AXIS];

    let mut slow_timer: u32 = 1000;
    let mut ticker = Ticker::every(Duration::from_micros(1000));

    // communication error tracking flags
    // flag to track if there was a communication error in the loop
    let mut loop_driver_communication_error = false;
    let mut loop_axis_sensors_communication_error = [false; config::N_AXIS];
    // flag to track if the communicaiton is down at the moment
    let mut driver_communication_down = false;
    let mut axis_sensors_communication_down = [false; config::N_AXIS];
    // flag to track if the communication was down in the last loop
    let mut last_driver_communication_timestamp = Instant::now();
    let mut last_axis_sensors_communication_timestamp = [Instant::now(); config::N_AXIS];

    #[cfg(feature = "dynamixel")]
    {
        // set the state to switched on directly if dynamixel is used
        board_state.state_machine.set_state(CiA402State::SwitchedOn);
        {
            SHARED_MEMORY.lock().await.set_poulpe_state(board_state)
        };
    }

    // set inital control mode to positon
    {
        SHARED_MEMORY
            .lock()
            .await
            .set_control_mode(MotionMode::Position)
    };

    let mut t0 = Instant::now();
    loop {
        let t_loop = Instant::now();
        // reset the communication problem flag
        loop_driver_communication_error = false;
        loop_axis_sensors_communication_error = [false; config::N_AXIS];

        let pos = actuator.get_current_position().unwrap_or_else(|e| {
            error!("Error reading position: {:?}", e);
            loop_driver_communication_error = true;
            [f32::NAN; config::N_AXIS]
        });
        {
            // warn!("ELAPSED 0 {:?}",t0.elapsed().as_micros());
            // info!("pos: {:?}", pos);
            SHARED_MEMORY.lock().await.set_current_position(pos);
            // warn!("ELAPSED 1 {:?}",t0.elapsed().as_micros());
        }

        // initialise the torque on variable to the value in the previous loop
        let mut torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };
        let mut board_state = { SHARED_MEMORY.lock().await.get_poulpe_state() };

        #[cfg(feature = "ethercat")]
        {
            // process the commands
            let control_word = { SHARED_MEMORY.lock().await.get_control_word() };
            board_state.process_command(control_word); // if we are in the init state, we can only go to the switch on state
            if board_state.is_torque_enabled() {
                torque_on = [true; config::N_AXIS];
            } else {
                torque_on = [false; config::N_AXIS];
            }
        }

        // verify that the board state is not in an error state
        #[cfg(not(feature = "ignore_errors"))] // if errors are ignored the operation continues
        {
            if board_state.is_fault() {
                // if there was an init error the operation stops and cannot restart
                torque_on = [false; config::N_AXIS];
                {
                    SHARED_MEMORY.lock().await.set_torque_on(torque_on)
                };
            } else if board_state.is_fault_reaction_state() {

                // set the control mode to stopped
                // thic control mode will brake the motor producing a torque in the opposite direction of the velocity
                {SHARED_MEMORY.lock().await.set_control_mode(MotionMode::Stopped);}
                // if mode change is not allowed, do it here directly
                // otherwise, the control mode will be changed later in code
                #[cfg(not(feature = "allow_mode_change"))]
                {
                    actuator.set_control_mode(MotionMode::Stopped).unwrap_or_else(|e| {
                        error!("Error setting control mode: {:?}", e);
                        loop_driver_communication_error = true;
                    });
                }
                
                // get the latest velociyt and torque
                let velocity = {SHARED_MEMORY.lock().await.get_current_velocity()};
                let torque = {SHARED_MEMORY.lock().await.get_current_torque()};


                // update the fault response counter
                fault_response_counter +=1;
                if fault_response_counter % 500 == 0 {
                    warn!("Fault reaction active, velocity: {:?}, torque {}, quick stop time {}", velocity, torque, fault_response_counter as f32 * 0.001);
                }

                // if velocity is almost zero and the torque is almost zero, stop the operation
                // dont stop if the fault response counter is not yet at the maximum
                // and stop right away if the fault response counter is at the two times the maximum (emergency stop)
                if (velocity.iter().all(|v| v.abs() < 0.05)) && 
                    (torque.iter().all(|t| t.abs() < 100.0) && 
                    (fault_response_counter >= emergency_stop_response_counter_max)) || 
                    fault_response_counter >= 2*emergency_stop_response_counter_max {
                    torque_on = [false; config::N_AXIS];
                    {
                        SHARED_MEMORY.lock().await.set_torque_on(torque_on)
                    };
                    warn!(
                        "Fault response done, stopping operation",
                    );
                    // notify that the fault handling is done
                    // this will set the state to fault
                    board_state.notify_fault_reaction_done();
                }
            }
        }

        // handle the quick stop command
        #[cfg(feature = "allow_quickstop")]
        {
            if board_state.is_quick_stop_active() {

                // set the control mode to stopped
                // thic control mode will brake the motor producing a torque in the opposite direction of the velocity
                {SHARED_MEMORY.lock().await.set_control_mode(MotionMode::Stopped);}
                // if mode change is not allowed, do it here directly
                // otherwise, the control mode will be changed later in code
                #[cfg(not(feature = "allow_mode_change"))]
                {
                    actuator.set_control_mode(MotionMode::Stopped).unwrap_or_else(|e| {
                        error!("Error setting control mode: {:?}", e);
                        loop_driver_communication_error = true;
                    });
                }
                
                // get the latest velociyt and torque
                let velocity = {SHARED_MEMORY.lock().await.get_current_velocity()};
                let torque = {SHARED_MEMORY.lock().await.get_current_torque()};

                // update the counter
                quick_stop_response_counter +=1;
                if quick_stop_response_counter % 500 == 0 {
                    warn!("Quick stop active, velocity: {:?}, torque {}, quick stop time {}", velocity, torque, quick_stop_response_counter as f32 * 0.001);
                }

                // if velocity is almost zero and the torque is almost zero, stop the operation
                // dont stop if the fault response counter is not yet at the maximum
                // and stop right away if the fault response counter is at the two times the maximum (emergency stop)
                if (velocity.iter().all(|v| v.abs() < 0.05)) && 
                    (torque.iter().all(|t| t.abs() < 100.0) && 
                    (quick_stop_response_counter >= emergency_stop_response_counter_max)) ||
                    quick_stop_response_counter >= 2*emergency_stop_response_counter_max {
                    torque_on = [false; config::N_AXIS];
                    {
                        SHARED_MEMORY.lock().await.set_torque_on(torque_on)
                    };
                    // go back to the position mode only if the mode change is not allowed
                    #[cfg(not(feature = "allow_mode_change"))]
                    {
                        {SHARED_MEMORY.lock().await.set_control_mode(MotionMode::Position);}
                        actuator.set_control_mode(MotionMode::Position).unwrap_or_else(|e| {
                            error!("Error setting control mode: {:?}", e);
                            loop_driver_communication_error = true;
                        });
                    }
                    warn!(
                        "Quick stop done, stopping operation",
                    );
                    // notify that the quick stop is done
                    // this will set the state to switched on disabled
                    board_state.notify_quick_stop_done();
                    // allow the new quick stop to be activated later if needed
                    quick_stop_response_counter = 0;
                }
            }
        }

        // set the torque on if not already set
        if init_torque_on != torque_on {
            actuator.set_torque(torque_on).unwrap_or_else(|e| {
                error!("Error setting torque: {:?}", e);
                loop_driver_communication_error = true;
            });
            init_torque_on = torque_on;
        }

        let mut control_mode = { SHARED_MEMORY.lock().await.get_control_mode() };
        #[cfg(feature = "allow_mode_change")]
        {
            if init_control_mode.to_u8() != control_mode.to_u8() {
                actuator.set_control_mode(control_mode).unwrap_or_else(|e| {
                    error!("Error setting control mode: {:?}", e);
                    loop_driver_communication_error = true;
                });
                init_control_mode = control_mode;
            }
        }

        match actuator.get_control_mode(){
            Ok(mode) => {
                { SHARED_MEMORY.lock().await.set_control_mode_display(mode[0]) };
                control_mode = mode[0];
            },
            Err(e) => {
                error!("Error reading control mode: {:?}", e);
                loop_driver_communication_error = true;
            }
        }


        match control_mode {
            MotionMode::Position => {
                //Unfiltered
                #[cfg(not(feature = "cmd_filter"))]
                let target = { SHARED_MEMORY.lock().await.get_target_position() };

                //Filtered
                #[cfg(feature = "cmd_filter")]
                let target = {
                    let mut t = { SHARED_MEMORY.lock().await.get_target_position() };
                    t.iter_mut().enumerate().for_each(|(i, t)| {
                        if !torque_on[i] {
                            //Trick to make the filter converge => reset target
                            for _ in 0..1000 {
                                cmd_filter[i].run(*t);
                            }
                        }
                        *t = cmd_filter[i].run(*t)
                    });
                    t
                };

                // add the feedforward control to the velocity loop
                #[cfg(feature = "velocity_feedforward")]
                {
                    // velocity feedforward from shared memory
                    let mut velocity_ff = { SHARED_MEMORY.lock().await.get_velocity_feedforward() };

                    // get velocity feedforward timestamp
                    let velocity_ff_timestamp = {
                        SHARED_MEMORY
                            .lock()
                            .await
                            .get_velocity_feedforward_timestamp()
                    };
                    // check if the velocity feedforward value has been set and is it too old (older than 200ms)
                    match velocity_ff_timestamp {
                        Some(timestamp) => {
                            if timestamp.elapsed().as_millis() > 200 {
                                velocity_ff = [0.0; config::N_AXIS];
                            }
                        }
                        None => {
                            velocity_ff = [0.0; config::N_AXIS];
                        }
                    }

                    // filter the velocity feedforward
                    velocity_ff
                        .iter_mut()
                        .enumerate()
                        .for_each(|(i, v)| *v = vel_ff_filter[i].run(*v));
                    // set the velocity feedforward
                    actuator
                        .set_velocity_feedforward(velocity_ff)
                        .unwrap_or_else(|e| {
                            error!("Error setting velocity feedforward: {:?}", e);
                            loop_driver_communication_error = true;
                        });
                }

                // set the target position (filtered or not)
                actuator.set_target_position(target).unwrap_or_else(|e| {
                    error!("Error setting target pos: {:?}", e);
                    loop_driver_communication_error = true;
                });

                
                let torque_ff = { SHARED_MEMORY.lock().await.get_target_torque() };
                actuator.set_torque_feedforward(torque_ff).unwrap_or_else(|e| {
                    error!("Error setting torque feedforward: {:?}", e);
                    loop_driver_communication_error = true;
                });
            }
            MotionMode::Torque => {
                let target = { SHARED_MEMORY.lock().await.get_target_torque() };
                // set target torque
                actuator.set_target_torque(target).unwrap_or_else(|e| {
                    error!("Error setting target torque: {:?}", e);
                    loop_driver_communication_error = true;
                });
            }
            MotionMode::Velocity => {
                let target = { SHARED_MEMORY.lock().await.get_target_velocity() };
                // set target velocity
                actuator.set_target_velocity(target).unwrap_or_else(|e| {
                    error!("Error setting target velocity: {:?}", e);
                    loop_driver_communication_error = true;
                });
            }
            _ => {}
        }

        // set the target position (filtered or not)
        // actuator.set_target_position(target).unwrap_or_else(|e| {
        //     error!("Error setting target pos: {:?}", e);
        //     loop_driver_communication_error = true;
        // });

        // Update torque-flux limits
        update_limit_setting!(
            actuator,                                           // orbita2d/3d
            get_torque_flux_limit,                              // shared memory getter
            get_torque_flux_limit_max,                          // shared memory getter
            init_torquefluxlimit,                               // previous value
            init_torquefluxlimit_max,                           // previous value
            set_torque_flux_limit,                              // actuator setter
            loop_driver_communication_error,                           // error led flag
            "Setting torquefluxlimit: {:?} => {:?} (max={:?})", // onchange log message
            "Error setting torque/flux limit: {:?}"             // error message
        );

        // Update velocity limits
        update_limit_setting!(
            actuator,                                         // orbita2d/3d
            get_velocity_limit,                               // shared memory getter
            get_velocity_limit_max,                           // shared memory getter
            init_velocitylimit,                               // previous value
            init_velocitylimit_max,                           // previous value
            set_velocity_limit,                               // actuator setter
            loop_driver_communication_error,                         // error led flag
            "Setting velocitylimit: {:?} => {:?} (max={:?})", // onchange log message
            "Error setting velocity limit: {:?}"              // error message
        );

        // read the real-time values

        //  read the torque and set the shared memory
        match actuator.get_current_torque() {
            Ok(mut torque) => {
                torque
                    .iter_mut()
                    .enumerate()
                    .for_each(|(i, t)| *t = torque_filter[i].run(*t));
                SHARED_MEMORY.lock().await.set_current_torque(torque);
            }
            Err(_e) => {
                loop_driver_communication_error = true;
                error!("Torque error");
            }
        }

        // read the velocity and set the shared memory
        match actuator.get_current_velocity() {
            Ok(mut vel) => {
                vel.iter_mut()
                    .enumerate()
                    .for_each(|(i, t)| *t = vel_filter[i].run(*t));
                SHARED_MEMORY.lock().await.set_current_velocity(vel);
            }
            Err(_e) => {
                loop_driver_communication_error = true;
                error!("Vel error");
            }
        }

        // read the axis sensors and set the shared memory
        let mut sensor_vars = {SHARED_MEMORY.lock().await.get_axis_sensor()};
        let axis_sensor_ret = actuator.get_axis_sensors();
        axis_sensor_ret
            .iter()
            .enumerate()
            .for_each(|(i, sensor)| {
                match sensor {
                    Ok(sensor) => {
                        if !sensor.is_nan() {
                            //FIXME: hope it the sensor reading will better work to remove this
                            sensor_vars[i] = *sensor;
                        }
                    }
                    Err(_e) => {
                        loop_axis_sensors_communication_error[i] = true;
                        error!("Axis sensor {} reading error: {:?}", i, _e);
                    }
                }
            });
        // update the shared memory with the (possibly) new sensor values
        {SHARED_MEMORY.lock().await.set_axis_sensor(sensor_vars)};

        // read the dc bus current and
        // set the error led if there was an error
        if error_led != prev_error_led {
            SHARED_MEMORY.lock().await.set_error_led(error_led);
            prev_error_led = error_led;
        }

        // get dc bus voltage
        match actuator.get_bus_voltage() {
            Ok(v) => {
                {
                    SHARED_MEMORY.lock().await.set_bus_voltage(v)
                };

                for (i, bus_volt) in v.iter().enumerate() {
                    if *bus_volt < config::MIN_BUS_VOLTAGE {
                        // stop everything if the bus voltage is too low
                        // catastrophic error
                        // no recovery - the board needs to be restarted
                        board_state.set_motor_error_flag(i, MotorErrorFlag::LowBusVoltage);
                        board_state.set_fault_state();
                        {
                            SHARED_MEMORY.lock().await.set_poulpe_state(board_state)
                        };
                        error!(
                            "Bus voltage {}V is too low (under {}V)!",
                            bus_volt,
                            config::MIN_BUS_VOLTAGE
                        );
                    }
                }
                debug!("Bus voltage: {:?}", v);
            }
            Err(e) => {
                loop_driver_communication_error = true;
                error!("Bus voltage reading error {:?}", e);
            }
        }

        // check the driver states
        // if driver in fault state stop the operation
        let driver_states = actuator.check_driver_states();
        driver_states
            .iter()
            .enumerate()
            .for_each(|(i, driver_state)| {
                match driver_state {
                    Ok(driver_state) => {
                        debug!("Driver state: OK");
                    }
                    Err(e) => {
                        // this is a catastrophic error
                        // the driver state is not read correctly
                        // the operation needs to stop
                        board_state.set_fault_state();
                        board_state.set_motor_error_flag(i, MotorErrorFlag::DriverFault);

                        //loop_driver_communication_error = true;
                        error!("Driver state reading error {:?}", e);
                    }
                }
            });
        {
            SHARED_MEMORY.lock().await.set_poulpe_state(board_state)
        };

        // running the second (slow) task at slower rate (1Hz)
        if slow_timer == 0 {
            // update the flux pid gains
            update_actuator_setting!(
                actuator,                       // orbita2d/3d
                init_fluxpid,                   // previous value
                get_flux_pid_gains,             // shared memory getter
                set_flux_pid_gains,             // actuator setter
                loop_driver_communication_error,       // error led flag
                "Error setting flux pid: {:?}"  // error message
            );
            // update the torque pid gains
            update_actuator_setting!(
                actuator,                         // orbita2d/3d
                init_torquepid,                   // previous value
                get_torque_pid_gains,             // shared memory getter
                set_torque_pid_gains,             // actuator setter
                loop_driver_communication_error,         // error led flag
                "Error setting torque pid: {:?}"  // error message
            );
            // update the velocity pid gains
            update_actuator_setting!(
                actuator,                           // orbita2d/3d
                init_velocitypid,                   // previous value
                get_velocity_pid_gains,             // shared memory getter
                set_velocity_pid_gains,             // actuator setter
                loop_driver_communication_error,           // error led flag
                "Error setting velocity pid: {:?}"  // error message
            );
            // update the position pid gains
            update_actuator_setting!(
                actuator,                           // orbita2d/3d
                init_positionpid,                   // previous value
                get_position_pid_gains,             // shared memory getter
                set_position_pid_gains,             // actuator setter
                loop_driver_communication_error,           // error led flag
                "Error setting position pid: {:?}"  // error message
            );

            // update the uq/ud limit
            update_actuator_setting!(
                actuator,                          // orbita2d/3d
                init_uqudlimit,                    // previous value
                get_uq_ud_limit,                   // shared memory getter
                set_uq_ud_limit,                   // actuator setter
                loop_driver_communication_error,          // error led flag
                "Error setting uq/ud limit: {:?}"  // error message
            );

            // perform checks on the actuator to determine the error state
            let mut board_temp = [0.0; config::N_AXIS];
            // get temperature
            match actuator.get_board_temperature() {
                Ok(t) => {
                    // save the temperatures
                    {
                        SHARED_MEMORY.lock().await.set_board_temperature(t)
                    };
                    // find the max temperature
                    board_temp = t;
                    info!("Board temperature: {:?}", t);
                }
                Err(e) => {
                    loop_driver_communication_error = true;
                    error!("Board temperature reading error {:?}", e);
                }
            }

            let mut motor_temp = [0.0; config::N_AXIS];
            #[cfg(not(feature = "no_temperature_sensor"))]
            {
                // get motor temperatures
                motor_temp[0] = adc_read_temperature(
                    &mut motor_temperature_adc,
                    &mut config.temperature_sensing.pin1,
                )
                .unwrap_or(motor_temp[0]);
                #[cfg(feature = "pvt")]
                {
                    motor_temp[1] = adc_read_temperature(
                        &mut motor_temperature_adc,
                        &mut config.temperature_sensing.pin2,
                    )
                    .unwrap_or(motor_temp[1]);
                }
                #[cfg(all(feature = "pvt", feature = "orbita3d"))]
                {
                    motor_temp[2] = adc_read_temperature(
                        &mut motor_temperature_adc,
                        &mut config.temperature_sensing.pin3,
                    )
                    .unwrap_or(motor_temp[2]);
                }
                #[cfg(not(feature = "pvt"))]
                {
                    motor_temp = [motor_temp[0]; config::N_AXIS];
                }
                info!("Motor temperature: {:?}", motor_temp);
                {
                    SHARED_MEMORY.lock().await.set_motor_temperature(motor_temp)
                };
            }

            // check the temperatures and set the error state if needed
            // this function will set the error state if the temperature is too high
            // it will set the warning state if the temperature is high
            // it will update (set/reset) the necessary flags as well
            // it outputs the errorr and warning messages
            verify_temperatures_and_update_state!(
                board_state,            // board state
                board_temp,             // board temperature
                motor_temp,             // motor temperature
                config::MAX_BOARD_TEMP, // max board temperature
                config::MAX_MOTOR_TEMP, // max motor temperature
                config::HIGH_TEMP,      // high temperature
                config::MIN_TEMP        // min measurable temperature
            );

            // verify that the communication is working
            // - check if the communication was down and how long it was down
            // - if it's down for more than max allowed time, stop the operation
            verify_communication_timeout!(
                driver_communication_down,
                last_driver_communication_timestamp, 
                board_state,
                MotorErrorFlag::None,
                0,
                HomingErrorFlag::LowLevelCommunicaiton,
                "Driver"
            );

            for i in 0..config::N_AXIS {
            // verify communication with axis sensors
                verify_communication_timeout!(
                    axis_sensors_communication_down[i],
                    last_axis_sensors_communication_timestamp[i],
                    board_state,
                    MotorErrorFlag::AxisSensorCommunicationFail,
                    i,
                    HomingErrorFlag::None,
                    "Axis sensor"
                );
            }



        
        // if the low level comunication error is already set, check which driver it corresponds to
        // and set the driver communication error only for that driver
        if board_state.get_homing_error_flags().contains(&Some(HomingErrorFlag::LowLevelCommunicaiton)) {
            let driver_is_communicating = actuator.check_driver_communication();
            driver_is_communicating.iter().enumerate().for_each(|(i, is_communicating)| {
                if !*is_communicating {
                    // if the driver is not communicating, set the communication error flag for that driver
                    board_state.set_motor_error_flag(i, MotorErrorFlag::DriverCommunicationFail);
                    error!("Driver {} communication fail", i);
                }
            });
        }
        {
            SHARED_MEMORY.lock().await.set_poulpe_state(board_state)
        };


            // set the error led to active
            if board_state.is_fault() {
                error_led = true;
            }

            // dispaly current state
            if board_state.is_fault() {
                error!("Board state: {:?}", board_state);
            } else if board_state.is_warning() {
                warn!("Board state: {:?}", board_state);
            } else {
                info!("Board state: {:?}", board_state);
            }

            slow_timer = 1000;
        } else {
            slow_timer -= 1;
        }

        {
            SHARED_MEMORY.lock().await.set_poulpe_state(board_state)
        };

        // verify if there was a communication problem in this loop
        notify_communication_status!(
            loop_driver_communication_error,
            driver_communication_down,
            last_driver_communication_timestamp
        );

        for i in 0..config::N_AXIS {
            // verify communication with axis sensors
            notify_communication_status!(
                loop_axis_sensors_communication_error[i],
                axis_sensors_communication_down[i],
                last_axis_sensors_communication_timestamp[i]
            );
        }

        #[cfg(feature = "debug_execution_time")]
        {
            let elapsed = t_loop.elapsed().as_micros();
            warn!("Motor control loop elapsed time: {}us \t time between loops: {}us",elapsed, t0.elapsed().as_micros());
            t0 = Instant::now();
        }
        
        // TODO do not remove this delay
        // it is necessary for thread sincronization
        // I dont understand why though
        Timer::after(Duration::from_micros(1)).await;
        // 1ms frequency
        ticker.next().await;
    }
}
