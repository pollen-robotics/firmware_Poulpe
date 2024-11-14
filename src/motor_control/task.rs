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

use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_sync::pipe::ReadFuture;
use embassy_time::{block_for, Duration, Instant, Ticker, Timer};
use micromath::F32Ext;
use modular_bitfield::error;

const SPI_FREQ: u32 = 2_000_000;

use crate::state_machine::CiA402State;
use crate::{
    config::{self, ActuatorConfig, DonutHall},
    state_machine::poulpe_state::{HomingErrorFlag, MotorErrorFlag, PoulpeState},
    motor_control::{
        analog::AnalogInput,
        sensors::{AD5047Sensor, I2cHallSensor, SensorKind},
        RawSensorsIO,
    },
    IrqsI2c, SHARED_MEMORY,
};

use super::sensors;
use super::{
    sensors::AksimSensor,
    ventouse::{Ventouse, VentouseKind},
    Actuator, Foc, RawMotorsIO,
};

use super::driver::{DriverDRV8316, DriverTMC6200};


// this macro checks the temperatures of the motors and the boards
// if the temperature is too high, set the error state
// if the temperature is high, set the warning state
// if the temperature is back to normal, clear the warning state
// the function outputs the error and warning messages
macro_rules! verify_temperatures_and_update_state {
    (
        $board_state: ident,
        $board_temp: expr,
        $motor_temp: expr,
        $max_board_temp: expr,
        $max_motor_temp: expr,
        $high_temp: expr

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
            } else if $board_state.check_motor_error_flag(i, MotorErrorFlag::HighTemperatureWarning){
                // if the motor or the board temperature is back to normal, clear the warning state
                $board_state.clear_motor_error_flag(i, MotorErrorFlag::HighTemperatureWarning);   
                info!(
                    "Axis {} Temperature (motor: {}C, board: {}C) is back to normal!",
                    i, m, b
                );                 
            }
            // }
        }
    };
}


// macro updating the commuinication error status
// if the error is true, set the communication error flag and the timestamp
// if not reset the communication error flag
macro_rules! notify_communication_status {
    (   
        $error: ident, 
        $communication_down: ident, 
        $last_communication_timestamp: ident
    ) => {
        let now = Instant::now();
        if $error {
            // if this is the first time set the communication error flag and the timestamp
            // this is used to track the time since the last communication error
            if !$communication_down {
                $last_communication_timestamp = now;
            }
        } 
        $communication_down = $error;
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
            warn!($debug_message, limit, new_limit, limit_max);

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

    for (i, s) in moved_sensors.iter().enumerate() {
        diff[i] = *s - init_sensors[i];
        // if motor moved acors 0 the diff will be bigger around 2PI-diff
        if diff[i] > 3.141592 {
            diff[i] = diff[i] - 2.0 * 3.141592;
        } else if diff[i] < -3.141592 {
            diff[i] = diff[i] + 2.0 * 3.141592;
        }

        debug!("diff: {:?}", diff[i]);

        // #[cfg(feature = "ec45")]
        let should_move: [f32; 2] = [-0.15, 0.05];
        #[cfg(feature = "ec60")]
        let should_move: [f32; 2] = [-0.25, 0.09];

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

// read the axis sensors
// disables the torque to avoid the noise
// make a few tries to avoid nan values and errors
// if there is an error, return an error
pub async fn robust_read_axis_sensors<'d, const N: usize>(
    mut actuator: &mut Actuator<'d, N>,
    n_read_tries: u8,
    n_read: u8,
) -> Result<[f32; N], spi::Error> {
    // read the sensors - but disable the torque to avoid the noise
    actuator.set_torque([false; N]).unwrap();

    Timer::after(Duration::from_micros(100000)).await;

    let mut n_read_tries = n_read_tries;
    // make a few tries to avoid nan values:
    let sensor_reads = loop {
        n_read_tries = n_read_tries - 1;
        if n_read_tries == 0 {
            error!("Error reading axis sensors: too many tries (10), retrying...");
            return Err(spi::Error::ModeFault);
        }

        // We read n_read time the sensor and take the average and deviation to check if the sensor is stable
        let mut sensor_reads_avg: [f32; N] = [0.0; N];
        let mut sensor_reads_std: [f32; N] = [0.0; N];
        let mut sensor_reads_M2: [f32; N] = [0.0; N];

        let mut n: f32 = 0.0;
        for _ in 0..n_read {
            n = n + 1.0;
            match actuator.get_axis_sensors() {
                Ok(sensors) => {
                    if sensors.iter().any(|x| x.is_nan()) {
                        error!("Nan values in sensors, retrying...");
                        Timer::after(Duration::from_micros(100000)).await; // wait for a bit
                        continue;
                    }
                    // break sensors;
                    let mut delta: [f32; N] = [0.0; N];
                    for s in 0..N {
                        delta[s] = sensors[s] - sensor_reads_avg[s];
                        sensor_reads_avg[s] = sensor_reads_avg[s] + delta[s] / n;
                        sensor_reads_M2[s] = sensor_reads_M2[s]
                            + F32Ext::sqrt(delta[s] * (sensors[s] - sensor_reads_avg[s]));
                        sensor_reads_std[s] = sensor_reads_M2[s] / n;
                    }
                }
                Err(e) => {
                    error!("Error reading axis sensors: {:?}", e);
                    Timer::after(Duration::from_micros(100000)).await;
                    continue; //  retry the init if the read
                }
            }
        }
        info!(
            "Sensor avg: {:?} sensor deviation: {:?}",
            sensor_reads_avg, sensor_reads_std
        );
        let mut should_retry: bool = false;
        for s in 0..N {
            if sensor_reads_std[s] > 1e-3 {
                error!("Sensor deviation is to high!");
                should_retry = true;
            }
        }
        if should_retry {
            continue;
        }
        break sensor_reads_avg;
    };
    // read the sensors - but disable the torque to avoid the noise
    actuator.set_torque([true; N]).unwrap();
    // wait a bit to make sure the torque is enabled
    Timer::after(Duration::from_micros(100000)).await;

    Ok(sensor_reads)
}

//Find index for Orbita3D motors
#[cfg(feature = "orbita3d")]
pub async fn find_index_orbita3d<'d, const N: usize>(
    mut actuator: &mut Actuator<'d, N>,
    hardware_zeros: [f32; N],
    mut donut_hall: &mut DonutHall<'d>,
) -> Result<HomingErrorFlag, super::motors_io::IOError> {
    //FIXME:
    // - Maybe torque off is not so good, moving motor can induce motion in the torque off motor...

    // actuator.set_torque([false, false, false]).unwrap();

    let indices = actuator.find_index(&mut donut_hall).unwrap_or_else(|e| {
        error!("Error finding index: {:?}", e);
        [255; N]
    });
    info!("Found indices: {:?}", indices);
    //TODO retry if 255 or duplicate

    if indices.contains(&255) {
        // errors in finding the Hall
        error!("Bad index!");
        #[cfg(not(feature = "ignore_errors"))]
        return Ok(HomingErrorFlag::IndexSearchFail);
    }
    if (1..indices.len()).any(|i| indices[i..].contains(&indices[i - 1])) {
        //thanks Stackoverflow
        error!("Duplicate index!");
        #[cfg(not(feature = "ignore_errors"))]
        return Ok(HomingErrorFlag::IndexSearchFail);
    }

    actuator.set_index_sensor(indices);
    actuator.set_torque([false; N]).unwrap(); //be sure to torque off to avoid noise in axis sensors?
    block_for(Duration::from_millis(10));

    // let zeros = [1.0193205177783966, 0.7377220094203949, 0.4328247159719467]; //Orbita domain
    let zeros = hardware_zeros;

    if zeros[0] == zeros[1] && zeros[1] == zeros[2] && zeros[0] == 0.0 {
        //Forgot to pass the zeros as argument! FIXME switch to a different zeroing mode?
        // => assuming HallZero mode
        error!("No zero given in paramter! => HallZero mode");
        // Set the initial position to the axis sensor values (used for pc-side "sofwtare" zeroring )

        // read the current axis sensors positions
        let mut init_sensors = match robust_read_axis_sensors(&mut actuator, 10, 100).await {
            Ok(sensor_values) => {
                debug!("init sensors: {:?}", sensor_values);
                sensor_values
            }
            Err(e) => {
                error!("Error reading axis sensors: {:?}", e);
                #[cfg(not(feature = "ignore_errors"))]
                return Ok(HomingErrorFlag::AxisSensorReadFail);
                #[cfg(feature = "ignore_errors")]
                [0.0; N] // use the default value if ignoring errors
            }
        };

        init_sensors.iter_mut().for_each(|x| *x = wrap_to_pi(*x));
        debug!("init axis sensors: {:?}", init_sensors);
        match actuator.set_current_position(init_sensors) {
            Ok(_) => {
                SHARED_MEMORY
                    .lock()
                    .await
                    .set_current_position(init_sensors[..config::N_AXIS].try_into().unwrap());
                // stupid rust thing to convert N array to N_AXIS array (N = 3, N_AXIS = 3)
            }
            Err(e) => {
                error!("Error setting current position: {:?}", e);
                #[cfg(not(feature = "ignore_errors"))]
                return Ok(HomingErrorFlag::ZeroingFail);
            }
        }
        // #[cfg(feature = "orbita3d")]
        match actuator.set_target_position(init_sensors) {
            Ok(_) => {
                SHARED_MEMORY
                    .lock()
                    .await
                    .set_target_position(init_sensors[..config::N_AXIS].try_into().unwrap());
                // stupid rust thing to convert N array to N_AXIS array (N = 3, N_AXIS = 3)
            }
            Err(e) => {
                error!("Error setting target position: {:?}", e);
                #[cfg(not(feature = "ignore_errors"))]
                return Ok(HomingErrorFlag::ZeroingFail);
            }
        }
        return Ok(HomingErrorFlag::None);
    } else {
        info!("Hardware zeros: {:?}", zeros);
        let (mut offsets, found_turn) = actuator.compute_offset(indices, zeros).unwrap();

        if !(found_turn[0] == found_turn[1] && found_turn[1] == found_turn[2]) {
            //It may be possible in certain case?? But better forbid this
            error!("Incoherent number of turn found! {:?}", found_turn);
            #[cfg(not(feature = "ignore_errors"))]
            return Ok(HomingErrorFlag::ZeroingFail);
        }
        if offsets.iter().any(|&x| x.is_nan()) {
            // Check for NaN
            error!("Bad offsets! {:?}", offsets);
            #[cfg(not(feature = "ignore_errors"))]
            return Ok(HomingErrorFlag::ZeroingFail);
        }

        // read the current axis sensors positions
        let curpos = match robust_read_axis_sensors(&mut actuator, 10, 100).await {
            Ok(sensor_values) => {
                debug!("init sensors: {:?}", sensor_values);
                sensor_values
            }
            Err(e) => {
                error!("Error reading axis sensors: {:?}", e);
                #[cfg(not(feature = "ignore_errors"))]
                return Ok(HomingErrorFlag::AxisSensorReadFail);
                #[cfg(feature = "ignore_errors")]
                [0.0; N] // use the default value if ignoring errors
            }
        };

        offsets[0] *= -1.0 / config::BrushlessMotor::ecx22().axis_ratio();
        offsets[1] *= -1.0 / config::BrushlessMotor::ecx22().axis_ratio();
        offsets[2] *= -1.0 / config::BrushlessMotor::ecx22().axis_ratio();

        offsets[0] += curpos[0];
        offsets[1] += curpos[1];
        offsets[2] += curpos[2];

        debug!("indices: {:?} offsets: {:?}", indices, offsets);
        actuator.set_current_position(offsets)?;
        return Ok(HomingErrorFlag::None);
    }
}

#[cfg(feature = "orbita2d")]
pub async fn find_index_orbita2d<'d, const N: usize>(
    mut actuator: &mut Actuator<'d, N>,
    hardware_zeros: [f32; N],
) -> Result<HomingErrorFlag, super::motors_io::IOError> {
    actuator.set_torque([false; N])?; //be sure to torque off to avoid noise in axis sensors?
    block_for(Duration::from_millis(10));
    // let zeros = [5.236674785614014, 1.6637036800384521]; //Orbita domain
    let zeros = hardware_zeros;

    if zeros[0] == zeros[1] && zeros[0] == 0.0 {
        //Forgot to pass the zeros as argument! FIXME switch to a different zeroing mode?
        error!("No zero given in paramter! => Relative zero mode");
        // do nothing
    } else {
        info!("Hardware zeros: {:?}", zeros);
        // read the axis sensors
        let curaxis = match robust_read_axis_sensors(&mut actuator, 10, 100).await {
            Ok(sensor_values) => {
                debug!("init sensors: {:?}", sensor_values);
                sensor_values
            }
            Err(e) => {
                error!("Error reading axis sensors: {:?}", e);
                #[cfg(not(feature = "ignore_errors"))]
                return Ok(HomingErrorFlag::AxisSensorReadFail);
                #[cfg(feature = "ignore_errors")]
                [0.0; N] // use the default value if ignoring errors
            }
        };

        let r = 1.0; // no axis ratio for Orbita2D
        #[cfg(feature = "ec45")]
        let r = 1.0 / config::BrushlessMotor::ec45().axis_ratio();
        #[cfg(feature = "ec60")]
        let r = 1.0 / config::BrushlessMotor::ec60().axis_ratio();

        let axis_offset = [
            wrap_to_pi(curaxis[0] - zeros[0]),
            wrap_to_pi(curaxis[1] - zeros[1]),
        ];

        let mut motor_offsets = [0.0; N];
        // inverse kinematics
        motor_offsets[0] = -(r * axis_offset[0] + r * axis_offset[1]);
        motor_offsets[1] = -(r * axis_offset[0] - r * axis_offset[1]);

        // read the current motor posiitons
        let curpos = actuator.get_current_position()?;
        motor_offsets[0] += curpos[0];
        motor_offsets[1] += curpos[1];
        // set the offset
        actuator.set_current_position(motor_offsets)?;
    }
    return Ok(HomingErrorFlag::None);
}

pub async fn set_error_led() {
    SHARED_MEMORY.lock().await.set_error_led(true);
}

// function wrapping an angle in radians to
// the range [-pi, pi]
fn wrap_to_pi(angle: f32) -> f32 {
    const PI: f32 = 3.14159265359;
    (((angle + PI) % (2.0 * PI)) + (2.0 * PI)) % (2.0 * PI) - PI
}

#[embassy_executor::task]
pub async fn control_loop(config: ActuatorConfig, hardware_zeros: [f32; config::N_AXIS]) {
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
    #[cfg(all(feature = "gamma", feature = "orbita3d"))]
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
            config::BrushlessMotor::ecx22(),
            #[cfg(feature = "beta")]
            config::CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
            #[cfg(feature = "gamma")]
            config::CurrentSensing::ventouse_3d(), // current sense for gamma elec ventouse 2d
        );
        let driver_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.a.driver_cs, Level::High, Speed::Medium),
            driver_spi_config,
        );
        #[cfg(feature = "gamma")]
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
    let mut ad5047_spi_config = spi::Config::default();
    ad5047_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);
    ad5047_spi_config.mode = spi::MODE_1;

    ad5047_spi_config.bit_order = spi::BitOrder::MsbFirst;
    #[cfg(feature = "orbita2d")]
    let ad5047_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.ad5047.cs, Level::High, Speed::Medium),
        ad5047_spi_config,
    );
    #[cfg(feature = "orbita2d")]
    let mut ad5047 = AD5047Sensor::new(ad5047_spi);
    #[cfg(feature = "orbita2d")]
    ad5047.init().unwrap();
    #[cfg(feature = "orbita2d")]
    let mut ad5047 = SensorKind::Center(ad5047);

    //////////

    //Donut sensor BUS B

    let mut ad5047top_spi_config = spi::Config::default();

    ad5047top_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);

    ad5047top_spi_config.mode = spi::MODE_1;

    ad5047top_spi_config.bit_order = spi::BitOrder::MsbFirst;
    #[cfg(feature = "orbita3d")]
    let ad5047top_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.ad5047top.cs, Level::High, Speed::Medium),
        ad5047top_spi_config,
    );
    #[cfg(feature = "orbita3d")]
    let ad5047top = AD5047Sensor::new(ad5047top_spi);
    #[cfg(feature = "orbita3d")]
    let ad5047top = SensorKind::DonutTop(ad5047top);

    let mut ad5047mid_spi_config = spi::Config::default();

    ad5047mid_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);

    ad5047mid_spi_config.mode = spi::MODE_1;

    ad5047mid_spi_config.bit_order = spi::BitOrder::MsbFirst;
    #[cfg(feature = "orbita3d")]
    let ad5047mid_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.ad5047mid.cs, Level::High, Speed::Medium),
        ad5047mid_spi_config,
    );
    #[cfg(feature = "orbita3d")]
    let ad5047mid = AD5047Sensor::new(ad5047mid_spi);
    #[cfg(feature = "orbita3d")]
    let ad5047mid = SensorKind::DonutMid(ad5047mid);

    let mut ad5047bot_spi_config = spi::Config::default();

    ad5047bot_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);

    ad5047bot_spi_config.mode = spi::MODE_1;

    ad5047bot_spi_config.bit_order = spi::BitOrder::MsbFirst;
    #[cfg(feature = "orbita3d")]
    let ad5047bot_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.ad5047bot.cs, Level::High, Speed::Medium),
        ad5047bot_spi_config,
    );
    #[cfg(feature = "orbita3d")]
    let ad5047bot = AD5047Sensor::new(ad5047bot_spi);
    #[cfg(feature = "orbita3d")]
    let ad5047bot = SensorKind::DonutBot(ad5047bot);
    ///////////////

    let ventouse_b = {
        let foc_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.b.foc_cs, Level::High, Speed::Medium),
            foc_spi_config,
        );
        let foc = Foc::new(
            foc_spi,
            config.b.foc_enable,
            #[cfg(all(feature = "orbita2d", feature = "ec45"))]
            config::BrushlessMotor::ec45(),
            #[cfg(all(feature = "orbita2d", feature = "ec60"))]
            config::BrushlessMotor::ec60(),
            #[cfg(feature = "orbita3d")]
            config::BrushlessMotor::ecx22(),
            #[cfg(feature = "beta")]
            config::CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
            #[cfg(all(feature = "gamma", feature = "orbita2d"))]
            config::CurrentSensing::ventouse_2d(), // current sense for gamma elec ventouse 2d
            #[cfg(all(feature = "gamma", feature = "orbita3d"))]
            config::CurrentSensing::ventouse_3d(), // current sense for gamma elec ventouse 2d
        );

        let driver_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.b.driver_cs, Level::High, Speed::Medium),
            driver_spi_config,
        );

        #[cfg(all(feature = "orbita3d", feature = "gamma"))]
        let driver = DriverDRV8316::new(driver_spi, config.b.driver_status_pin);
        #[cfg(any(feature = "beta", all(feature = "orbita2d", feature = "gamma")))]
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
            #[cfg(all(feature = "orbita2d", feature = "ec45"))]
            config::BrushlessMotor::ec45(),
            #[cfg(all(feature = "orbita2d", feature = "ec60"))]
            config::BrushlessMotor::ec60(),
            #[cfg(feature = "orbita3d")]
            config::BrushlessMotor::ecx22(),
            #[cfg(feature = "beta")]
            config::CurrentSensing::ventouse_bob(), // current sense for the TMC BOB board
            #[cfg(all(feature = "gamma", feature = "orbita2d"))]
            config::CurrentSensing::ventouse_2d(), // current sense for gamma elec ventouse 2d
            #[cfg(all(feature = "gamma", feature = "orbita3d"))]
            config::CurrentSensing::ventouse_3d(), // current sense for gamma elec ventouse 2d
        );

        let driver_spi = SpiDeviceWithConfig::new(
            &spi_bus,
            Output::new(config.c.driver_cs, Level::High, Speed::Medium),
            driver_spi_config,
        );

        #[cfg(all(feature = "orbita3d", feature = "gamma"))]
        let driver = DriverDRV8316::new(driver_spi, config.c.driver_status_pin);
        #[cfg(any(feature = "beta", all(feature = "orbita2d", feature = "gamma")))]
        let driver = DriverTMC6200::new(driver_spi, config.c.driver_status_pin);

        let ventouse_c = Ventouse::new(foc, driver);
        VentouseKind::C(ventouse_c)
    };

    //Aksim sensor BUS C
    let mut aksim_spi_config = spi::Config::default();

    aksim_spi_config.frequency = embassy_stm32::time::Hertz(SPI_FREQ);

    aksim_spi_config.mode = spi::MODE_1;

    aksim_spi_config.bit_order = spi::BitOrder::MsbFirst;

    #[cfg(feature = "orbita2d")]
    let aksim_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.aksim.cs, Level::High, Speed::Medium),
        aksim_spi_config,
    );
    #[cfg(feature = "orbita2d")]
    let aksim = AksimSensor::new(aksim_spi);
    #[cfg(feature = "orbita2d")]
    let aksim = SensorKind::Ring(aksim);
    ////

    //Donut I2C Hall sensors
    #[cfg(feature = "orbita3d")]
    let i2c = I2c::new(
        config.donut_hall.peri,
        config.donut_hall.scl,
        config.donut_hall.sda,
        IrqsI2c,
        NoDma,
        NoDma,
        Hertz(100_000),
        Default::default(),
    );

    #[cfg(feature = "orbita3d")]
    let mut donut_hall = DonutHall::new(i2c);
    // let mut donut_hall=SensorKind::DonutHall(donut_hall);

    // let val=donut_hall.read();
    // match val {
    // 	Ok(val) => {
    // 	    info!("Donut sensor: {:#x}",val);
    // 	},
    // 	Err(e) => {
    // 	    error!("Donut sensor error: {:?}",e);
    // 	}
    // }

    // error!("Donut sensor: {:#x}",val);
    /////////

    // initialise the adc for motor temperature reading
    #[cfg(not(feature = "no_temperture_sensor"))]
    let mut motor_temperature_sensor = AnalogInput::new(config.temperature_sensor);

    // Setup the actuator with the configured ventouses
    #[cfg(all(feature = "orbita2d", feature = "gamma"))]
    let mut actuator = Actuator::new([ventouse_b, ventouse_c], [aksim, ad5047]);
    #[cfg(all(feature = "orbita2d", feature = "beta"))]
    // We invert motor_a and motor_b because of... mechanics
    let mut actuator = Actuator::new([ventouse_c, ventouse_b], [aksim, ad5047]);
    #[cfg(feature = "orbita3d")]
    let mut actuator = Actuator::new(
        [ventouse_a, ventouse_b, ventouse_c],
        [ad5047top, ad5047mid, ad5047bot],
    );

    // set the hardware zeros
    actuator.set_hardware_zeros(hardware_zeros);
    {SHARED_MEMORY.lock().await.set_hardware_zeros(hardware_zeros);};

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

        // read the axis sensors
        // this function makes a few tries to avoid nan values and errors
        // it disables the torque to avoid the noise (during the read - enable it after)
        // if there is an error, return an error
        let init_sensors = match robust_read_axis_sensors(&mut actuator, 10, 100).await {
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
        let moved_sensors = match robust_read_axis_sensors(&mut actuator, 10, 100).await {
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

        SHARED_MEMORY.lock().await.set_axis_sensor(moved_sensors);

        // motor check - move the motors in the other direction
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
        match find_index_orbita3d(&mut actuator, hardware_zeros, &mut donut_hall).await {
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
        match find_index_orbita2d(&mut actuator, hardware_zeros).await {
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

    let mut init_torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };
    let mut init_target_position = { SHARED_MEMORY.lock().await.get_target_position() };

    // a variable used for the safe fault handling
    let mut fault_response_counter = 5000; // 5000 loops at 1kHz = 5 seconds


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
    let mut loop_communication_error = false;
    // flag to track if the communicaiton is down at the moment
    let mut communication_down = false;
    // flag to track if the communication was down in the last loop
    let mut last_communication_timestamp = Instant::now();


    #[cfg(feature = "dynamixel")]
    {
        // set the state to switched on directly if dynamixel is used
        board_state.state_machine.set_state(CiA402State::SwitchedOn);
        { SHARED_MEMORY.lock().await.set_poulpe_state(board_state) };
    }


    loop {
        let t0 = Instant::now();
        // reset the communication problem flag
        loop_communication_error = false;

        let pos = actuator.get_current_position().unwrap_or_else(|e| {
            error!("Error reading position: {:?}", e);
            loop_communication_error = true;
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
            board_state.process_command(control_word);// if we are in the init state, we can only go to the switch on state
            if board_state.is_operation_enabled(){
                torque_on = [true;  config::N_AXIS];
            }else{
                torque_on = [false;  config::N_AXIS];
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
                // if there was a bus voltage error the operation stops but gently
                let stopping_velocity_limit = [0.1; config::N_AXIS];
                {
                    SHARED_MEMORY
                        .lock()
                        .await
                        .set_velocity_limit(stopping_velocity_limit)
                };

                // reduce the torque limit to 0 (from 1) over 5 seconds
                // this runs at 1kHz so it will take 5000 iterations
                let mut home_torque_limit =
                    { SHARED_MEMORY.lock().await.get_torque_flux_limit() };
                home_torque_limit.iter_mut().for_each(
                    |t| *t -= 0.0002, // 1/5000 = 0.0002 (5 seconds at 1kHz)
                );
                // update the fault response counter
                fault_response_counter -= 1;
                // if the torque limit is under 5% (0.05), the operation stops
                if home_torque_limit.iter().all(|t| *t < 0.05) || fault_response_counter <= 0 {
                    torque_on = [false; config::N_AXIS];
                    {
                        SHARED_MEMORY.lock().await.set_torque_on(torque_on)
                    };
                    info!("torque limit under 5%, stopping operation, {:?}", home_torque_limit);
                    // notify that the fault handling is done
                    // this will set the state to fault
                    board_state.notify_fault_reaction_done();
                } else {
                    // if the torque limit is not under 5%, set the new torque limit
                    SHARED_MEMORY
                        .lock()
                        .await
                        .set_torque_flux_limit(home_torque_limit)
                };
            }
        }

        // set the torque on if not already set
        if init_torque_on != torque_on {
            actuator.set_torque(torque_on).unwrap_or_else(|e| {
                error!("Error setting torque: {:?}", e);
                loop_communication_error = true;
            });
            init_torque_on = torque_on;
        }

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

        // set the target position (filtered or not)
        actuator.set_target_position(target).unwrap_or_else(|e| {
            error!("Error setting target pos: {:?}", e);
            loop_communication_error = true;
        });

        // Update torque-flux limits
        update_limit_setting!(
            actuator,                                           // orbita2d/3d
            get_torque_flux_limit,                              // shared memory getter
            get_torque_flux_limit_max,                          // shared memory getter
            init_torquefluxlimit,                               // previous value
            init_torquefluxlimit_max,                           // previous value
            set_torque_flux_limit,                              // actuator setter
            loop_communication_error,                           // error led flag
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
            loop_communication_error,                         // error led flag
            "Setting velocitylimit: {:?} => {:?} (max={:?})", // onchange log message
            "Error setting velocity limit: {:?}"              // error message
        );

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
                    loop_communication_error = true;
                });
        }

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
                loop_communication_error = true;
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
                loop_communication_error = true;
                error!("Vel error");
            }
        }

        // read the axis sensors and set the shared memory
        match actuator.get_axis_sensors() {
            Ok(sensors) => {
                if !sensors.iter().any(|s| s.is_nan()) {
                    //FIXME: hope it the sensor reading will better work to remove this
                    SHARED_MEMORY.lock().await.set_axis_sensor(sensors);
                }
            }
            Err(_e) => {
                // removed because of too much spamming
                loop_communication_error=true;
                error!("Axis sensors error");
            }
        }

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
                        {SHARED_MEMORY.lock().await.set_poulpe_state(board_state)};
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
                loop_communication_error = true;
                error!("Bus voltage reading error {:?}", e);
            }
        }
        
        // check the driver states 
        // if driver in fault state stop the operation
        let driver_states = actuator.check_driver_states();
        driver_states.iter().enumerate().for_each(|(i, driver_state)| {
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

                    loop_communication_error = true;
                    error!("Driver state reading error {:?}", e);
                }
            }
        });
        {SHARED_MEMORY.lock().await.set_poulpe_state(board_state)};
        

        // running the second (slow) task at slower rate (1Hz)
        if slow_timer == 0 {
            // update the flux pid gains
            update_actuator_setting!(
                actuator,                       // orbita2d/3d
                init_fluxpid,                   // previous value
                get_flux_pid_gains,             // shared memory getter
                set_flux_pid_gains,             // actuator setter
                loop_communication_error,       // error led flag
                "Error setting flux pid: {:?}"  // error message
            );
            // update the torque pid gains
            update_actuator_setting!(
                actuator,                         // orbita2d/3d
                init_torquepid,                   // previous value
                get_torque_pid_gains,             // shared memory getter
                set_torque_pid_gains,             // actuator setter
                loop_communication_error,         // error led flag
                "Error setting torque pid: {:?}"  // error message
            );
            // update the velocity pid gains
            update_actuator_setting!(
                actuator,                           // orbita2d/3d
                init_velocitypid,                   // previous value
                get_velocity_pid_gains,             // shared memory getter
                set_velocity_pid_gains,             // actuator setter
                loop_communication_error,           // error led flag
                "Error setting velocity pid: {:?}"  // error message
            );
            // update the position pid gains
            update_actuator_setting!(
                actuator,                           // orbita2d/3d
                init_positionpid,                   // previous value
                get_position_pid_gains,             // shared memory getter
                set_position_pid_gains,             // actuator setter
                loop_communication_error,           // error led flag
                "Error setting position pid: {:?}"  // error message
            );

            // update the uq/ud limit
            update_actuator_setting!(
                actuator,                          // orbita2d/3d
                init_uqudlimit,                    // previous value
                get_uq_ud_limit,                   // shared memory getter
                set_uq_ud_limit,                   // actuator setter
                loop_communication_error,          // error led flag
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
                    loop_communication_error = true;
                    error!("Board temperature reading error {:?}", e);
                }
            }

            let mut motor_temp = [0.0; config::N_AXIS];
            #[cfg(not(feature = "no_temperature_sensor"))]
            {
                // read the motor temperature
                match motor_temperature_sensor.read_temperature() {
                    Ok(t) => {
                        motor_temp = [t; config::N_AXIS];
                        {
                            SHARED_MEMORY.lock().await.set_motor_temperature(motor_temp)
                        };
                        info!("Motor temperature: {:?}", t);
                    }
                    Err(e) => {
                        loop_communication_error = true;
                        error!("Motor temperature reading error {:?}", e);
                    }
                }
            }

            // check the temperatures and set the error state if needed
            // this function will set the error state if the temperature is too high
            // it will set the warning state if the temperature is high 
            // it will update (set/reset) the necessary flags as well
            // it outputs the errorr and warning messages
            verify_temperatures_and_update_state!(
                board_state,                    // board state
                board_temp,                     // board temperature
                motor_temp,                     // motor temperature
                config::MAX_BOARD_TEMP,         // max board temperature
                config::MAX_MOTOR_TEMP,         // max motor temperature
                config::HIGH_TEMP               // high temperature 
            );

            // verify that the communication is working
            // - check if the communication was down and how long it was down
            // - if it's down for more than max allowed time, stop the operation
            if communication_down {
                let communication_down_duration = last_communication_timestamp.elapsed().as_secs() as u32;
                if (communication_down_duration > config::MAX_COMMUNICATION_DOWN_TIME) {
                    board_state.set_fault_state();
                    { SHARED_MEMORY.lock().await.set_poulpe_state(board_state) };
                    board_state.set_homing_error_flag(HomingErrorFlag::CommunicationFail);
                    error!(
                        "Communication error for {} secs ( max allowed {} secs), stopping operation!",
                        communication_down_duration,
                        config::MAX_COMMUNICATION_DOWN_TIME
                    );
                }
            }

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
            loop_communication_error,
            communication_down,
            last_communication_timestamp
        );

        // let elapsed=t0.elapsed().as_micros();
        // info!("Motor control loop elapsed: {} us",elapsed);
        // Timer::after(Duration::from_micros(1000-elapsed)).await;
        // Timer::after(Duration::from_millis(1)).await;
        ticker.next().await;
    }
}
