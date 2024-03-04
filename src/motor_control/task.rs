use core::cell::RefCell;

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
use embassy_time::{block_for, Duration, Instant, Ticker, Timer};

const SPI_FREQ: u32 = 2_000_000;

use crate::{
    config::{self, ActuatorConfig, DonutHall},
    motor_control::{
        sensors::{AD5047Sensor, I2cHallSensor, SensorKind},
        BoardStatus, RawSensorsIO,
    },
    IrqsI2c, SHARED_MEMORY,
};

use super::{
    sensors::AksimSensor,
    ventouse::{Ventouse, VentouseKind},
    Actuator, Driver, Foc, RawMotorsIO,
};

pub async fn set_error_led() {
    SHARED_MEMORY.lock().await.set_error_led(true);
}

#[embassy_executor::task]
pub async fn control_loop(config: ActuatorConfig) {
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
        // current sense for wailer B2 board
        config::CurrentSensing::wailer_B2(),
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
    let ad5047 = AD5047Sensor::new(ad5047_spi);
    #[cfg(feature = "orbita2d")]
    let ad5047 = SensorKind::Center(ad5047);

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
        // current sense for wailer B2 board
        config::CurrentSensing::wailer_B2(),
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
        #[cfg(all(feature = "orbita2d", feature = "ec45"))]
        config::BrushlessMotor::ec45(),
        #[cfg(all(feature = "orbita2d", feature = "ec60"))]
        config::BrushlessMotor::ec60(),
        #[cfg(feature = "orbita3d")]
        config::BrushlessMotor::ecx22(),
        // current sense for wailer B2 board
        config::CurrentSensing::wailer_B2(),
    );

    let driver_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.c.driver_cs, Level::High, Speed::Medium),
        driver_spi_config,
    );
    let driver = Driver::new(driver_spi);

    let ventouse_c = Ventouse::new(foc, driver);
    let ventouse_c = VentouseKind::C(ventouse_c);

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

    // Setup the actuator with the configured ventouses
    #[cfg(feature = "orbita2d")]
    // let mut actuator = Actuator::new([ventouse_b, ventouse_c], [aksim, ad5047]);
    //We invert motor_a and motor_b because of... mechanics
    let mut actuator = Actuator::new([ventouse_c, ventouse_b], [aksim, ad5047]);
    #[cfg(feature = "orbita3d")]
    let mut actuator = Actuator::new(
        [ventouse_a, ventouse_b, ventouse_c],
        [ad5047top, ad5047mid, ad5047bot],
    );

    // trying to init the actuator
    let mut init_error: BoardStatus = BoardStatus::Ok;

    // initialization of the actuator (try two times)
    for try_i in 0..2 {
        info!("Initialization try no. {:?}", try_i + 1);
        // no error at the beginning
        init_error = BoardStatus::Ok;

        //wait for a random duration to avoid all the actuators to start at the same time
        block_for(Duration::from_millis(config::DXL_ID as u64 * 10));

        let res_init = actuator.init().await;
        match res_init {
            Ok(_v) => {
                info!("Registers init ok");
            }
            Err(e) => {
                // error on init
                init_error = BoardStatus::InitError;
                error!("Registers init error: {:?}", e);
                continue; //  retry the init if there is an error
            }
        }

        // read the axis sensors - but disable the torque to avoid the noise
        #[cfg(feature = "orbita2d")]
        actuator.set_torque([false, false]).unwrap(); //FIXME: axis sensors are too noisy when torque is on
        #[cfg(feature = "orbita3d")]
        actuator.set_torque([false, false, false]).unwrap(); //FIXME: axis sensors are too noisy when torque is on
                                                             // #[cfg(feature = "orbita2d")]
        Timer::after(Duration::from_micros(100000)).await;

        let init_sensors = actuator.get_axis_sensors().unwrap();
        // #[cfg(feature = "orbita2d")]
        Timer::after(Duration::from_micros(100000)).await;
        #[cfg(feature = "orbita2d")]
        actuator.set_torque([true, true]).unwrap();
        #[cfg(feature = "orbita3d")]
        actuator.set_torque([true, true, true]).unwrap();

        // motor check - move the motors and check if the sensors are moving
        let res = actuator.check_motors_1().await;
        match res {
            Ok(_v) => {}
            Err(e) => {
                init_error = BoardStatus::InitError;
                error!("Motor check 1 error: {:?}", e);
                continue; //  retry the init if there is an error
            }
        }

        // read the sensors - but disable the torque to avoid the noise
        #[cfg(feature = "orbita2d")]
        actuator.set_torque([false, false]).unwrap(); //FIXME: axis sensors are too noisy when torque is on
        #[cfg(feature = "orbita3d")]
        actuator.set_torque([false, false, false]).unwrap(); //FIXME: axis sensors are too noisy when torque is on

        Timer::after(Duration::from_micros(100000)).await;

        let moved_sensors = actuator.get_axis_sensors().unwrap();
        SHARED_MEMORY.lock().await.set_axis_sensor(moved_sensors);

        Timer::after(Duration::from_micros(100000)).await;
        // enable torques
        #[cfg(feature = "orbita2d")]
        actuator.set_torque([true, true]).unwrap();
        #[cfg(feature = "orbita3d")]
        actuator.set_torque([true, true, true]).unwrap();

        // motor check - move the motors in the other direction
        let res = actuator.check_motors_2().await;
        match res {
            Ok(_v) => {}
            Err(e) => {
                init_error = BoardStatus::InitError;
                error!("Motor check 2 error: {:?}", e);
                continue; //  retry the init if there is an error
            }
        }

        // verify that the sensors have moved
        // checking if the sensors are read properly and they are in the correct direction
        let mut diff = [0.0; config::N_AXIS];
        #[cfg(feature = "orbita3d")]
        {
            for (i, s) in moved_sensors.iter().enumerate() {
                diff[i] = *s - init_sensors[i];
                if diff[i] > 3.141592 {
                    diff[i] = diff[i] - 2.0 * 3.141592;
                }

                if (diff[i] <= 0.0 && diff[i] > -0.08) || (diff[i] > 0.0 || diff[i].is_nan()) {
                    error!(
                        "Axis sensor {:?} moved too little: {:?} Check sensor connection??",
                        i, diff[i]
                    );
                    init_error = BoardStatus::SensorError;
                }
            }
        }
        #[cfg(feature = "orbita2d")]
        {
            for (i, s) in moved_sensors.iter().enumerate() {
                diff[i] = *s - init_sensors[i];
                if diff[i] > 3.141592 {
                    diff[i] = diff[i] - 2.0 * 3.141592;
                }

                // //WTF? We cannot use abs() for f32?
                // let absdiff: f32 = if diff[i].is_sign_positive() {
                //     diff[i]
                // } else {
                //     -diff[i]
                // };

                if i == 0 {
                    if (diff[i] > -0.1) || diff[i].is_nan() {
                        error!(
                            "Axis sensor {:?} moved too little: {:?} Check sensor connection??",
                            i, diff[i]
                        );
                        init_error = BoardStatus::SensorError;
                    }
                }
                if i == 1 {
                    if (diff[i] < 0.05) || diff[i].is_nan() {
                        error!(
                            "Axis sensor {:?} moved too little: {:?} Check sensor connection??",
                            i, diff[i]
                        );
                        init_error = BoardStatus::SensorError;
                    }
                }
            }
        }

        //Find index for Orbita3D motors
        #[cfg(feature = "orbita3d")]
        {
            //FIXME:
            // - Maybe torque off is not so good, moving motor can induce motion in the torque off motor...

            // actuator.set_torque([false, false, false]).unwrap();

            let indices = actuator.find_index(&mut donut_hall).unwrap_or_else(|e| {
                error!("Error finding index: {:?}", e);
                init_error = BoardStatus::IndexError;
                [255; config::N_AXIS]
            });
            info!("Found indices: {:?}", indices);
            //TODO retry if 255 or duplicate

            if indices.contains(&255)
            // errors in finding the Hall
            {
                error!("Bad index!");
                continue; //Retry
            }
            if (1..indices.len()).any(|i| indices[i..].contains(&indices[i - 1])) {
                //thanks Stackoverflow
                error!("Duplicate index!");
                continue; //Retry
            }
            actuator.set_index_sensor(indices);
            actuator.set_torque([false, false, false]).unwrap(); //be sure to torque off to avoid noise in axis sensors?
            block_for(Duration::from_millis(10));
            // let zeros = [1.0193205177783966, 0.7377220094203949, 0.4328247159719467]; //Orbita domain
            let zeros = config::HARDWARE_ZEROS;

            if zeros[0] == zeros[1] && zeros[1] == zeros[2] && zeros[0] == 0.0 {
                //Forgot to pass the zeros as argument! FIXME switch to a different zeroing mode?
                // => assuming HallZero mode
                error!("No zero given in paramter! => HallZero mode");
                // Set the initial position to the axis sensor values (used for pc-side "sofwtare" zeroring )

                let init_sensors = actuator.get_axis_sensors().unwrap();
                debug!("init axis sensors: {:?}", init_sensors);
                let res = actuator.set_current_position(init_sensors);

                match res {
                    Ok(_) => {
                        SHARED_MEMORY
                            .lock()
                            .await
                            .set_current_position(init_sensors);
                    }
                    Err(e) => {
                        init_error = BoardStatus::ZeroingError;
                        error!("Error setting current position: {:?}", e);
                    }
                }
                // #[cfg(feature = "orbita3d")]
                let res = actuator.set_target_position(init_sensors);
                // #[cfg(feature = "orbita3d")]
                match res {
                    Ok(_) => {
                        SHARED_MEMORY.lock().await.set_target_position(init_sensors);
                    }
                    Err(e) => {
                        init_error = BoardStatus::ZeroingError;
                        error!("Error setting target position: {:?}", e);
                    }
                }
            } else {
                info!("Hardware zeros: {:?}", zeros);
                let (mut offsets, found_turn) = actuator.compute_offset(indices, zeros).unwrap();

                if !(found_turn[0] == found_turn[1] && found_turn[1] == found_turn[2]) {
                    //It may be possible in certain case?? But better forbid this
                    error!("Incoherent number of turn found! {:?}", found_turn);
                    continue;
                }
                if offsets.iter().any(|&x| x.is_nan()) {
                    // Check for NaN
                    error!("Bad offsets! {:?}", offsets);
                    continue;
                }

                let curpos = actuator.get_axis_sensors().unwrap();

                offsets[0] *= -1.0 / config::BrushlessMotor::ecx22().axis_ratio();
                offsets[1] *= -1.0 / config::BrushlessMotor::ecx22().axis_ratio();
                offsets[2] *= -1.0 / config::BrushlessMotor::ecx22().axis_ratio();

                offsets[0] += curpos[0];
                offsets[1] += curpos[1];
                offsets[2] += curpos[2];

                debug!("indices: {:?} offsets: {:?}", indices, offsets);
                actuator.set_current_position(offsets);
            }
        }

        block_for(Duration::from_millis(100));
        #[cfg(feature = "orbita2d")]
        actuator.set_torque([false, false]).unwrap();

        // if no error during init, we can break the loop
        if init_error == BoardStatus::Ok {
            debug!("init sensors: {:?}", init_sensors);
            debug!("moved sensors: {:?}", moved_sensors);
            debug!("diff sensors: {:?}", diff);
            break;
        }
    }

    // Print the error if there is one
    if init_error == BoardStatus::Ok {
        info!("Init successfull!");
    } else {
        error!("Error during init, stopping control loop!");
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
    if init_error != BoardStatus::Ok {
        SHARED_MEMORY.lock().await.set_error_led(true);
    }
    // set the error state of the system
    {
        SHARED_MEMORY.lock().await.set_error_state(init_error)
    };

    //"Slow" registers
    let mut init_fluxpid = { SHARED_MEMORY.lock().await.get_flux_pid_gains() };
    let mut init_torquepid = { SHARED_MEMORY.lock().await.get_torque_pid_gains() };
    let mut init_velocitypid = { SHARED_MEMORY.lock().await.get_velocity_pid_gains() };
    let mut init_positionpid = { SHARED_MEMORY.lock().await.get_position_pid_gains() };
    let mut init_uqudlimit = { SHARED_MEMORY.lock().await.get_uq_ud_limit() };
    let mut init_torquefluxlimit = { SHARED_MEMORY.lock().await.get_torque_flux_limit() };
    let mut init_velocitylimit = { SHARED_MEMORY.lock().await.get_velocity_limit() };

    let mut init_torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };
    let mut init_target_position = { SHARED_MEMORY.lock().await.get_target_position() };

    // actuator.set_torque([false,false]).unwrap();
    let mut error_led = false;
    let mut prev_error_led = false;
    if init_error != BoardStatus::Ok {
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

    loop {
        let pos = actuator.get_current_position().unwrap_or_else(|e| {
            error!("Error reading position: {:?}", e);
            error_led = true;
            [f32::NAN; config::N_AXIS]
        });
        {
            // warn!("ELAPSED 0 {:?}",t0.elapsed().as_micros());
            // info!("pos: {:?}", pos);
            SHARED_MEMORY.lock().await.set_current_position(pos);
            // warn!("ELAPSED 1 {:?}",t0.elapsed().as_micros());
        }

        let mut torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };
        let mut error_state = { SHARED_MEMORY.lock().await.get_error_state() };
        if error_state != BoardStatus::Ok {
            // if init error, we turn off the torque
            torque_on = [false; config::N_AXIS];
            {
                SHARED_MEMORY.lock().await.set_torque_on(torque_on)
            };
        }
        actuator.set_torque(torque_on).unwrap_or_else(|e| {
            error!("Error setting torque: {:?}", e);
            error_led = true;
        });

        //Unfiltered
        #[cfg(not(feature = "cmd_filter"))]
        let target = { SHARED_MEMORY.lock().await.get_target_position() };

        //Filtered
        let mut target = { SHARED_MEMORY.lock().await.get_target_position() };
        #[cfg(feature = "cmd_filter")]
        target.iter_mut().enumerate().for_each(|(i, t)| {
            if !torque_on[i] {
                //Trick to make the filter converge => reset target
                for _ in 0..1000 {
                    cmd_filter[i].run(*t);
                }
            }
            *t = cmd_filter[i].run(*t)
        });

        actuator.set_target_position(target).unwrap_or_else(|e| {
            error!("Error setting target pos: {:?}", e);
            error_led = true;
        });

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
                    error_led = true;
                });
        }

        let torque = actuator.get_current_torque();
        match torque {
            Ok(mut torque) => {
                torque
                    .iter_mut()
                    .enumerate()
                    .for_each(|(i, t)| *t = torque_filter[i].run(*t));
                SHARED_MEMORY.lock().await.set_current_torque(torque);
                // info!("sensors: {:?}", sensors);
            }
            Err(_e) => {
                error_led = true;
                error!("Torque error");
            }
        }

        let vel = actuator.get_current_velocity();
        match vel {
            Ok(mut vel) => {
                vel.iter_mut()
                    .enumerate()
                    .for_each(|(i, t)| *t = vel_filter[i].run(*t));
                SHARED_MEMORY.lock().await.set_current_velocity(vel);
                // info!("sensors: {:?}", sensors);
            }
            Err(_e) => {
                error_led = true;
                error!("Vel error");
            }
        }
        // warn!("ELAPSED 6 {:?}",t0.elapsed().as_micros());

        let sensors = actuator.get_axis_sensors();
        match sensors {
            Ok(sensors) => {
                if !sensors.iter().any(|s| s.is_nan()) {
                    //FIXME: hope it the sensor reading will better work to remove this
                    SHARED_MEMORY.lock().await.set_axis_sensor(sensors);
                }

                // info!("sensors: {:?}", sensors);
            }
            Err(_e) => {
                // SHARED_MEMORY.lock().await.set_axis_sensor([999999.0, 999999.0]);
                // error_led=true;
                // error!("Axis sensors error");
            }
        }

        if error_led != prev_error_led {
            SHARED_MEMORY.lock().await.set_error_led(error_led);
            prev_error_led = error_led;
        }

        if slow_timer == 0 {
            //PID and limits: only for debug

            let fluxpid = { SHARED_MEMORY.lock().await.get_flux_pid_gains() };
            if fluxpid != init_fluxpid {
                actuator.set_flux_pid_gains(fluxpid).unwrap_or_else(|e| {
                    error!("Error setting flux pid: {:?}", e);
                    error_led = true;
                });
                init_fluxpid = fluxpid;
            }

            let torquepid = { SHARED_MEMORY.lock().await.get_torque_pid_gains() };
            if torquepid != init_torquepid {
                actuator
                    .set_torque_pid_gains(torquepid)
                    .unwrap_or_else(|e| {
                        error!("Error setting torque pid: {:?}", e);
                        error_led = true;
                    });
                init_torquepid = torquepid;
            }

            let velocitypid = { SHARED_MEMORY.lock().await.get_velocity_pid_gains() };
            if velocitypid != init_velocitypid {
                actuator
                    .set_velocity_pid_gains(velocitypid)
                    .unwrap_or_else(|e| {
                        error!("Error setting velocity pid: {:?}", e);
                        error_led = true;
                    });
                init_velocitypid = velocitypid;
            }

            let positionpid = { SHARED_MEMORY.lock().await.get_position_pid_gains() };
            if positionpid != init_positionpid {
                actuator
                    .set_position_pid_gains(positionpid)
                    .unwrap_or_else(|e| {
                        error!("Error setting position pid: {:?}", e);
                        error_led = true;
                    });
                init_positionpid = positionpid;
            }

            let uqudlimit = { SHARED_MEMORY.lock().await.get_uq_ud_limit() };
            if uqudlimit != init_uqudlimit {
                actuator.set_uq_ud_limit(uqudlimit).unwrap_or_else(|e| {
                    error!("Error setting uq/ud limit: {:?}", e);
                    error_led = true;
                });
                init_uqudlimit = uqudlimit;
            }

            let torquefluxlimit = { SHARED_MEMORY.lock().await.get_torque_flux_limit() };
            if torquefluxlimit != init_torquefluxlimit {
                actuator
                    .set_torque_flux_limit(torquefluxlimit)
                    .unwrap_or_else(|e| {
                        error!("Error setting torque/flux limit: {:?}", e);
                        error_led = true;
                    });
                init_torquefluxlimit = torquefluxlimit;
            }

            let velocitylimit = { SHARED_MEMORY.lock().await.get_velocity_limit() };
            if velocitylimit != init_velocitylimit {
                actuator
                    .set_velocity_limit(velocitylimit)
                    .unwrap_or_else(|e| {
                        error!("Error setting velocity limit: {:?}", e);
                        error_led = true;
                    });
                init_velocitylimit = velocitylimit;
            }

            // //Less error at lower frequency...
            // let sensors=actuator.get_axis_sensors();
            // match sensors {
            // 	Ok(sensors) => {
            // 	    SHARED_MEMORY.lock().await.set_axis_sensor(sensors);
            // 	    info!("sensors: {:?}", sensors);
            // 	},
            // 	Err(_e) => {
            // 	    // SHARED_MEMORY.lock().await.set_axis_sensor([999999.0, 999999.0]);
            // 	    error_led=true;
            // 	    error!("Axis sensors error");
            // 	}
            // }

            slow_timer = 1000;
        } else {
            slow_timer -= 1;
        }

        // let elapsed=t0.elapsed().as_micros();
        // warn!("ELAPSED: {:?}",elapsed);
        // Timer::after(Duration::from_micros(1000-elapsed)).await;
        // Timer::after(Duration::from_millis(1)).await;
        ticker.next().await;
    }
}
