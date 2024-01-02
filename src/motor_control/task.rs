use core::cell::RefCell;

use defmt::{info, error, debug, warn};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
    spi,
};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Duration, Timer, block_for, Instant};

const SPI_FREQ: u32 = 2_000_000;


use crate::{
    config::{self, ActuatorConfig},
    SHARED_MEMORY, motor_control::{sensors::{AD5047Sensor, SensorKind}, RawSensorsIO},
};

use super::{
    ventouse::{Ventouse, VentouseKind},
    Actuator, Driver, Foc, RawMotorsIO, sensors::AksimSensor,
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



    //Aksim Ring sensor BUS B
    /////////////
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

	let aksim=AksimSensor::new(aksim_spi);
    #[cfg(feature = "orbita2d")]

	let aksim=SensorKind::Ring(aksim);
    //////////


    //Donut sensor BUS B TODO


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
	let ad5047top=AD5047Sensor::new(ad5047top_spi);
    #[cfg(feature = "orbita3d")]
	let ad5047top=SensorKind::DonutTop(ad5047top);


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
	let ad5047mid=AD5047Sensor::new(ad5047mid_spi);
    #[cfg(feature = "orbita3d")]
	let ad5047mid=SensorKind::DonutMid(ad5047mid);

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
	let ad5047bot=AD5047Sensor::new(ad5047bot_spi);
    #[cfg(feature = "orbita3d")]
	let ad5047bot=SensorKind::DonutBot(ad5047bot);
///////////////






    let foc_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(config.b.foc_cs, Level::High, Speed::Medium),
        foc_spi_config,
    );
    let foc = Foc::new(
        foc_spi,
        config.b.foc_enable,
	#[cfg(feature = "orbita2d")]
        config::BrushlessMotor::ec45(),
	#[cfg(feature = "orbita3d")]
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
	#[cfg(feature = "orbita2d")]
        config::BrushlessMotor::ec45(),
	#[cfg(feature = "orbita3d")]
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



    //ad5047 sensor BUS C
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
    let ad5047=AD5047Sensor::new(ad5047_spi);
    #[cfg(feature = "orbita2d")]
    let ad5047=SensorKind::Center(ad5047);
    /////////



    // Setup the actuator with the configured ventouses
    #[cfg(feature = "orbita2d")]
    let mut actuator = Actuator::new([ventouse_b, ventouse_c], [aksim, ad5047]);
    #[cfg(feature = "orbita3d")]
    let mut actuator = Actuator::new([ventouse_a, ventouse_b, ventouse_c], [ad5047top, ad5047mid, ad5047bot]);
    let mut init_error=false;
    let res_init=actuator.init().await;
    match res_init {
	Ok(_v) => {
	    info!("Registers init ok");
	},
	Err(e) => {
	    init_error=true;
	    error!("init error: {:?}", e);
	}
    }

    let init_sensors=actuator.get_axis_sensors().unwrap();
    let res = actuator.check_motors_1().await;
    match res {
	Ok(_v) => {

	},
	Err(e) => {
	    init_error=true;
	    error!("Motor check error: {:?}", e);
	}
    }



    let moved_sensors = actuator.get_axis_sensors().unwrap();
    let res =actuator.check_motors_2().await;
    match res {
	Ok(_v) => {

	},
	Err(e) => {
	    init_error=true;
	    error!("Motor check error: {:?}", e);
	}
    }

    let mut diff=[0.0;config::N_AXIS];
    for (i, s) in moved_sensors.iter().enumerate() {
        diff[i] = *s - init_sensors[i];
	//Orbita3D
	if (diff[i]<0.0 && diff[i]>-0.1) || (diff[i]>0.0) {
	    error!("Axis sensor {:?} moved too little: {:?} Check sensor connection??", i, diff[i]);
	    init_error=true;

	}
    }
    debug!("init sensors: {:?}", init_sensors);
    debug!("moved sensors: {:?}", moved_sensors);
    debug!("diff sensors: {:?}", diff);



    block_for(Duration::from_secs(1));
    #[cfg(feature = "orbita2d")]
    actuator.set_torque([false,false]).unwrap();
    #[cfg(feature = "orbita3d")]
    actuator.set_torque([false,false,false]).unwrap();
    info!("init done");
    // Init SharedMemory with real values before actually running the control loop
    SHARED_MEMORY.lock().await.init(&mut actuator);
    if init_error {
	SHARED_MEMORY.lock().await.set_error_led(true);
    }

    //"Slow" registers
    let mut init_fluxpid = { SHARED_MEMORY.lock().await.get_flux_pid_gains() };
    let mut init_torquepid = { SHARED_MEMORY.lock().await.get_torque_pid_gains() };
    let mut init_velocitypid = { SHARED_MEMORY.lock().await.get_velocity_pid_gains() };
    let mut init_positionpid = { SHARED_MEMORY.lock().await.get_position_pid_gains() };
    let mut init_uqudlimit = { SHARED_MEMORY.lock().await.get_uq_ud_limit() };
    let mut init_torquefluxlimit = { SHARED_MEMORY.lock().await.get_torque_flux_limit() };
    let mut init_velocitylimit = { SHARED_MEMORY.lock().await.get_velocity_limit() };



    // actuator.set_torque([false,false]).unwrap();
    let mut error_led=false;
    let mut prev_error_led=false;

    use biquad::*;
    let f0 = 10.hz();
    let fs = 1.khz();

    // Create coefficients for the biquads
    let coeffs = Coefficients::<f32>::from_params(Type::LowPass, fs, f0, Q_BUTTERWORTH_F32).unwrap();

    // Create two different biquads
    // let mut biquad = DirectForm1::<f32>::new(coeffs);
    // let mut biquad = DirectForm2Transposed::<f32>::new(coeffs);
    let mut torque_filter=[DirectForm2Transposed::<f32>::new(coeffs); config::N_AXIS];
    let mut vel_filter=[DirectForm2Transposed::<f32>::new(coeffs); config::N_AXIS];
    let mut slow_timer:u32=1000;
    loop {
	let t0=Instant::now();
	// warn!("ELAPSED -1 {:?}",t0.elapsed().as_micros());
	//TODO match and set error led for every call
        let pos = actuator.get_current_position().unwrap_or_else(|e|
	    {
		error!("Error reading position: {:?}", e);
		error_led=true;
		[f32::NAN; config::N_AXIS]
	    });
        {
	    // warn!("ELAPSED 0 {:?}",t0.elapsed().as_micros());
            SHARED_MEMORY.lock().await.set_current_position(pos);
	    // warn!("ELAPSED 1 {:?}",t0.elapsed().as_micros());
        }


        let torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };
	// warn!("ELAPSED 2 {:?}",t0.elapsed().as_micros());
        actuator.set_torque(torque_on).unwrap_or_else(|e|
						      {
							  error!("Error setting torque: {:?}", e);
							  error_led=true;
						      }
	);
	// warn!("ELAPSED 3 {:?}",t0.elapsed().as_micros());
        let target = { SHARED_MEMORY.lock().await.get_target_position() };
        actuator.set_target_position(target).unwrap_or_else(|e|
						      {
							  error!("Error setting target pos: {:?}", e);
							  error_led=true;
						      }
	);





	let sensors=actuator.get_axis_sensors();
	match sensors {
	    Ok(sensors) => {
		SHARED_MEMORY.lock().await.set_axis_sensor(sensors);
		// info!("sensors: {:?}", sensors);
	    },
	    Err(_e) => {
		// SHARED_MEMORY.lock().await.set_axis_sensor([999999.0, 999999.0]);
		error_led=true;
		error!("Axis sensors error");
	    }
	}




	let torque=actuator.get_current_torque();
	match torque {
	    Ok(mut torque) => {
		torque.iter_mut().enumerate().for_each(|(i,t)| {*t=torque_filter[i].run(*t)});
		SHARED_MEMORY.lock().await.set_current_torque(torque);
		// info!("sensors: {:?}", sensors);
	    },
	    Err(_e) => {

		error_led=true;
		error!("Torque error");
	    }
	}

	let vel=actuator.get_current_velocity();
	match vel {
	    Ok(mut vel) => {
		vel.iter_mut().enumerate().for_each(|(i,t)| {*t=vel_filter[i].run(*t)});
		SHARED_MEMORY.lock().await.set_current_velocity(vel);
		// info!("sensors: {:?}", sensors);
	    },
	    Err(_e) => {

		error_led=true;
		error!("Vel error");
	    }
	}



	if error_led!=prev_error_led {
	    SHARED_MEMORY.lock().await.set_error_led(error_led);
	    prev_error_led=error_led;
	}


	if slow_timer == 0
	{
	    //PID and limits: only for debug


            let fluxpid = { SHARED_MEMORY.lock().await.get_flux_pid_gains() };
	    if fluxpid!=init_fluxpid {

		actuator.set_flux_pid_gains(fluxpid).unwrap_or_else(|e|
								    {
									error!("Error setting flux pid: {:?}", e);
									error_led=true;
								    }
		);
		init_fluxpid=fluxpid;

	    }


            let torquepid = { SHARED_MEMORY.lock().await.get_torque_pid_gains() };
	    if torquepid!=init_torquepid {

		actuator.set_torque_pid_gains(torquepid).unwrap_or_else(|e|
									{
									    error!("Error setting torque pid: {:?}", e);
									    error_led=true;
									}
		);
		init_torquepid=torquepid;
	    }

            let velocitypid = { SHARED_MEMORY.lock().await.get_velocity_pid_gains() };
	    if velocitypid!=init_velocitypid {

		actuator.set_velocity_pid_gains(velocitypid).unwrap_or_else(|e|
									    {
										error!("Error setting velocity pid: {:?}", e);
										error_led=true;
									    }
		);
		init_velocitypid=velocitypid;
	    }

            let positionpid = { SHARED_MEMORY.lock().await.get_position_pid_gains() };
	    if positionpid!=init_positionpid {

		actuator.set_position_pid_gains(positionpid).unwrap_or_else(|e|
									    {
										error!("Error setting position pid: {:?}", e);
										error_led=true;
									    }
		);
		init_positionpid=positionpid;
	    }

            let uqudlimit = { SHARED_MEMORY.lock().await.get_uq_ud_limit() };
	    if uqudlimit!=init_uqudlimit {

		actuator.set_uq_ud_limit(uqudlimit).unwrap_or_else(|e|
								   {
								       error!("Error setting uq/ud limit: {:?}", e);
								       error_led=true;
								   }
		);
		init_uqudlimit=uqudlimit;
	    }


            let torquefluxlimit = { SHARED_MEMORY.lock().await.get_torque_flux_limit() };
	    if torquefluxlimit!=init_torquefluxlimit {

		actuator.set_torque_flux_limit(torquefluxlimit).unwrap_or_else(|e|
									       {
										   error!("Error setting torque/flux limit: {:?}", e);
										   error_led=true;
									       }
		);
		init_torquefluxlimit=torquefluxlimit;
	    }

            let velocitylimit = { SHARED_MEMORY.lock().await.get_velocity_limit() };
	    if velocitylimit!=init_velocitylimit {

		actuator.set_velocity_limit(velocitylimit).unwrap_or_else(|e|
									  {
									      error!("Error setting velocity limit: {:?}", e);
									      error_led=true;
									  }
		);
		init_velocitylimit=velocitylimit;
	    }


	    slow_timer=1000;


	}
	else
	{
	    slow_timer-=1;
	}

	let elapsed=t0.elapsed().as_micros();
	// warn!("ELAPSED: {:?}",elapsed);
        Timer::after(Duration::from_micros(1000-elapsed)).await;
        // Timer::after(Duration::from_millis(1)).await;
    }
}
