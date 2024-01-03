use defmt::{info, error, debug};
use embassy_stm32::{gpio::Pin, spi};
use embassy_time::{Duration, Timer};

use crate::{
    config::{self, DonutHall},
    motor_control::foc::{MotionMode, Tmc4671Registers, OPENLOOP_ACCELERATION, UQ_UD_EXT},
};

use super::{
    // axis_sensor::AxisSensor,
    driver::Driver,
    foc::Foc,
    motors_io::{IOError, Pid, RawMotorsIO},
};

pub struct Ventouse<'d, T, FocP, FocEnb, DrvP>
where
    T: spi::Instance,
    FocP: Pin,
    FocEnb: Pin,
    DrvP: Pin,
{
    foc: Foc<'d, T, FocP, FocEnb>,
    driver: Driver<'d, T, DrvP>,
    pub kind: char,
}

pub struct VentouseConfig<T, SCK, MOSI, MISO, FocCs, FocEnb, DrvCs>
where
    T: spi::Instance,
    SCK: spi::SckPin<T>,
    MOSI: spi::MosiPin<T>,
    MISO: spi::MisoPin<T>,
    FocCs: Pin,
    FocEnb: Pin,
    DrvCs: Pin,
{
    pub peri: T,
    pub sck: SCK,
    pub mosi: MOSI,
    pub miso: MISO,

    pub foc_cs: FocCs,
    pub foc_enable: FocEnb,

    pub driver_cs: DrvCs,

}

impl<'d, T, FocP, FocEnb, DrvP> Ventouse<'d, T, FocP, FocEnb, DrvP>
where
    T: spi::Instance,
    FocP: Pin,
    FocEnb: Pin,
    DrvP: Pin,
{
    pub fn new(foc: Foc<'d, T, FocP, FocEnb>, driver: Driver<'d, T, DrvP>) -> Self {
        Self { foc, driver, kind:'?' }
    }

    pub async fn init(&mut self, kind:char) -> Result<(), embassy_stm32::spi::Error> {
	self.kind=kind;
	self.foc.tmc4671_disable();
	info!("[Ventouse{:?}] Initializing register...", self.kind);

	match self.driver.tmc6200_checked_write(0x00u8, 0x00000000u32)
	{
	    Ok(_) => info!("[Ventouse{:?}] TMC6200 init done",self.kind),
	    Err(e) => error!("[Ventouse{:?}] TMC6200 init failed: {:?} => check SPI and power connection", self.kind,e),

	}
	match self.driver.tmc6200_checked_write(0x0au8, 0x00000000u32) // DRVSRENGTH=0 for BOB
	{
	    Ok(_) => info!("[Ventouse{:?}] TMC6200 init done",self.kind),
	    Err(e) => error!("[Ventouse{:?}] TMC6200 init failed: {:?}  => check SPI and power connection", self.kind,e),

	}

        match self.foc.tmc4671_init_registers().await
	{
	    Ok(_) => info!("[Ventouse{:?}] TMC4671 init done",self.kind),
	    Err(e) => error!("[Ventouse{:?}] TMC467100 init failed: {:?}  => check SPI and power connection", self.kind,e),

	}


        // self.foc.ppr = Some(self.foc.tmc4671_get_encoder_ppr()? as f32);
	self.foc.ppr=Some(524288.0/(2.0*3.141592)); //It seems that 524288=360° motor (0x80000)



        match self.align_motor().await
	{
	    Ok(_) => info!("[Ventouse{:?}] align done",self.kind),
	    Err(e) => error!("[Ventouse{:?}] align failed: {:?}  => check SPI and power connection", self.kind,e),

	}


        self.foc.tmc4671_set_mode(MotionMode::Position)?;
        info!("[Ventouse{:?}] Motor set to position mode done",self.kind);

        Ok(())
    }

    pub async fn align_motor(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        // /!\ Please note that the TMC6200 must be in Single-line mode (aka 6-PMW)
        self.driver.tmc6200_checked_write(0x00u8, 0x00000000u32)?;

        // Set openloop mode
        self.foc
            .tmc4671_checked_write(Tmc4671Registers::OPENLOOP_MODE as u8, 0x00000000)?; // Positive Openloop phi e (OPENLOOP_MODE)
        self.foc.tmc4671_checked_write(
            Tmc4671Registers::OPENLOOP_ACCELERATION as u8,
            OPENLOOP_ACCELERATION,
        )?; // Default acceleration
        self.foc
            .tmc4671_checked_write(Tmc4671Registers::OPENLOOP_VELOCITY_TARGET as u8, 0x00000000)?; // Motor is stopped
        self.foc
            .tmc4671_checked_write(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, 0x00000008)?; // Open loop mode
        self.foc.tmc4671_checked_write(
            Tmc4671Registers::ABN_DECODER_PHI_E_PHI_M_OFFSET as u8,
            0x00000000,
        )?;
        self.foc
            .tmc4671_checked_write(Tmc4671Registers::PHI_E_SELECTION as u8, 0x00000002)?; // Phi_e_openloop
        self.foc
            .tmc4671_checked_write(Tmc4671Registers::UQ_UD_EXT as u8, UQ_UD_EXT)?; // 2000, ud_ext only
        self.foc.tmc4671_enable(); // Start moving
        let _ = Timer::after(Duration::from_millis(1000)).await;
        // Now the motor is aligned with a phase.

        // Clear abn_decoder_count
        self.foc
            .tmc4671_checked_write(Tmc4671Registers::ABN_DECODER_COUNT as u8, 0x00000000)?;

        // Feedback selection
        self.foc
            .tmc4671_checked_write(Tmc4671Registers::PHI_E_SELECTION as u8, 0x00000003)?; // Phi_e_ABN
        self.foc
            .tmc4671_checked_write(Tmc4671Registers::VELOCITY_SELECTION as u8, 0x00000000)?; // PHI_E_SELECTION


	// set everything to 0
        self.foc.tmc4671_set_target_velocity(0)?;
	self.foc.tmc4671_set_actual_position(0)?;
	self.foc.tmc4671_set_target_position(0)?;
        self.foc.tmc4671_set_mode(MotionMode::Stopped)?;
        //put max value
        // self.foc.tmc4671_checked_write(Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8, 0x00007D00)?; // ~4000




        Ok(())
    }
    //check motors
    pub async fn check_motors_1(&mut self) -> Result<(), embassy_stm32::spi::Error> {
	//Assume that the registers are already initialized and the motors aligned

	// - Read the initial position and axis sensors
	// - Move the motors
	// - Read the final position and axis sensors
	// - check that it has moved "accordingly"


	// Get initial position
	let initial_position = self.foc.tmc4671_get_actual_position()?;
	debug!("[Ventouse{:?}] Initial position: {}",self.kind, initial_position);


        // Move!
        self.foc.tmc4671_set_mode(MotionMode::Velocity)?;

        // Rotate right
        info!("[Ventouse{:?}] Rotate right...",self.kind);
	#[cfg(feature = "orbita2d")]
        self.foc.tmc4671_set_target_velocity(50)?;
	#[cfg(feature = "orbita3d")]
        self.foc.tmc4671_set_target_velocity(500)?;
        let _ = Timer::after(Duration::from_millis(1000)).await;
        self.foc.tmc4671_set_target_velocity(0)?;
	let position = self.foc.tmc4671_get_actual_position()?;
	debug!("[Ventouse{:?}] position: {}",self.kind, position);
	// check that it has moved

	let diff=position-initial_position;
	//TODO is it the same for Orbita3D and Orbita2D?
	if diff<100000{
	    error!("[Ventouse{:?}] Motor has not moved enough: {} Check motor/encoder connection",self.kind, diff);
	}
	else{
	    info!("[Ventouse{:?}] Motor has moved: {}",self.kind, diff);

	}

	Ok(())
    }

    pub async fn check_motors_2(&mut self) -> Result<(), embassy_stm32::spi::Error> {
	//Assume that check_motors_1 has been called

        // Rotate left
        info!("[Ventouse{:?}] Rotate left...",self.kind);
	#[cfg(feature = "orbita2d")]
        self.foc.tmc4671_set_target_velocity(-50)?;
	#[cfg(feature = "orbita3d")]
        self.foc.tmc4671_set_target_velocity(-500)?;
        let _ = Timer::after(Duration::from_millis(1000)).await;

        // Stop
        info!("[Ventouse{:?}] Stop...", self.kind);
        self.foc.tmc4671_set_target_velocity(0)?;
        self.foc.tmc4671_set_mode(MotionMode::Stopped)?;

	//Start everything at 0
	self.foc.tmc4671_set_actual_position(0)?;
	self.foc.tmc4671_set_target_position(0)?;
        self.foc.tmc4671_set_mode(MotionMode::Position)?;
	Ok(())
    }

    // pub async fn find_index(&mut self, donut_sensor: &mut DonutHall<'_>) -> Result<(), IOError> //TODO
    // {
    // 	let d=donut_sensor.read();
    // 	Ok(())
    // }

}





impl<'d, T, FocP, FocEnb, DrvP> RawMotorsIO<1> for Ventouse<'d, T, FocP, FocEnb, DrvP>
where
    T: spi::Instance,
    FocP: Pin,
    FocEnb: Pin,
    DrvP: Pin,
{
    /// Check if the motors are ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; 1], IOError> {
        Ok([self.foc.enable.is_set_high()])
    }
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: [bool; 1]) -> Result<(), IOError> {
        if on[0] {
            self.foc.tmc4671_enable();
        } else {
            self.foc.tmc4671_disable();
        }
        Ok(())
    }


    // Get control mode
    fn get_control_mode(&mut self) -> Result<[MotionMode;1], IOError> {
	let mode =self
	    .foc
	    .tmc4671_get_mode()
	    .map_err(IOError::SpiError)?;
	Ok([mode])
    }

    // Set control mode
    fn set_control_mode(&mut self, mode: MotionMode) -> Result<(), IOError> {
	self.foc.tmc4671_set_mode(mode).map_err(IOError::SpiError)?;
	Ok(())
    }


    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f32; 1], IOError> {
        let encoder = self
            .foc
            .tmc4671_get_actual_position()
            .map_err(IOError::SpiError)?;
        let rads = conversion::encoder_to_rads(encoder, self.foc.ppr.unwrap());
        Ok([rads])
    }
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f32; 1], IOError> {
        let vel=self.
	    foc.
	    tmc4671_get_actual_velocity()
	    .map_err(IOError::SpiError)?;
	Ok([vel as f32]) //Should be rpm

    }
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f32; 1], IOError> {
	let torque=self.
	    foc.
	    tmc4671_get_torque_actual()
	    .map_err(IOError::SpiError)?;
	Ok([torque as f32]) //TODO is there a conversion to do?

    }

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; 1], IOError> {
        let pos=self.
	    foc.
	    tmc4671_get_target_position()
	    .map_err(IOError::SpiError)?;
	Ok([pos as f32])
	// Ok([0.0])

    }
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f32; 1]) -> Result<(), IOError> {
        self.foc
            .tmc4671_set_target_position(conversion::rads_to_encoder(
                position[0],
                self.foc.ppr.unwrap(),
            ))
            .map(|_| ())
            .map_err(IOError::SpiError)
    }



    fn get_target_velocity(&mut self) -> Result<[f32; 1], IOError> {
        let vel=self.
	    foc.
	    tmc4671_get_target_velocity()
	    .map_err(IOError::SpiError)?;
	Ok([vel as f32]) //TODO convert to rad/s


    }

    fn set_target_velocity(&mut self, velocity: [f32; 1]) -> Result<(), IOError> {
        self.foc
            .tmc4671_set_target_velocity(
                velocity[0] as i32) // TODO convert to rpm
            .map(|_| ())
            .map_err(IOError::SpiError)
    }


    fn get_target_torque(&mut self) -> Result<[f32; 1], IOError> {
        let pos=self.
	    foc.
	    tmc4671_get_target_torque()
	    .map_err(IOError::SpiError)?;
	Ok([pos as f32])


    }

    fn set_target_torque(&mut self, torque: [f32; 1]) -> Result<(), IOError> {
        self.foc
            .tmc4671_set_target_torque(
                torque[0] as i32)// TODO convert to mA
            .map(|_| ())
            .map_err(IOError::SpiError)
    }



    /// Get the uq_ud limit of the motors
    fn get_uq_ud_limit(&mut self) -> Result<[f32; 1], IOError> {
	//TODO Conversion. Is it in rpm?
        let limit=self.foc.tmc4671_get_uq_ud_limit()
            .map_err(IOError::SpiError)?;
	Ok([limit as f32])

    }
    /// Set the uq_ud limit of the motors
    fn set_uq_ud_limit(&mut self, limit: [f32; 1]) -> Result<(), IOError> {
	self.foc.tmc4671_set_uq_ud_limit(limit[0] as i16)
			.map(|_| ())
			.map_err(IOError::SpiError)

    }


    /// Get the torque_flux limit of the motors
    fn get_torque_flux_limit(&mut self) -> Result<[f32; 1], IOError> {
	//TODO Conversion. Is it in rpm?
        let limit=self.foc.tmc4671_get_torque_flux_limit()
            .map_err(IOError::SpiError)?;
	Ok([limit as f32])

    }
    /// Set the torque_flux limit of the motors
    fn set_torque_flux_limit(&mut self, limit: [f32; 1]) -> Result<(), IOError> {
	self.foc.tmc4671_set_torque_flux_limit(limit[0] as i16)
			.map(|_| ())
			.map_err(IOError::SpiError)

    }


    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f32; 1], IOError> {
	//TODO Conversion. Is it in rpm?
        let limit=self.foc.tmc4671_get_velocity_limit()
            .map_err(IOError::SpiError)?;
	Ok([limit as f32])

    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, limit: [f32; 1]) -> Result<(), IOError> {
	self.foc.tmc4671_set_velocity_limit(limit[0] as u32)
			.map(|_| ())
			.map_err(IOError::SpiError)

    }

    // /// Get the torque limit of the motors (in Nm)
    // fn get_torque_limit(&mut self) -> Result<[f32; 1], IOError> {
    //     Ok([0.0])
    // }
    // /// Set the torque limit of the motors (in Nm)
    // fn set_torque_limit(&mut self, _torque: [f32; 1]) -> Result<(), IOError> {
    //     Ok(())
    // }

    // /// Get the current PID gains of the motors
    // fn get_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
    //     Ok([Pid {
    //         p: 0,
    //         i: 0,
    //         // d: 0.0,
    //     }])
    // }
    // /// Set the current PID gains of the motors
    // fn set_pid_gains(&mut self, _pid: [Pid; 1]) -> Result<(), IOError> {
    //     Ok(())
    // }



    /// Get the current flux PID gains of the motors
    fn get_flux_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
	let rawpid=self.foc.tmc4671_get_pid_flux();
	match rawpid{
	    Ok(pid) => Ok([Pid {
		p: ((pid>>16) & 0x7fff) as i16,
		i: (pid & 0x7fff) as i16,
	    }]),
	    Err(e) => Err(IOError::SpiError(e)),
	}

    }
    /// Set the current flux PID gains of the motors
    fn set_flux_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
	let _pid=pid[0].i as u32 | ((pid[0].p as u32)<<16);
	self.foc.tmc4671_set_pid_flux(_pid)
	    .map(|_| ())
	    .map_err(IOError::SpiError)

    }



    /// Get the current torque PID gains of the motors
    fn get_torque_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
	let rawpid=self.foc.tmc4671_get_pid_torque();
	match rawpid{
	    Ok(pid) => Ok([Pid {
		p: ((pid>>16) & 0x7fff) as i16,
		i: (pid & 0x7fff) as i16,
	    }]),
	    Err(e) => Err(IOError::SpiError(e)),
	}

    }
    /// Set the current torque PID gains of the motors
    fn set_torque_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
	let _pid=pid[0].i as u32 | ((pid[0].p as u32)<<16);
	self.foc.tmc4671_set_pid_torque(_pid)
	    .map(|_| ())
	    .map_err(IOError::SpiError)

    }


    /// Get the current velocity PID gains of the motors
    fn get_velocity_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
	let rawpid=self.foc.tmc4671_get_pid_velocity();
	match rawpid{
	    Ok(pid) => Ok([Pid {
		p: ((pid>>16) & 0x7fff) as i16,
		i: (pid & 0x7fff) as i16,
	    }]),
	    Err(e) => Err(IOError::SpiError(e)),
	}

    }
    /// Set the current velocity PID gains of the motors
    fn set_velocity_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
	let _pid=pid[0].i as u32 | ((pid[0].p as u32)<<16);
	self.foc.tmc4671_set_pid_velocity(_pid)
	    .map(|_| ())
	    .map_err(IOError::SpiError)

    }


    /// Get the current position PID gains of the motors
    fn get_position_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
	let rawpid=self.foc.tmc4671_get_pid_position();
	match rawpid{
	    Ok(pid) => Ok([Pid {
		p: ((pid>>16) & 0x7fff) as i16,
		i: (pid & 0x7fff) as i16,
	    }]),
	    Err(e) => Err(IOError::SpiError(e)),
	}

    }
    /// Set the current position PID gains of the motors
    fn set_position_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
	let _pid=pid[0].i as u32 | ((pid[0].p as u32)<<16);
	self.foc.tmc4671_set_pid_position(_pid)
	    .map(|_| ())
	    .map_err(IOError::SpiError)

    }

    fn find_index(&mut self, donut_sensor: &mut DonutHall) -> Result<(), IOError> //TODO
    {
	// - read initial Hall state
	// - Slowly move the motor (velocity mode?)
	// - Loop while Hall state is the same
	// - Returns the index of the Hall that changed
	// - setup the motor back
	let d=donut_sensor.read();
	match d{
	    Ok(d) => {	debug!("FIND INDEX: {:#x} {:?}",d,self.kind);},
	    Err(e) => error!("DonutHall error: {:?}",e),

	}

	Ok(())
    }






}

pub enum VentouseKind<'d> {
    #[allow(dead_code)]
    A(config::VentouseA<'d>),
    B(config::VentouseB<'d>),
    C(config::VentouseC<'d>),
}

impl<'d> VentouseKind<'d> {
    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        match self {
            VentouseKind::A(va) => va.init('A').await,
            VentouseKind::B(vb) => vb.init('B').await,
            VentouseKind::C(vc) => vc.init('C').await,
        }
    }

    pub async fn check_motors_1(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        match self {
            VentouseKind::A(va) => va.check_motors_1().await,
            VentouseKind::B(vb) => vb.check_motors_1().await,
            VentouseKind::C(vc) => vc.check_motors_1().await,
        }
    }
    pub async fn check_motors_2(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        match self {
            VentouseKind::A(va) => va.check_motors_2().await,
            VentouseKind::B(vb) => vb.check_motors_2().await,
            VentouseKind::C(vc) => vc.check_motors_2().await,
        }
    }

    // pub fn get_ventouse(&mut self, v: char) -> Option<&mut dyn RawMotorsIO<1>> {
    // 		match v {
    // 			'A' => match self {
    // 				VentouseKind::A(va) => Some(va),
    // 				_ => None,
    // 			},
    // 			'B' => match self {
    // 				VentouseKind::B(vb) => Some(vb),
    // 				_ => None,
    // 			},
    // 			'C' => match self {
    // 				VentouseKind::C(vc) => Some(vc),
    // 				_ => None,
    // 			},
    // 			_ => None,
    // 		}
    // 	}



}

impl<'d> RawMotorsIO<1> for VentouseKind<'d> {
    /// Check if the motors are ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.is_torque_on(),
            VentouseKind::B(vb) => vb.is_torque_on(),
            VentouseKind::C(vc) => vc.is_torque_on(),
        }
    }
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: [bool; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_torque(on),
            VentouseKind::B(vb) => vb.set_torque(on),
            VentouseKind::C(vc) => vc.set_torque(on),
        }
    }


    /// Get control mode
    fn get_control_mode(&mut self) -> super::Result<[MotionMode; 1]> {
		match self {
			VentouseKind::A(va) => va.get_control_mode(),
			VentouseKind::B(vb) => vb.get_control_mode(),
			VentouseKind::C(vc) => vc.get_control_mode(),
		}
	}

    ///set control mode

    fn set_control_mode(&mut self, mode: MotionMode) -> super::Result<()> {
		match self {
			VentouseKind::A(va) => va.set_control_mode(mode),
			VentouseKind::B(vb) => vb.set_control_mode(mode),
			VentouseKind::C(vc) => vc.set_control_mode(mode),
		}
	}


    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_current_position(),
            VentouseKind::B(vb) => vb.get_current_position(),
            VentouseKind::C(vc) => vc.get_current_position(),
        }
    }
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_current_velocity(),
            VentouseKind::B(vb) => vb.get_current_velocity(),
            VentouseKind::C(vc) => vc.get_current_velocity(),
        }
    }
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_current_torque(),
            VentouseKind::B(vb) => vb.get_current_torque(),
            VentouseKind::C(vc) => vc.get_current_torque(),
        }
    }

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_target_position(),
            VentouseKind::B(vb) => vb.get_target_position(),
            VentouseKind::C(vc) => vc.get_target_position(),
        }
    }
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_target_position(position),
            VentouseKind::B(vb) => vb.set_target_position(position),
            VentouseKind::C(vc) => vc.set_target_position(position),
        }
    }





    fn get_target_velocity(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_target_velocity(),
            VentouseKind::B(vb) => vb.get_target_velocity(),
            VentouseKind::C(vc) => vc.get_target_velocity(),
        }
    }

    fn set_target_velocity(&mut self, velocity: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_target_velocity(velocity),
            VentouseKind::B(vb) => vb.set_target_velocity(velocity),
            VentouseKind::C(vc) => vc.set_target_velocity(velocity),
        }
    }




    fn get_target_torque(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_target_torque(),
            VentouseKind::B(vb) => vb.get_target_torque(),
            VentouseKind::C(vc) => vc.get_target_torque(),
        }
    }

    fn set_target_torque(&mut self, torque: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_target_torque(torque),
            VentouseKind::B(vb) => vb.set_target_torque(torque),
            VentouseKind::C(vc) => vc.set_target_torque(torque),
        }
    }



    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_velocity_limit(),
            VentouseKind::B(vb) => vb.get_velocity_limit(),
            VentouseKind::C(vc) => vc.get_velocity_limit(),
        }
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_velocity_limit(velocity),
            VentouseKind::B(vb) => vb.set_velocity_limit(velocity),
            VentouseKind::C(vc) => vc.set_velocity_limit(velocity),
        }
    }

    /// Get the torque_flux limit of the motors (in Nm)
    fn get_torque_flux_limit(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_torque_flux_limit(),
            VentouseKind::B(vb) => vb.get_torque_flux_limit(),
            VentouseKind::C(vc) => vc.get_torque_flux_limit(),
        }
    }
    /// Set the torque_flux limit of the motors (in Nm)
    fn set_torque_flux_limit(&mut self, torque_flux: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_torque_flux_limit(torque_flux),
            VentouseKind::B(vb) => vb.set_torque_flux_limit(torque_flux),
            VentouseKind::C(vc) => vc.set_torque_flux_limit(torque_flux),
        }
    }

    /// Get the uq_ud limit of the motors (in Nm)
    fn get_uq_ud_limit(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_uq_ud_limit(),
            VentouseKind::B(vb) => vb.get_uq_ud_limit(),
            VentouseKind::C(vc) => vc.get_uq_ud_limit(),
        }
    }
    /// Set the uq_ud limit of the motors (in Nm)
    fn set_uq_ud_limit(&mut self, uq_ud: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_uq_ud_limit(uq_ud),
            VentouseKind::B(vb) => vb.set_uq_ud_limit(uq_ud),
            VentouseKind::C(vc) => vc.set_uq_ud_limit(uq_ud),
        }
    }


    // /// Get the current PID gains of the motors
    // fn get_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
    //     match self {
    //         VentouseKind::A(va) => va.get_pid_gains(),
    //         VentouseKind::B(vb) => vb.get_pid_gains(),
    //         VentouseKind::C(vc) => vc.get_pid_gains(),
    //     }
    // }
    // /// Set the current PID gains of the motors
    // fn set_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
    //     match self {
    //         VentouseKind::A(va) => va.set_pid_gains(pid),
    //         VentouseKind::B(vb) => vb.set_pid_gains(pid),
    //         VentouseKind::C(vc) => vc.set_pid_gains(pid),
    //     }
    // }



    /// Get the current flux PID gains of the motors
    fn get_flux_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_flux_pid_gains(),
            VentouseKind::B(vb) => vb.get_flux_pid_gains(),
            VentouseKind::C(vc) => vc.get_flux_pid_gains(),
        }
    }
    /// Set the current flux PID gains of the motors
    fn set_flux_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_flux_pid_gains(pid),
            VentouseKind::B(vb) => vb.set_flux_pid_gains(pid),
            VentouseKind::C(vc) => vc.set_flux_pid_gains(pid),
        }
    }


    /// Get the current torque PID gains of the motors
    fn get_torque_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_torque_pid_gains(),
            VentouseKind::B(vb) => vb.get_torque_pid_gains(),
            VentouseKind::C(vc) => vc.get_torque_pid_gains(),
        }
    }
    /// Set the current torque PID gains of the motors
    fn set_torque_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_torque_pid_gains(pid),
            VentouseKind::B(vb) => vb.set_torque_pid_gains(pid),
            VentouseKind::C(vc) => vc.set_torque_pid_gains(pid),
        }
    }


    /// Get the current velocity PID gains of the motors
    fn get_velocity_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_velocity_pid_gains(),
            VentouseKind::B(vb) => vb.get_velocity_pid_gains(),
            VentouseKind::C(vc) => vc.get_velocity_pid_gains(),
        }
    }
    /// Set the current velocity PID gains of the motors
    fn set_velocity_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_velocity_pid_gains(pid),
            VentouseKind::B(vb) => vb.set_velocity_pid_gains(pid),
            VentouseKind::C(vc) => vc.set_velocity_pid_gains(pid),
        }
    }


    /// Get the current position PID gains of the motors
    fn get_position_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_position_pid_gains(),
            VentouseKind::B(vb) => vb.get_position_pid_gains(),
            VentouseKind::C(vc) => vc.get_position_pid_gains(),
        }
    }
    /// Set the current position PID gains of the motors
    fn set_position_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_position_pid_gains(pid),
            VentouseKind::B(vb) => vb.set_position_pid_gains(pid),
            VentouseKind::C(vc) => vc.set_position_pid_gains(pid),
        }
    }

    // fn get_ventouse(&mut self, v: char) -> Option<&mut dyn RawMotorsIO<1>> {
    // 		match v {
    // 		    'A' => match self {
    // 				VentouseKind::A(va) => Some(va),
    // 				_ => None,
    // 			},
    // 			'B' => match self {
    // 				VentouseKind::B(vb) => Some(vb),
    // 				_ => None,
    // 			},
    // 			'C' => match self {
    // 				VentouseKind::C(vc) => Some(vc),
    // 				_ => None,
    // 			},
    // 			_ => None,
    // 		}
    // }
    fn find_index(&mut self, donut_sensor: &mut DonutHall) -> Result<(), IOError> //TODO
	{
	    match self {
		VentouseKind::A(va) => va.find_index(donut_sensor),
		VentouseKind::B(vb) => vb.find_index(donut_sensor),
		VentouseKind::C(vc) => vc.find_index(donut_sensor),
	    }
	}





}

mod conversion {
    pub fn encoder_to_rads(enc: i32, ppr: f32) -> f32 {
        enc as f32 / ppr
    }
    pub fn rads_to_encoder(rads: f32, ppr: f32) -> i32 {
        (rads * ppr) as i32
    }
}
