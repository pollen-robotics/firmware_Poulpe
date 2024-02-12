use defmt::{debug, error, info};
use embassy_stm32::{gpio::Pin, spi};
use embassy_time::{block_for, Duration, Instant, Timer};

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
        Self {
            foc,
            driver,
            kind: '?',
        }
    }

    pub async fn init(&mut self, kind: char) -> Result<(), IOError> {
        self.kind = kind;
        self.foc.tmc4671_disable();
        let mut ret_err = false;
        let mut err = IOError::InitError;
        info!("[Ventouse{:?}] Initializing register...", self.kind);

        match self.driver.tmc6200_checked_write(0x00u8, 0x00000000u32) {
            Ok(_) => info!("[Ventouse{:?}] TMC6200 init done", self.kind),
            Err(e) => {
                ret_err = true;
                error!(
                    "[Ventouse{:?}] TMC6200 init failed: {:?} => check SPI and power connection",
                    self.kind, e
                );
                err = IOError::SpiError(e);
            }
        }
        match self.driver.tmc6200_checked_write(0x0au8, 0x00000000u32) // DRVSRENGTH=0 for BOB
	{
	    Ok(_) => info!("[Ventouse{:?}] TMC6200 init done",self.kind),
	    Err(e) => {
		ret_err=true;
		error!("[Ventouse{:?}] TMC6200 init failed: {:?}  => check SPI and power connection", self.kind,e);
		err=IOError::SpiError(e);

	    }

	}

        match self.foc.tmc4671_init_registers().await {
            Ok(_) => info!("[Ventouse{:?}] TMC4671 init done", self.kind),
            Err(e) => {
                ret_err = true;
                error!(
                    "[Ventouse{:?}] TMC467100 init failed: {:?}  => check SPI and power connection",
                    self.kind, e
                );
                err = IOError::SpiError(e);
            }
        }

        match self.align_motor().await {
            Ok(_) => info!("[Ventouse{:?}] align done", self.kind),
            Err(e) => {
                ret_err = true;
                error!(
                    "[Ventouse{:?}] align failed: {:?}  => check SPI and power connection",
                    self.kind, e
                );
                err = IOError::SpiError(e);
            }
        }

        match self.foc.tmc4671_set_mode(MotionMode::Position) {
            Ok(_) => {}
            Err(e) => {
                ret_err = true;
                err = IOError::SpiError(e);
            }
        }
        info!("[Ventouse{:?}] Motor set to position mode done", self.kind);
        if ret_err {
            return Err(err);
        }
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
    pub async fn check_motors_1(&mut self) -> Result<(), IOError> {
        //Assume that the registers are already initialized and the motors aligned

        // - Read the initial position and axis sensors
        // - Move the motors
        // - Read the final position and axis sensors
        // - check that it has moved "accordingly"

        // Get initial position
        let initial_position = self
            .foc
            .tmc4671_get_actual_position()
            .map_err(IOError::SpiError)?;
        debug!(
            "[Ventouse{:?}] Initial position: {}",
            self.kind, initial_position
        );

        // Move!
        self.foc
            .tmc4671_set_mode(MotionMode::Velocity)
            .map_err(IOError::SpiError)?;

        // Rotate right
        info!("[Ventouse{:?}] Rotate right...", self.kind);
        #[cfg(feature = "orbita2d")]
        if self.kind == 'B' {
            self.foc
                .tmc4671_set_target_velocity(1000)
                .map_err(IOError::SpiError)?;
        } else {
            self.foc
                .tmc4671_set_target_velocity(50)
                .map_err(IOError::SpiError)?;
        }
        #[cfg(feature = "orbita3d")]
        self.foc
            .tmc4671_set_target_velocity(500)
            .map_err(IOError::SpiError)?;
        let _ = Timer::after(Duration::from_millis(1000)).await;
        self.foc
            .tmc4671_set_target_velocity(0)
            .map_err(IOError::SpiError)?;
        let position = self
            .foc
            .tmc4671_get_actual_position()
            .map_err(IOError::SpiError)?;
        debug!("[Ventouse{:?}] position: {}", self.kind, position);
        // check that it has moved

        let diff = position.saturating_sub(initial_position);
        //TODO is it the same for Orbita3D and Orbita2D?
        #[cfg(feature = "orbita2d")]
        const MIN_DIFF: i32 = 10000;
        #[cfg(feature = "orbita3d")]
        const MIN_DIFF: i32 = 100000;

        if diff < MIN_DIFF {
            error!(
                "[Ventouse{:?}] Motor has not moved enough: {} Check motor/encoder connection",
                self.kind, diff
            );
            return Err(IOError::InitError);
        } else {
            info!("[Ventouse{:?}] Motor has moved: {}", self.kind, diff);
        }

        Ok(())
    }

    pub async fn check_motors_2(&mut self) -> Result<(), IOError> {
        //Assume that check_motors_1 has been called

        // Rotate left
        info!("[Ventouse{:?}] Rotate left...", self.kind);
        #[cfg(feature = "orbita2d")]
        if self.kind == 'B' {
            self.foc
                .tmc4671_set_target_velocity(-1000)
                .map_err(IOError::SpiError)?;
        } else {
            self.foc
                .tmc4671_set_target_velocity(-50)
                .map_err(IOError::SpiError)?;
        }
        // self.foc.tmc4671_set_target_velocity(-150).map_err(IOError::SpiError)?;
        #[cfg(feature = "orbita3d")]
        self.foc
            .tmc4671_set_target_velocity(-500)
            .map_err(IOError::SpiError)?;
        let _ = Timer::after(Duration::from_millis(1000)).await;

        // Stop
        info!("[Ventouse{:?}] Stop...", self.kind);
        self.foc
            .tmc4671_set_target_velocity(0)
            .map_err(IOError::SpiError)?;
        self.foc
            .tmc4671_set_mode(MotionMode::Stopped)
            .map_err(IOError::SpiError)?;

        //Start everything at 0
        self.foc
            .tmc4671_set_actual_position(0)
            .map_err(IOError::SpiError)?;
        self.foc
            .tmc4671_set_target_position(0)
            .map_err(IOError::SpiError)?;
        self.foc
            .tmc4671_set_mode(MotionMode::Position)
            .map_err(IOError::SpiError)?;
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
    fn get_control_mode(&mut self) -> Result<[MotionMode; 1], IOError> {
        let mode = self.foc.tmc4671_get_mode().map_err(IOError::SpiError)?;
        Ok([mode])
    }

    // Set control mode
    fn set_control_mode(&mut self, mode: MotionMode) -> Result<(), IOError> {
        self.foc.tmc4671_set_mode(mode).map_err(IOError::SpiError)?;
        Ok(())
    }

    /// Set the current position of the motors (in radians)
    fn set_current_position(&mut self, pos: [f32; 1]) -> Result<(), IOError> {
        self.foc
            .tmc4671_set_actual_position(conversion::rad_to_encoder(
                self.foc.brushless_motor_config.angle_mech_to_elec(pos[0]), // mechanical to electrical angle
                self.foc.ppr,
            ))
            .map_err(IOError::SpiError)?;
        Ok(())
    }

    /// Get the current position of the motors output (in radians)
    fn get_current_position(&mut self) -> Result<[f32; 1], IOError> {
        let encoder = self
            .foc
            .tmc4671_get_actual_position()
            .map_err(IOError::SpiError)?;
        Ok([self.foc.brushless_motor_config.angle_elec_to_mech(
            // electrical to mechanical angle
            conversion::encoder_to_rad(encoder, self.foc.ppr),
        )])
    }
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f32; 1], IOError> {
        let vel = self
            .foc
            .tmc4671_get_actual_velocity()
            .map_err(IOError::SpiError)?;
        Ok([self
            .foc
            .brushless_motor_config
            .angle_elec_to_mech(conversion::rpm_to_rads(vel as f32))])
    }

    /// Get the current torque of the motors (in Nm)
    /// for now its in mAmps
    fn get_current_torque(&mut self) -> Result<[f32; 1], IOError> {
        let torque = self
            .foc
            .tmc4671_get_torque_actual()
            .map_err(IOError::SpiError)?;
        Ok([self
            .foc
            .current_sensing_config
            .adc_to_mAmps(torque as f32, self.foc.adc_resolution)])
    }

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; 1], IOError> {
        let pos = self
            .foc
            .tmc4671_get_target_position()
            .map_err(IOError::SpiError)?;
        Ok([self.foc.brushless_motor_config.angle_elec_to_mech(
            // electrical to mechanical position
            conversion::encoder_to_rad(pos, self.foc.ppr),
        )])
    }
    /// Set the current target position of the motors output (in radians)
    fn set_target_position(&mut self, position: [f32; 1]) -> Result<(), IOError> {
        self.foc
            .tmc4671_set_target_position(conversion::rad_to_encoder(
                self.foc
                    .brushless_motor_config
                    .angle_mech_to_elec(position[0]), // mechanical to electrical angle
                self.foc.ppr,
            ))
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    /// Set the current velocity feedforward (in radians per second)
    fn set_velocity_feedforward(&mut self, velocity: [f32; 1]) -> Result<(), IOError> {
        let vel_rpm = self
            .foc
            .brushless_motor_config
            .angle_mech_to_elec(conversion::rads_to_rpm(velocity[0] as f32));
        self.foc
            .tmc4671_set_velocity_offset(vel_rpm as i32)
            .map(|_| ())
            .map_err(IOError::SpiError)
    }
    // get current velocity feedforward (in radians per second)
    fn get_velocity_feedforward(&mut self) -> Result<[f32; 1], IOError> {
        let vel = self
            .foc
            .tmc4671_get_velocity_offset()
            .map_err(IOError::SpiError)?;
        Ok([self
            .foc
            .brushless_motor_config
            .angle_elec_to_mech(conversion::rpm_to_rads(vel as f32))])
    }

    // get current target velocity (in radians per second)
    fn get_target_velocity(&mut self) -> Result<[f32; 1], IOError> {
        let vel = self
            .foc
            .tmc4671_get_target_velocity()
            .map_err(IOError::SpiError)?;

        Ok([self
            .foc
            .brushless_motor_config
            .angle_elec_to_mech(conversion::rpm_to_rads(vel as f32))])
    }
    // set new target velocity (in radians per second)
    fn set_target_velocity(&mut self, velocity: [f32; 1]) -> Result<(), IOError> {
        let vel_rpm = self
            .foc
            .brushless_motor_config
            .angle_mech_to_elec(conversion::rads_to_rpm(velocity[0] as f32));
        self.foc
            .tmc4671_set_target_velocity(vel_rpm as i32)
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    /// Get the target torque of the motors (in Nm)
    /// for now its in mAmps
    fn get_target_torque(&mut self) -> Result<[f32; 1], IOError> {
        let torque = self
            .foc
            .tmc4671_get_target_torque()
            .map_err(IOError::SpiError)?;
        Ok([self
            .foc
            .current_sensing_config
            .adc_to_mAmps(torque as f32, self.foc.adc_resolution)])
    }

    fn set_target_torque(&mut self, torque: [f32; 1]) -> Result<(), IOError> {
        // torque from mAmps to ADC counts
        let torque_adc = self
            .foc
            .current_sensing_config
            .mAmps_tp_adc(torque[0], self.foc.adc_resolution);
        self.foc
            .tmc4671_set_target_torque(torque_adc as i32) // TODO convert to mA
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    /// Get the uq_ud limit of the motors
    fn get_uq_ud_limit(&mut self) -> Result<[i16; 1], IOError> {
        //TODO Conversion
        let limit = self
            .foc
            .tmc4671_get_uq_ud_limit()
            .map_err(IOError::SpiError)?;
        Ok([limit as i16])
    }
    /// Set the uq_ud limit of the motors
    fn set_uq_ud_limit(&mut self, limit: [i16; 1]) -> Result<(), IOError> {
        self.foc
            .tmc4671_set_uq_ud_limit(limit[0] as i16)
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    /// Get the flux torque limits of the motors mAmps
    fn get_torque_flux_limit(&mut self) -> Result<[f32; 1], IOError> {
        let limit = self
            .foc
            .tmc4671_get_torque_flux_limit()
            .map_err(IOError::SpiError)?;
        Ok([self
            .foc
            .current_sensing_config
            .adc_to_mAmps(limit as f32, self.foc.adc_resolution)])
    }
    /// Set the torque_flux limit of the motors
    fn set_torque_flux_limit(&mut self, limit: [f32; 1]) -> Result<(), IOError> {
        // limit from mAmps to ADC counts
        let limit_adc = self
            .foc
            .current_sensing_config
            .mAmps_tp_adc(limit[0], self.foc.adc_resolution);
        self.foc
            .tmc4671_set_torque_flux_limit(limit_adc as u16)
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f32; 1], IOError> {
        let limit = self
            .foc
            .tmc4671_get_velocity_limit()
            .map_err(IOError::SpiError)?;
        // limit in rad/s
        Ok([self
            .foc
            .brushless_motor_config
            .angle_elec_to_mech(conversion::rpm_to_rads(limit as f32))])
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, limit: [f32; 1]) -> Result<(), IOError> {
        let limit_rpm = self
            .foc
            .brushless_motor_config
            .angle_mech_to_elec(conversion::rads_to_rpm(limit[0] as f32));
        self.foc
            .tmc4671_set_velocity_limit(limit_rpm as u32)
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
        let rawpid = self.foc.tmc4671_get_pid_flux();
        match rawpid {
            Ok(pid) => Ok([Pid {
                p: ((pid >> 16) & 0x7fff) as i16,
                i: (pid & 0x7fff) as i16,
            }]),
            Err(e) => Err(IOError::SpiError(e)),
        }
    }
    /// Set the current flux PID gains of the motors
    fn set_flux_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        let _pid = pid[0].i as u32 | ((pid[0].p as u32) << 16);
        self.foc
            .tmc4671_set_pid_flux(_pid)
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    /// Get the current torque PID gains of the motors
    fn get_torque_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        let rawpid = self.foc.tmc4671_get_pid_torque();
        match rawpid {
            Ok(pid) => Ok([Pid {
                p: ((pid >> 16) & 0x7fff) as i16,
                i: (pid & 0x7fff) as i16,
            }]),
            Err(e) => Err(IOError::SpiError(e)),
        }
    }
    /// Set the current torque PID gains of the motors
    fn set_torque_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        let _pid = pid[0].i as u32 | ((pid[0].p as u32) << 16);
        self.foc
            .tmc4671_set_pid_torque(_pid)
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    /// Get the current velocity PID gains of the motors
    fn get_velocity_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        let rawpid = self.foc.tmc4671_get_pid_velocity();
        match rawpid {
            Ok(pid) => Ok([Pid {
                p: ((pid >> 16) & 0x7fff) as i16,
                i: (pid & 0x7fff) as i16,
            }]),
            Err(e) => Err(IOError::SpiError(e)),
        }
    }
    /// Set the current velocity PID gains of the motors
    fn set_velocity_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        let _pid = pid[0].i as u32 | ((pid[0].p as u32) << 16);
        self.foc
            .tmc4671_set_pid_velocity(_pid)
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    /// Get the current position PID gains of the motors
    fn get_position_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        let rawpid = self.foc.tmc4671_get_pid_position();
        match rawpid {
            Ok(pid) => Ok([Pid {
                p: ((pid >> 16) & 0x7fff) as i16,
                i: (pid & 0x7fff) as i16,
            }]),
            Err(e) => Err(IOError::SpiError(e)),
        }
    }
    /// Set the current position PID gains of the motors
    fn set_position_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        let _pid = pid[0].i as u32 | ((pid[0].p as u32) << 16);
        self.foc
            .tmc4671_set_pid_position(_pid)
            .map(|_| ())
            .map_err(IOError::SpiError)
    }

    fn find_index(&mut self, donut_sensor: &mut DonutHall) -> Result<[u8; 1], IOError> //TODO
    {
        // - Move the motor anticlockwise for about 11.25° (22.5/2) (while the other motors are Off in case they are touching)
        // - Record the Hall state d0
        // - Move the motor clockwise until the Hall state changes d1
        // - If the change is a 255 (dead zone) then the closest (in the CCW direction) is the starting index
        // - If the change is another index (from a dead zone) then the closest (in the CCW direction) is the detected index
        // - setup the motor back
        // - Returns the index of the Hall that changed
        block_for(Duration::from_millis(50));

        self.set_torque([true])?;

        let compute_idx = |d: u16| {
            let mut allindices: [u8; 3] = [0xff; 3]; //Assuming there is only 3 active sensors?
            let mut tmpidx = 0;
            let mut i: usize = 0;
            let mut didx = d.clone();
            while tmpidx < 16 {
                let idx = didx.trailing_ones();
                if idx == 0 && tmpidx > 0 {
                    break;
                }

                if idx < 16 && idx + tmpidx < 16 {
                    allindices[i] = (idx + tmpidx) as u8;
                }
                tmpidx += idx + 1;
                didx >>= (idx + 1);
                i += 1;
                if i == 3 {
                    return allindices; //FIXME: what if there are more? It should not...
                }
            }
            allindices
        };

        let d = donut_sensor.read().unwrap_or_else(|e| {
            error!("{:?} FIND INDEX error: {:?}", self.kind, e);
            0
        });

        let pos = self.foc.tmc4671_get_actual_position().unwrap_or_else(|e| {
            error!("{:?} GET POS error: {:?}", self.kind, e);
            0
        });
        let rads = self
            .foc
            .brushless_motor_config
            .angle_elec_to_mech(conversion::encoder_to_rad(pos, self.foc.ppr));

        let start_indices = compute_idx(d);
        debug!(
            "{:?} Start indices: {:?} ({:#018b}) start pos: {:?}",
            self.kind,
            start_indices,
            d,
            rads.to_degrees()
        );

        self.set_target_velocity([0.0])?;
        self.set_control_mode(MotionMode::Velocity)?;

        self.set_target_velocity([-0.4 / self.foc.brushless_motor_config.axis_ratio()])?;
        // self.set_control_mode(MotionMode::Velocity)?;
        let t0 = Instant::now();
        let mut dd = donut_sensor.read().unwrap_or_else(|e| {
            error!("{:?} FIND INDEX error: {:?}", self.kind, e);
            0
        });

        // let mut detected = false;
        // while !detected && t0.elapsed().as_millis() < 500 {
        //     //We move back until we see a change in the Hall state

        //     if dd != d && dd.count_ones() <= d.count_ones() {
        //         detected = true;
        //     }
        //     // debug!("READ INDEX: {:#018b} {:#018b} {:?}", d, dd, self.kind);
        //     else {
        //         dd = donut_sensor.read().unwrap_or_else(|e| {
        //             error!("FIND INDEX error: {:?}", e);
        //             0
        //         });
        //     }
        // }

        while t0.elapsed().as_millis() < 1000 {
            //We move back until we see a change in the Hall state
            // debug!("READ INDEX: {:#018b} {:#018b} {:?}", d, dd, self.kind);
            if dd != d {
                debug!("{:?} FOUND! {:#018b} -> {:#018b}", self.kind, d, dd);
                self.set_target_velocity([0.0])?;
                break;
            } else {
                dd = donut_sensor.read().unwrap_or_else(|e| {
                    error!("{:?} FIND INDEX error: {:?}", self.kind, e);
                    0
                });
                // block_for(Duration::from_millis(5));
            }
        }

        //First position of the detection zone
        let pos0 = self.foc.tmc4671_get_actual_position().unwrap_or_else(|e| {
            error!("{:?} GET POS error: {:?}", self.kind, e);
            0
        });

        let rads0 = self
            .foc
            .brushless_motor_config
            .angle_elec_to_mech(conversion::encoder_to_rad(pos0, self.foc.ppr));

        let end_indices0 = compute_idx(dd);
        debug!(
            "{:?} end indices: {:?} ({:#018b}) end pos: {:?}",
            self.kind,
            end_indices0,
            dd,
            rads0.to_degrees()
        );
        debug!(
            "{:?} MOVED: {:?}",
            self.kind,
            (rads0 - rads).to_degrees() * self.foc.brushless_motor_config.axis_ratio()
        );

        let mut final_idx = 255;
        let mut detected_from_left = false;
        //If we end up in a detected zone starting from either a detected sensor or a dead zone (255)
        for index in end_indices0.iter() {
            if !start_indices.contains(index) && *index != 255 {
                debug!(
                    "{:?} Moved index: {:?} => found a new idx (idx=new idx)",
                    self.kind, *index
                );
                detected_from_left = true; //we are entering a Hall detection zone from the left (CCW)
                final_idx = *index;
            }
        }
        //If we end up in a dead zone (255), starting from a detected sensor
        for index in start_indices.iter() {
            if !end_indices0.contains(index) && *index != 255 {
                debug!(
                    "{:?} Moved index: {:?} => found a dead zone (idx=starting idx)",
                    self.kind, *index
                );
                detected_from_left = false; //We are exiting a Hall detection zone from the right
                final_idx = *index;
            }
        }

        if detected_from_left {
            //We are searchnig for the end of the detection zone (255) => we move in the CCW direction
            self.set_control_mode(MotionMode::Velocity)?;
            debug!(
                "{:?} Searching for the end of the detection zone (255) => we move in the CCW direction", self.kind
            );
            self.set_target_velocity([-0.4 / self.foc.brushless_motor_config.axis_ratio()])?;
        } else {
            //We are searching for the start of the detection zone (255) => we move ine the CW direction
            self.set_control_mode(MotionMode::Velocity)?;
            debug!("{:?} Searching for the start of the detection zone (255) => we move in the CW direction", self.kind);
            self.set_target_velocity([0.4 / self.foc.brushless_motor_config.axis_ratio()])?;
        }
        let mut dd = donut_sensor.read().unwrap_or_else(|e| {
            error!("{:?} FIND INDEX error: {:?}", self.kind, e);
            0
        });
        let d = dd;
        while t0.elapsed().as_millis() < 1000 {
            //We move back until we see a change in the Hall state
            // debug!("READ INDEX: {:#018b} {:#018b} {:?}", d, dd, self.kind);
            if dd != d && dd.count_zeros() < d.count_zeros() {
                debug!("{:?} FOUND! {:#018b} -> {:#018b}", self.kind, d, dd);
                self.set_target_velocity([0.0])?;
                break;
            } else {
                dd = donut_sensor.read().unwrap_or_else(|e| {
                    error!("{:?} FIND INDEX error: {:?}", self.kind, e);
                    0
                });
                // block_for(Duration::from_millis(5));
            }
        }

        //second position of the detection zone
        let pos1 = self.foc.tmc4671_get_actual_position().unwrap_or_else(|e| {
            error!("{:?} GET POS error: {:?}", self.kind, e);
            0
        });

        let rads1 = self
            .foc
            .brushless_motor_config
            .angle_elec_to_mech(conversion::encoder_to_rad(pos1, self.foc.ppr));

        let end_indices1 = compute_idx(dd);
        debug!(
            "{:?} end indices: {:?} ({:#018b}) end pos: {:?}",
            self.kind,
            end_indices1,
            dd,
            rads1.to_degrees()
        );
        debug!(
            "{:?} MOVED: {:?}",
            self.kind,
            (rads1 - rads0).to_degrees() * self.foc.brushless_motor_config.axis_ratio()
        );

        // The center position should be in the middle of the two positions
        let midpos = (pos0 as f64 + pos1 as f64) / 2.0;
        debug!(
            "{:?} POS0: {:?} POS1: {:?} Mid position: {:?} IDX: {:?}",
            self.kind, pos0, pos1, midpos, final_idx
        );

        self.set_control_mode(MotionMode::Position)?;
        let _ = self.foc.tmc4671_set_target_position(midpos as i32);
        block_for(Duration::from_millis(500));
        let _ = self.foc.tmc4671_set_actual_position(0);
        let _ = self.foc.tmc4671_set_target_position(0);
        Ok([final_idx])
    }
}

pub enum VentouseKind<'d> {
    #[allow(dead_code)]
    A(config::VentouseA<'d>),
    B(config::VentouseB<'d>),
    C(config::VentouseC<'d>),
}

impl<'d> VentouseKind<'d> {
    pub async fn init(&mut self) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.init('A').await,
            VentouseKind::B(vb) => vb.init('B').await,
            VentouseKind::C(vc) => vc.init('C').await,
        }
    }

    pub async fn check_motors_1(&mut self) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.check_motors_1().await,
            VentouseKind::B(vb) => vb.check_motors_1().await,
            VentouseKind::C(vc) => vc.check_motors_1().await,
        }
    }
    pub async fn check_motors_2(&mut self) -> Result<(), IOError> {
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
    fn set_current_position(&mut self, pos: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_current_position(pos),
            VentouseKind::B(vb) => vb.set_current_position(pos),
            VentouseKind::C(vc) => vc.set_current_position(pos),
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

    // Set velocity feedforward
    fn set_velocity_feedforward(&mut self, velocity: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_velocity_feedforward(velocity),
            VentouseKind::B(vb) => vb.set_velocity_feedforward(velocity),
            VentouseKind::C(vc) => vc.set_velocity_feedforward(velocity),
        }
    }
    // get velocity feedforward
    fn get_velocity_feedforward(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_velocity_feedforward(),
            VentouseKind::B(vb) => vb.get_velocity_feedforward(),
            VentouseKind::C(vc) => vc.get_velocity_feedforward(),
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

    /// Get the torque_flux limit of the motors (in mAmps)
    fn get_torque_flux_limit(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_torque_flux_limit(),
            VentouseKind::B(vb) => vb.get_torque_flux_limit(),
            VentouseKind::C(vc) => vc.get_torque_flux_limit(),
        }
    }
    /// Set the torque_flux limit of the motors (in mAmps)
    fn set_torque_flux_limit(&mut self, torque_flux: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_torque_flux_limit(torque_flux),
            VentouseKind::B(vb) => vb.set_torque_flux_limit(torque_flux),
            VentouseKind::C(vc) => vc.set_torque_flux_limit(torque_flux),
        }
    }

    /// Get the uq_ud limit of the motors
    fn get_uq_ud_limit(&mut self) -> Result<[i16; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_uq_ud_limit(),
            VentouseKind::B(vb) => vb.get_uq_ud_limit(),
            VentouseKind::C(vc) => vc.get_uq_ud_limit(),
        }
    }
    /// Set the uq_ud limit of the motors
    fn set_uq_ud_limit(&mut self, uq_ud: [i16; 1]) -> Result<(), IOError> {
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
    fn find_index(&mut self, donut_sensor: &mut DonutHall) -> Result<[u8; 1], IOError> //TODO
    {
        match self {
            VentouseKind::A(va) => va.find_index(donut_sensor),
            VentouseKind::B(vb) => vb.find_index(donut_sensor),
            VentouseKind::C(vc) => vc.find_index(donut_sensor),
        }
    }
}

mod conversion {
    // functions to convert encoder values to radians and vice versa
    pub fn encoder_to_rad(enc: i32, ppr: f32) -> f32 {
        enc as f32 / ppr * 6.28318530718 // 2*pi = 6.28
    }
    pub fn rad_to_encoder(rads: f32, ppr: f32) -> i32 {
        (rads * ppr / 6.28318530718) as i32 // 2*pi = 6.28
    }
    // functions to convert from rpm to radians per second and vice versa
    pub fn rpm_to_rads(rpm: f32) -> f32 {
        rpm * 0.10471975512 // 2*pi/60 = 0.1047
    }
    pub fn rads_to_rpm(rads: f32) -> f32 {
        rads * 9.54929658551 // 60/2*pi = 9.549
    }
}
