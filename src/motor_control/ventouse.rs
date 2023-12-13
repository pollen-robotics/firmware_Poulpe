use defmt::info;
use embassy_stm32::{gpio::Pin, spi};
use embassy_time::{Duration, Timer};

use crate::{
    config,
    motor_control::foc::{MotionMode, Tmc4671Registers, OPENLOOP_ACCELERATION, UQ_UD_EXT},
};

use super::{
    axis_sensor::AxisSensor,
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
        Self { foc, driver }
    }

    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
	self.foc.tmc4671_disable();
	info!("Initializing register...");
	self.driver.tmc6200_checked_write(0x00u8, 0x00000000u32)?;
	self.driver.tmc6200_checked_write(0x0au8, 0x00000000u32)?; // DRVSRENGTH=0 for BOB

        self.foc.tmc4671_init_registers().await?;
        info!("TMC4671 init done");

        self.foc.ppr = Some(self.foc.tmc4671_get_encoder_ppr()? as f32);

        self.align_motor().await?;
        info!("Motor align done");
        self.foc.tmc4671_set_mode(MotionMode::Position)?;
        info!("Motor set to position mode done");

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

        //put max value
        // self.foc.tmc4671_checked_write(Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8, 0x00007D00)?; // ~4000

        // Move!
        self.foc.tmc4671_set_mode(MotionMode::Velocity)?;

        // Rotate right
        info!("Rotate right...");
        self.foc.tmc4671_set_target_velocity(50)?;
        let _ = Timer::after(Duration::from_millis(1000)).await;

        // Rotate left
        info!("Rotate left...");
        self.foc.tmc4671_set_target_velocity(-50)?;
        let _ = Timer::after(Duration::from_millis(1000)).await;

        // Stop
        info!("Stop...");
        self.foc.tmc4671_set_target_velocity(0)?;
        self.foc.tmc4671_set_mode(MotionMode::Stopped)?;
	// self.foc.tmc4671_disable();
	let pos=self.foc.tmc4671_get_actual_position()?;
	self.foc.tmc4671_set_target_position(pos)?;


        Ok(())
    }
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
        Ok([0.0])
    }
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f32; 1], IOError> {
        Ok([0.0])
    }

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; 1], IOError> {
        Ok([0.0])
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

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f32; 1], IOError> {
        Ok([0.0])
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, _velocity: [f32; 1]) -> Result<(), IOError> {
        Ok(())
    }

    /// Get the torque limit of the motors (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f32; 1], IOError> {
        Ok([0.0])
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_limit(&mut self, _torque: [f32; 1]) -> Result<(), IOError> {
        Ok(())
    }

    /// Get the current PID gains of the motors
    fn get_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        Ok([Pid {
            p: 0.0,
            i: 0.0,
            d: 0.0,
        }])
    }
    /// Set the current PID gains of the motors
    fn set_pid_gains(&mut self, _pid: [Pid; 1]) -> Result<(), IOError> {
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
            VentouseKind::A(va) => va.init().await,
            VentouseKind::B(vb) => vb.init().await,
            VentouseKind::C(vc) => vc.init().await,
        }
    }
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

    /// Get the torque limit of the motors (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_torque_limit(),
            VentouseKind::B(vb) => vb.get_torque_limit(),
            VentouseKind::C(vc) => vc.get_torque_limit(),
        }
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_limit(&mut self, torque: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_torque_limit(torque),
            VentouseKind::B(vb) => vb.set_torque_limit(torque),
            VentouseKind::C(vc) => vc.set_torque_limit(torque),
        }
    }

    /// Get the current PID gains of the motors
    fn get_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        match self {
            VentouseKind::A(va) => va.get_pid_gains(),
            VentouseKind::B(vb) => vb.get_pid_gains(),
            VentouseKind::C(vc) => vc.get_pid_gains(),
        }
    }
    /// Set the current PID gains of the motors
    fn set_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(va) => va.set_pid_gains(pid),
            VentouseKind::B(vb) => vb.set_pid_gains(pid),
            VentouseKind::C(vc) => vc.set_pid_gains(pid),
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
