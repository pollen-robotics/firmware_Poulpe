use defmt::Format;
use embassy_embedded_hal::shared_bus;
use embassy_stm32::{spi, i2c};

use crate::config::DonutHall;

use super::foc::MotionMode;

pub type Result<T> = core::result::Result<T, IOError>;

#[derive(Debug, Format)]
pub enum IOError {
    SpiError(spi::Error),
    I2cError,
    InvalidData,
    Unavailable,
    InitError,
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct Pid {
    pub p: i16,
    pub i: i16,

}

pub trait RawMotorsIO<const N: usize> {
    /// Check if the motors are ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; N]>;
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: [bool; N]) -> Result<()>;

    /// Get the control mode
    fn get_control_mode(&mut self) -> Result<[MotionMode; N]>;
    /// Set the control mode
    fn set_control_mode(&mut self, mode:MotionMode) -> Result<()>;


    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f32; N]>;

    fn set_current_position(&mut self, pos:[f32;N]) -> Result<()>;

    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f32; N]>;
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f32; N]>;

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; N]>;
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f32; N]) -> Result<()>;



    /// Get the current target velocity of the motors (in rpm)
    fn get_target_velocity(&mut self) -> Result<[f32; N]>;
    /// Set the current target velocity of the motors (in rpm)
    fn set_target_velocity(&mut self, velocity: [f32; N]) -> Result<()>;


    /// Get the current target torque of the motors (in ?? mA)
    fn get_target_torque(&mut self) -> Result<[f32; N]>;
    /// Set the current target torque of the motors (in ?? mA)
    fn set_target_torque(&mut self, torque: [f32; N]) -> Result<()>;


    /// Get uq/ud limit
    fn get_uq_ud_limit(&mut self) -> Result<[i16; N]>;
    /// Set uq/ud limit
    fn set_uq_ud_limit(&mut self, limit: [i16; N]) -> Result<()>;

    /// Get torque/flux limit
    fn get_torque_flux_limit(&mut self) -> Result<[u16; N]>;
    /// Set torque/flux limit
    fn set_torque_flux_limit(&mut self, limit: [u16; N]) -> Result<()>;

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[u32; N]>;
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, limit: [u32; N]) -> Result<()>;

    // /// Get the torque limit of the motors (in Nm)
    // fn get_torque_limit(&mut self) -> Result<[f32; N]>;
    // /// Set the torque limit of the motors (in Nm)
    // fn set_torque_limit(&mut self, limit: [f32; N]) -> Result<()>;

    // /// Get the current PID gains of the motors
    // fn get_pid_gains(&mut self) -> Result<[Pid; N]>;
    // /// Set the current PID gains of the motors
    // fn set_pid_gains(&mut self, pid: [Pid; N]) -> Result<()>;


    /// Get the current flux PID gains of the motors
    fn get_flux_pid_gains(&mut self) -> Result<[Pid; N]>;
    /// Set the current flux PID gains of the motors
    fn set_flux_pid_gains(&mut self, pid: [Pid; N]) -> Result<()>;

    /// Get the current torque PID gains of the motors
    fn get_torque_pid_gains(&mut self) -> Result<[Pid; N]>;
    /// Set the current torque PID gains of the motors
    fn set_torque_pid_gains(&mut self, pid: [Pid; N]) -> Result<()>;

    /// Get the current velocity PID gains of the motors
    fn get_velocity_pid_gains(&mut self) -> Result<[Pid; N]>;
    /// Set the current velocity PID gains of the motors
    fn set_velocity_pid_gains(&mut self, pid: [Pid; N]) -> Result<()>;

    /// Get the current position PID gains of the motors
    fn get_position_pid_gains(&mut self) -> Result<[Pid; N]>;
    /// Set the current position PID gains of the motors
    fn set_position_pid_gains(&mut self, pid: [Pid; N]) -> Result<()>;


    fn find_index(&mut self, donut_sensor: &mut DonutHall) -> Result<[u8;N]>;


}
