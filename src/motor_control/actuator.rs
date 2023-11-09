use embassy_futures::join;

use super::motors_io::{Pid, RawMotorsIO, Result};
use super::ventouse::VentouseKind;

pub struct Actuator<const N: usize> {
    axes: [VentouseKind; N],
}

impl<const N: usize> Actuator<N> {
    pub fn new(axes: [VentouseKind; N]) -> Self {
        Self { axes }
    }

    pub async fn init(&mut self) {
        join::join_array(self.axes.each_mut().map(|v| v.init())).await;
    }
}

// TODO: make this generic (how?)
impl<const N: usize> RawMotorsIO<N> for Actuator<N> {
    /// Check if the motors are ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; N]> {
        let mut res = [false; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.is_torque_on()?[0];
        }

        Ok(res)
    }
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: [bool; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_torque([on[i]])?;
        }

        Ok(())
    }

    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_current_position()?[0];
        }

        Ok(res)
    }
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_current_velocity()?[0];
        }

        Ok(res)
    }
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_current_torque()?[0];
        }

        Ok(res)
    }

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_target_position()?[0];
        }

        Ok(res)
    }
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_target_position([position[i]])?;
        }

        Ok(())
    }

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_velocity_limit()?[0];
        }

        Ok(res)
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_velocity_limit([velocity[i]])?;
        }

        Ok(())
    }

    /// Get the torque limit of the motors (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_torque_limit()?[0];
        }

        Ok(res)
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_limit(&mut self, torque: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_torque_limit([torque[i]])?;
        }

        Ok(())
    }

    /// Get the current PID gains of the motors
    fn get_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0.0,
            i: 0.0,
            d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current PID gains of the motors
    fn set_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_pid_gains([pid[i]])?;
        }

        Ok(())
    }
}
