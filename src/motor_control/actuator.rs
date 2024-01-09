use embassy_futures::join;

use crate::config::DonutHall;

use super::foc::MotionMode;
use super::motors_io::{Pid, RawMotorsIO, Result, IOError};
use super::sensors_io::{RawSensorsIO};

use super::sensors::{SensorKind};
use super::ventouse::VentouseKind;

pub struct Actuator<'d, const N: usize> {
    axes: [VentouseKind<'d>; N],
    sensors: [SensorKind<'d>; N],
    #[cfg(feature = "orbita3d")]
    index_sensor: [u8; N],
}

impl<'d, const N: usize> Actuator<'d, N> {
    #[cfg(feature = "orbita3d")]
    pub fn new(axes: [VentouseKind<'d>; N], sensors: [SensorKind<'d>;N]) -> Self {
        Self { axes, sensors, index_sensor: [0xff; N] }

    }
    #[cfg(feature = "orbita2d")]
    pub fn new(axes: [VentouseKind<'d>; N], sensors: [SensorKind<'d>;N]) -> Self {
        Self { axes, sensors }

    }

    pub async fn init(&mut self) -> Result<()>{
        let res=join::join_array(self.axes.each_mut().map(|v| v.init())).await;
	// Ok(())
	for r in res{
	    match r{
		Ok(_) => {},
		Err(e) => {return Err(e)},
	    }
	}
	Ok(())
    }

    // check motors
    pub async fn check_motors_1(&mut self) -> Result<()> {
	let res=join::join_array(self.axes.each_mut().map(|v| v.check_motors_1())).await;

	for r in res{
	    match r{
		Ok(_) => {},
		Err(e) => {return Err(e)},
	    }
	}

	Ok(())
    }
    pub async fn check_motors_2(&mut self) -> Result<()> {
	let res=join::join_array(self.axes.each_mut().map(|v| v.check_motors_2())).await;
	for r in res{
	    match r{
		Ok(_) => {},
		Err(e) => {return Err(e)},
	    }
	}
	Ok(())
    }

    // pub fn get_ventouse(&mut self, v:char) ->Option<&mut dyn RawMotorsIO<1>>{
    // 	match v {
    // 	    'A' => self.axes[0].get_ventouse('A'),
    // 	    'B' => self.axes[1].get_ventouse('B'),
    // 	    'C' => self.axes[2].get_ventouse('C'),
    // 	    _ => None,
    // 	}
    // }

    #[cfg(feature = "orbita3d")]
    pub fn get_index_sensor(&mut self) -> [u8;N] {
	self.index_sensor
    }

    #[cfg(feature = "orbita3d")]
    pub fn set_index_sensor(&mut self, index:[u8;N]) {
	self.index_sensor=index;
    }


}

// TODO: make this generic (how?)
impl<'d, const N: usize> RawMotorsIO<N> for Actuator<'d, N> {


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

    /// Get the control mode
    fn get_control_mode(&mut self) -> Result<[MotionMode; N]> {
        let mut res = [MotionMode::Stopped; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_control_mode()?[0];
        }

        Ok(res)
    }

    /// Set the control mode
    fn set_control_mode(&mut self, mode:MotionMode) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
	    axis.set_control_mode(mode)?;
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




    /// Get the current target velocity of the motors (in rpm)
    fn get_target_velocity(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_target_velocity()?[0];
        }

        Ok(res)

    }
    /// Set the current target velocity of the motors (in rpm)
    fn set_target_velocity(&mut self, velocity: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_target_velocity([velocity[i]])?;
        }

        Ok(())
    }


    /// Get the current target torque of the motors (in ?? mA)
    fn get_target_torque(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_target_torque()?[0];
        }

        Ok(res)

    }
    /// Set the current target torque of the motors (in ?? mA)
    fn set_target_torque(&mut self, torque: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_target_torque([torque[i]])?;
        }

        Ok(())
    }





    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[u32; N]> {
        let mut res = [0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_velocity_limit()?[0];
        }

        Ok(res)
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [u32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_velocity_limit([velocity[i]])?;
        }

        Ok(())
    }

    /// Get the torque limit of the motors (in Nm)
    fn get_torque_flux_limit(&mut self) -> Result<[u16; N]> {
        let mut res = [0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_torque_flux_limit()?[0];
        }

        Ok(res)
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_flux_limit(&mut self, torque: [u16; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_torque_flux_limit([torque[i]])?;
        }

        Ok(())
    }



    /// Get the torque limit of the motors (in Nm)
    fn get_uq_ud_limit(&mut self) -> Result<[i16; N]> {
        let mut res = [0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_uq_ud_limit()?[0];
        }

        Ok(res)
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_uq_ud_limit(&mut self, torque: [i16; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_uq_ud_limit([torque[i]])?;
        }

        Ok(())
    }


    // /// Get the current PID gains of the motors
    // fn get_pid_gains(&mut self) -> Result<[Pid; N]> {
    //     let mut res = [Pid {
    //         p: 0,
    //         i: 0,
    //         // d: 0.0,
    //     }; N];
    //     for (i, axis) in self.axes.iter_mut().enumerate() {
    //         res[i] = axis.get_pid_gains()?[0];
    //     }
    //     Ok(res)
    // }
    // /// Set the current PID gains of the motors
    // fn set_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
    //     for (i, axis) in self.axes.iter_mut().enumerate() {
    //         axis.set_pid_gains([pid[i]])?;
    //     }

    //     Ok(())
    // }

/// Get the current flux PID gains of the motors
    fn get_flux_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0,
            i: 0,
            // d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_flux_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current flux PID gains of the motors
    fn set_flux_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_flux_pid_gains([pid[i]])?;
        }

        Ok(())
    }


/// Get the current torque PID gains of the motors
    fn get_torque_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0,
            i: 0,
            // d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_torque_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current torque PID gains of the motors
    fn set_torque_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_torque_pid_gains([pid[i]])?;
        }

        Ok(())
    }


/// Get the current velocity PID gains of the motors
    fn get_velocity_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0,
            i: 0,
            // d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_velocity_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current velocity PID gains of the motors
    fn set_velocity_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_velocity_pid_gains([pid[i]])?;
        }

        Ok(())
    }


/// Get the current position PID gains of the motors
    fn get_position_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0,
            i: 0,
            // d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_position_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current position PID gains of the motors
    fn set_position_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_position_pid_gains([pid[i]])?;
        }

        Ok(())
    }

    fn find_index(&mut self, donut_sensor: &mut DonutHall) -> Result<[u8;N]> {
	let mut indices:[u8;N]=[255;N];
	for (i, axis) in self.axes.iter_mut().enumerate() {

	    let idx=axis.find_index(donut_sensor);
	    match idx{
		Ok(val) => {
		    indices[i]=val[0];
		},
		Err(e) => indices[i]=255,

	    }
	}
	Ok(indices)

    }

}


impl<'d, const N: usize> RawSensorsIO<N> for Actuator<'d, N> {
   /// The axis sensor
    fn get_axis_sensors(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, sensor) in self.sensors.iter_mut().enumerate() {

            // res[i] = sensor.get_axis_sensors()?[0];

	    match sensor.get_axis_sensors() {
		Ok(val) => res[i] = val[0],
		Err(_) => res[i] = f32::NAN,

		}
	    }
        Ok(res)
    }

}
