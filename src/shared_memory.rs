use core::cell::RefCell;

use defmt::Format;

use crate::motor_control::{Actuator, RawMotorsIO, RawSensorsIO, Pid};
use crate::{motor_control::foc::MotionMode};

#[derive(Clone, Format)]
pub struct Memory<const N: usize> {
    torque_on: [bool; N],
    control_mode: [MotionMode; N],

    current_position: [f32; N],
    current_velocity: [f32; N],
    current_torque: [f32; N],

    target_position: [f32; N],
    target_velocity: [f32; N],
    target_torque: [f32; N],
    velocity_feedforward: [f32; N],

    flux_pid_gains: [Pid;N],
    torque_pid_gains: [Pid;N],
    velocity_pid_gains: [Pid;N],
    position_pid_gains: [Pid;N],

    uq_ud_limit: [i16;N],
    torque_flux_limit: [u16;N],
    velocity_limit: [f32;N],


    axis_sensor: [f32; N],


    #[cfg(feature = "orbita3d")]
    index_sensor: [u8; N],

    // #[cfg(feature = "orbita3d")]
    // hall_states: u16,

    error_led: bool,

}

#[derive(Format)]
pub struct SharedMemory<const N: usize> {
    inner: RefCell<Memory<N>>,
}

impl<const N: usize> SharedMemory<N> {
    pub fn get_torque_on(&self) -> [bool; N] {
        self.inner.borrow().torque_on
    }
    pub fn set_torque_on(&self, on: [bool; N]) {
        self.inner.borrow_mut().torque_on = on;
    }


    pub fn get_control_mode(&self) -> [MotionMode; N] {
        self.inner.borrow().control_mode

    }
    pub fn set_control_mode(&self, mode: [MotionMode; N]) {
        self.inner.borrow_mut().control_mode = mode;
    }


    pub fn get_current_position(&self) -> [f32; N] {
        self.inner.borrow().current_position
    }
    pub fn set_current_position(&self, pos: [f32; N]) {
        self.inner.borrow_mut().current_position = pos;
    }

    pub fn get_target_position(&self) -> [f32; N] {
        self.inner.borrow().target_position
    }
    pub fn set_target_position(&self, pos: [f32; N]) {
        self.inner.borrow_mut().target_position = pos;
    }


    pub fn get_current_velocity(&self) -> [f32; N] {
        self.inner.borrow().current_velocity
    }
    pub fn set_current_velocity(&self, vel: [f32; N]) {
        self.inner.borrow_mut().current_velocity = vel;
    }

    pub fn get_target_velocity(&self) -> [f32; N] {
        self.inner.borrow().target_velocity
    }
    pub fn set_target_velocity(&self, vel: [f32; N]) {
        self.inner.borrow_mut().target_velocity = vel;
    }

    // set velocity feedforward 
    pub fn set_velocity_feedforward(&self, vel: [f32; N]) {
        self.inner.borrow_mut().velocity_feedforward = vel;
    }
    // get velocity feedforward
    pub fn get_velocity_feedforward(&self) -> [f32; N] {
        self.inner.borrow().velocity_feedforward
    }

    pub fn get_current_torque(&self) -> [f32; N] {
        self.inner.borrow().current_torque
    }
    pub fn set_current_torque(&self, torque: [f32; N]) {
        self.inner.borrow_mut().current_torque = torque;
    }

    pub fn get_target_torque(&self) -> [f32; N] {
        self.inner.borrow().target_torque
    }
    pub fn set_target_torque(&self, torque: [f32; N]) {
        self.inner.borrow_mut().target_torque = torque;
    }

    pub fn get_axis_sensor(&self) -> [f32; N] {
	self.inner.borrow().axis_sensor
    }

    pub fn set_axis_sensor(&self, sensor: [f32;N]) {
	self.inner.borrow_mut().axis_sensor=sensor;
    }

    pub fn set_error_led(&self, error: bool) {
	self.inner.borrow_mut().error_led=error;
    }

    pub fn get_error_led(&self) -> bool {
	self.inner.borrow().error_led
    }

    pub fn get_full_state(&self) -> [f32; 3*N] {
	let mut state = [0.0; 3*N];
	// state[0..N].copy_from_slice(&self.get_target_position());
	// state[N..2*N].copy_from_slice(&self.get_current_position());
	// state[2*N..3*N].copy_from_slice(&self.get_current_velocity());
	// state[3*N..4*N].copy_from_slice(&self.get_current_torque());
	state[0..N].copy_from_slice(&self.get_current_position());
	state[N..2*N].copy_from_slice(&self.get_current_velocity());
	state[2*N..3*N].copy_from_slice(&self.get_current_torque());
	state
    }

    pub fn get_flux_pid_gains(&self) -> [Pid;N] {
	self.inner.borrow().flux_pid_gains
    }
    pub fn set_flux_pid_gains(&self, gains: [Pid;N]) {
	self.inner.borrow_mut().flux_pid_gains=gains;
    }

    pub fn get_torque_pid_gains(&self) -> [Pid;N] {
	self.inner.borrow().torque_pid_gains
    }
    pub fn set_torque_pid_gains(&self, gains: [Pid;N]) {
	self.inner.borrow_mut().torque_pid_gains=gains;
    }

    pub fn get_velocity_pid_gains(&self) -> [Pid;N] {
	self.inner.borrow().velocity_pid_gains
    }
    pub fn set_velocity_pid_gains(&self, gains: [Pid;N]) {
	self.inner.borrow_mut().velocity_pid_gains=gains;
    }

    pub fn get_position_pid_gains(&self) -> [Pid;N] {
	self.inner.borrow().position_pid_gains
    }
    pub fn set_position_pid_gains(&self, gains: [Pid;N]) {
	self.inner.borrow_mut().position_pid_gains=gains;
    }

    pub fn get_uq_ud_limit(&self) -> [i16;N] {
	self.inner.borrow().uq_ud_limit
    }
    pub fn set_uq_ud_limit(&self, limit: [i16;N]) {
	self.inner.borrow_mut().uq_ud_limit=limit;
    }

    pub fn get_torque_flux_limit(&self) -> [u16;N] {
	self.inner.borrow().torque_flux_limit
    }
    pub fn set_torque_flux_limit(&self, limit: [u16;N]) {
	self.inner.borrow_mut().torque_flux_limit=limit;
    }

    pub fn get_velocity_limit(&self) -> [f32;N] {
	self.inner.borrow().velocity_limit
    }
    pub fn set_velocity_limit(&self, limit: [f32;N]) {
	self.inner.borrow_mut().velocity_limit=limit;
    }


    #[cfg(feature = "orbita3d")]
    pub fn get_index_sensor(&self) -> [u8;N] {
	self.inner.borrow_mut().index_sensor
    }


    #[cfg(feature = "orbita3d")]
    pub fn set_index_sensor(&self, index:[u8;N]) {
	self.inner.borrow_mut().index_sensor=index;
    }

    // #[cfg(feature = "orbita3d")]
    // pub fn get_hall_states(&self) -> u16 {
    // 	self.inner.borrow_mut().hall_states
    // }

    // #[cfg(feature = "orbita3d")]
    // pub fn set_hall_states(&self, hall:u16) {
    // 	self.inner.borrow_mut().hall_states=hall;
    // }




}

impl<const N: usize> SharedMemory<N> {
    pub const fn default() -> Self {
        Self {
            inner: RefCell::new(Memory {
                torque_on: [false; N],
		control_mode: [MotionMode::Torque; N],

                current_position: [0.0; N],
                current_velocity: [0.0; N],
                current_torque: [0.0; N],
                target_position: [0.0; N],
                target_velocity: [0.0; N],
                target_torque: [0.0; N],
		axis_sensor: [0.0; N],

		#[cfg(feature = "orbita3d")]
		index_sensor: [0xff; N],

		// #[cfg(feature = "orbita3d")]
		// hall_states: 0xffff,


		flux_pid_gains: [Pid{p:0,i:0};N],
		torque_pid_gains: [Pid{p:0,i:0};N],
		velocity_pid_gains: [Pid{p:0,i:0};N],
		position_pid_gains: [Pid{p:0,i:0};N],

		uq_ud_limit: [0;N],
		torque_flux_limit: [0;N],
		velocity_limit: [0.0;N],
                velocity_feedforward: [0.0; N],


		error_led: false,

            }),
        }
    }

    pub fn init(&self, actuator: &mut Actuator<N>) {
        *self.inner.borrow_mut() = Memory {
            torque_on: actuator.is_torque_on().unwrap_or([false; N]),
	    control_mode: actuator.get_control_mode().unwrap_or([MotionMode::Stopped; N]),

            current_position: actuator.get_current_position().unwrap_or([f32::NAN; N]),
            current_velocity: actuator.get_current_velocity().unwrap_or([f32::NAN; N]),
            current_torque: actuator.get_current_torque().unwrap_or([f32::NAN; N]),

            target_position: actuator.get_target_position().unwrap_or([f32::NAN; N]),
            target_velocity: actuator.get_target_velocity().unwrap_or([f32::NAN; N]),
            target_torque: actuator.get_target_torque().unwrap_or([f32::NAN; N]),

	    axis_sensor: actuator.get_axis_sensors().unwrap_or([f32::NAN; N]),

	    flux_pid_gains: actuator.get_flux_pid_gains().unwrap_or([Pid{p:0,i:0};N]),
	    torque_pid_gains: actuator.get_torque_pid_gains().unwrap_or([Pid{p:0,i:0};N]),
	    velocity_pid_gains: actuator.get_velocity_pid_gains().unwrap_or([Pid{p:0,i:0};N]),
	    position_pid_gains: actuator.get_position_pid_gains().unwrap_or([Pid{p:0,i:0};N]),
	    // uq_ud_limit: actuator.get_uq_ud_limit().unwrap_or([f32::NAN;N]),
	    // torque_flux_limit: actuator.get_torque_flux_limit().unwrap_or([f32::NAN;N]),
	    // velocity_limit: actuator.get_velocity_limit().unwrap_or([f32::NAN;N]),
	    uq_ud_limit: actuator.get_uq_ud_limit().unwrap_or([0;N]),
	    torque_flux_limit: actuator.get_torque_flux_limit().unwrap_or([0;N]),
	    velocity_limit: actuator.get_velocity_limit().unwrap_or([0.0;N]),
            velocity_feedforward: actuator.get_velocity_feedforward().unwrap_or([0.0; N]),

	    #[cfg(feature = "orbita3d")]
	    index_sensor: actuator.get_index_sensor(),

	    // #[cfg(feature = "orbita3d")]
	    // hall_states: actuator.get_hall_states(),

	    error_led: false,

        };
    }

    #[allow(dead_code)]
    pub fn snapshot(&self) -> Memory<N> {
        self.inner.borrow().clone()
    }
}
