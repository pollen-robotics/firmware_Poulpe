use core::cell::RefCell;

use defmt::Format;

use crate::motor_control::{Actuator, RawMotorsIO, RawSensorsIO};
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

    axis_sensor: [f32; N],

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

	    error_led: false,

        };
    }

    #[allow(dead_code)]
    pub fn snapshot(&self) -> Memory<N> {
        self.inner.borrow().clone()
    }
}
