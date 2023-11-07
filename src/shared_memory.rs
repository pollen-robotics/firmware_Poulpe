use core::cell::RefCell;

use crate::motor_control::{Actuator, RawMotorsIO};

pub struct Memory<const N: usize> {
    torque_on: [bool; N],
    current_position: [f32; N],
    target_position: [f32; N],
}

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
}

impl<const N: usize> SharedMemory<N> {
    pub const fn default() -> Self {
        Self {
            inner: RefCell::new(Memory {
                torque_on: [false; N],
                current_position: [0.0; N],
                target_position: [0.0; N],
            }),
        }
    }

    pub fn init(&self, actuator: &mut Actuator<N>) {
        *self.inner.borrow_mut() = Memory {
            torque_on: actuator.is_torque_on().unwrap(),
            current_position: actuator.get_current_position().unwrap(),
            target_position: actuator.get_target_position().unwrap(),
        };
    }
}
