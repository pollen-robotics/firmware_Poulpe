use core::cell::RefCell;

use crate::motor_control::foc::MotionMode;
use crate::motor_control::{Actuator, Pid, RawMotorsIO, RawSensorsIO};
use crate::state_machine::cia402_state_machine::CiA402StateMachine;
use crate::state_machine::poulpe_state::PoulpeState;
use defmt::error;
use defmt::Format;
use embassy_time::Instant;

#[derive(Clone, Format)]
pub struct Memory<const N: usize> {
    torque_on: [bool; N],
    control_mode: MotionMode,
    control_word: u16,

    current_position: [f32; N],
    current_velocity: [f32; N],
    current_torque: [f32; N],

    target_position: [f32; N],
    target_velocity: [f32; N],
    target_torque: [f32; N],
    velocity_feedforward: [f32; N],

    velocity_feedforward_timestamp: Option<Instant>,
    get_target_set_timestamp: Option<Instant>,

    flux_pid_gains: [Pid; N],
    torque_pid_gains: [Pid; N],
    velocity_pid_gains: [Pid; N],
    position_pid_gains: [Pid; N],

    board_temperatures: [f32; N],
    motor_temperature: [f32; N],
    bus_voltages: [f32; N],

    uq_ud_limit: [i16; N],
    torque_flux_limit: [f32; N],
    velocity_limit: [f32; N],

    // uq_ud_limit_max: [i16; N],
    torque_flux_limit_max: [f32; N],
    velocity_limit_max: [f32; N],

    axis_sensor: [f32; N],
    hardware_zeros: [f32; N],

    #[cfg(feature = "orbita3d")]
    index_sensor: [u8; N],

    // #[cfg(feature = "orbita3d")]
    // hall_states: u16,
    error_led: bool,
    poulpe_state: PoulpeState,
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

    pub fn get_control_mode(&self) -> MotionMode {
        self.inner.borrow().control_mode
    }
    pub fn set_control_mode(&self, mode: MotionMode) {
        self.inner.borrow_mut().control_mode = mode;
    }

    pub fn get_control_word(&self) -> u16 {
        self.inner.borrow().control_word
    }
    pub fn set_control_word(&self, word: u16) {
        self.inner.borrow_mut().control_word = word;
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
        // set timestamp
        self.inner.borrow_mut().get_target_set_timestamp = Some(Instant::now());
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
        // set the timestamp
        self.inner.borrow_mut().velocity_feedforward_timestamp = Some(Instant::now());
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

    pub fn set_axis_sensor(&self, sensor: [f32; N]) {
        self.inner.borrow_mut().axis_sensor = sensor;
    }

    pub fn get_hardware_zeros(&self) -> [f32; N] {
        self.inner.borrow().hardware_zeros
    }

    pub fn set_hardware_zeros(&self, zeros: [f32; N]) {
        self.inner.borrow_mut().hardware_zeros = zeros;
    }

    pub fn set_poulpe_state(&self, state: PoulpeState) {
        self.inner.borrow_mut().poulpe_state = state;
    }
    pub fn get_poulpe_state(&self) -> PoulpeState {
        self.inner.borrow().poulpe_state
    }

    pub fn set_error_led(&self, error: bool) {
        self.inner.borrow_mut().error_led = error;
    }

    pub fn get_error_led(&self) -> bool {
        self.inner.borrow().error_led
    }

    pub fn get_full_state(&self) -> [f32; 3 * N] {
        let mut state = [0.0; 3 * N];
        // state[0..N].copy_from_slice(&self.get_target_position());
        // state[N..2*N].copy_from_slice(&self.get_current_position());
        // state[2*N..3*N].copy_from_slice(&self.get_current_velocity());
        // state[3*N..4*N].copy_from_slice(&self.get_current_torque());
        state[0..N].copy_from_slice(&self.get_current_position());
        state[N..2 * N].copy_from_slice(&self.get_current_velocity());
        state[2 * N..3 * N].copy_from_slice(&self.get_current_torque());
        state
    }

    pub fn get_flux_pid_gains(&self) -> [Pid; N] {
        self.inner.borrow().flux_pid_gains
    }
    pub fn set_flux_pid_gains(&self, gains: [Pid; N]) {
        self.inner.borrow_mut().flux_pid_gains = gains;
    }

    pub fn get_torque_pid_gains(&self) -> [Pid; N] {
        self.inner.borrow().torque_pid_gains
    }
    pub fn set_torque_pid_gains(&self, gains: [Pid; N]) {
        self.inner.borrow_mut().torque_pid_gains = gains;
    }

    pub fn get_velocity_pid_gains(&self) -> [Pid; N] {
        self.inner.borrow().velocity_pid_gains
    }
    pub fn set_velocity_pid_gains(&self, gains: [Pid; N]) {
        self.inner.borrow_mut().velocity_pid_gains = gains;
    }

    pub fn get_position_pid_gains(&self) -> [Pid; N] {
        self.inner.borrow().position_pid_gains
    }
    pub fn set_position_pid_gains(&self, gains: [Pid; N]) {
        self.inner.borrow_mut().position_pid_gains = gains;
    }

    pub fn get_uq_ud_limit(&self) -> [i16; N] {
        self.inner.borrow().uq_ud_limit
    }
    pub fn set_uq_ud_limit(&self, limit: [i16; N]) {
        self.inner.borrow_mut().uq_ud_limit = limit;
    }

    pub fn get_torque_flux_limit(&self) -> [f32; N] {
        self.inner.borrow().torque_flux_limit
    }
    pub fn set_torque_flux_limit(&self, limit: [f32; N]) {
        self.inner.borrow_mut().torque_flux_limit = limit;
    }

    pub fn get_velocity_limit(&self) -> [f32; N] {
        self.inner.borrow().velocity_limit
    }
    pub fn set_velocity_limit(&self, limit: [f32; N]) {
        self.inner.borrow_mut().velocity_limit = limit;
    }

    /*
    pub fn get_uq_ud_limit_max(&self) -> [i16; N] {
        self.inner.borrow().uq_ud_limit_max
    }
    pub fn set_uq_ud_limit_max(&self, limit: [i16; N]) {
        self.inner.borrow_mut().uq_ud_limit_max = limit;
    }
    */
    pub fn get_torque_flux_limit_max(&self) -> [f32; N] {
        self.inner.borrow().torque_flux_limit_max
    }
    pub fn set_torque_flux_limit_max(&self, limit: [f32; N]) {
        self.inner.borrow_mut().torque_flux_limit_max = limit;
    }

    pub fn get_velocity_limit_max(&self) -> [f32; N] {
        self.inner.borrow().velocity_limit_max
    }
    pub fn set_velocity_limit_max(&self, limit: [f32; N]) {
        self.inner.borrow_mut().velocity_limit_max = limit;
    }

    pub fn get_velocity_feedforward_timestamp(&self) -> Option<Instant> {
        self.inner.borrow().velocity_feedforward_timestamp
    }
    pub fn get_target_set_timestamp(&self) -> Option<Instant> {
        self.inner.borrow().get_target_set_timestamp
    }

    pub fn get_board_temperature(&self) -> [f32; N] {
        self.inner.borrow().board_temperatures
    }
    pub fn set_board_temperature(&self, temp: [f32; N]) {
        self.inner.borrow_mut().board_temperatures = temp;
    }
    pub fn get_motor_temperature(&self) -> [f32; N] {
        self.inner.borrow().motor_temperature
    }
    pub fn set_motor_temperature(&self, temp: [f32; N]) {
        self.inner.borrow_mut().motor_temperature = temp;
    }
    pub fn get_bus_voltage(&self) -> [f32; N] {
        self.inner.borrow().bus_voltages
    }
    pub fn set_bus_voltage(&self, volt: [f32; N]) {
        self.inner.borrow_mut().bus_voltages = volt;
    }

    #[cfg(feature = "orbita3d")]
    pub fn get_index_sensor(&self) -> [u8; N] {
        self.inner.borrow_mut().index_sensor
    }

    #[cfg(feature = "orbita3d")]
    pub fn set_index_sensor(&self, index: [u8; N]) {
        self.inner.borrow_mut().index_sensor = index;
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
                control_mode: MotionMode::Torque,
                control_word: 0,

                current_position: [0.0; N],
                current_velocity: [0.0; N],
                current_torque: [0.0; N],
                target_position: [0.0; N],
                target_velocity: [0.0; N],
                target_torque: [0.0; N],
                axis_sensor: [0.0; N],
                hardware_zeros: [0.0; N],

                velocity_feedforward_timestamp: None,
                get_target_set_timestamp: None,

                #[cfg(feature = "orbita3d")]
                index_sensor: [0xff; N],

                // #[cfg(feature = "orbita3d")]
                // hall_states: 0xffff,
                flux_pid_gains: [Pid { p: 0, i: 0 }; N],
                torque_pid_gains: [Pid { p: 0, i: 0 }; N],
                velocity_pid_gains: [Pid { p: 0, i: 0 }; N],
                position_pid_gains: [Pid { p: 0, i: 0 }; N],

                board_temperatures: [0.0; N],
                motor_temperature: [0.0; N],
                bus_voltages: [0.0; N],

                uq_ud_limit: [0; N],
                torque_flux_limit: [0.0; N],
                velocity_limit: [0.0; N],

                // uq_ud_limit_max: [0; N],
                torque_flux_limit_max: [0.0; N],
                velocity_limit_max: [0.0; N],

                velocity_feedforward: [0.0; N],

                error_led: false,
                poulpe_state: PoulpeState::default(),
            }),
        }
    }

    pub fn init(&self, actuator: &mut Actuator<N>) {
        *self.inner.borrow_mut() = Memory {
            torque_on: actuator.is_torque_on().unwrap_or([false; N]),
            control_mode: match actuator.get_control_mode() {
                Ok(mode) => mode[0],
                Err(_) => MotionMode::Stopped,
            },
            control_word: 0,

            current_position: actuator.get_current_position().unwrap_or([f32::NAN; N]),
            current_velocity: actuator.get_current_velocity().unwrap_or([f32::NAN; N]),
            current_torque: actuator.get_current_torque().unwrap_or([f32::NAN; N]),

            target_position: actuator.get_target_position().unwrap_or([f32::NAN; N]),
            target_velocity: actuator.get_target_velocity().unwrap_or([f32::NAN; N]),
            target_torque: actuator.get_target_torque().unwrap_or([f32::NAN; N]),

            velocity_feedforward_timestamp: Some(Instant::now()),
            get_target_set_timestamp: Some(Instant::now()),

            axis_sensor: actuator.get_axis_sensors().unwrap_or([f32::NAN; N]),
            hardware_zeros: actuator.get_hardware_zeros().unwrap_or([f32::NAN; N]),

            flux_pid_gains: actuator
                .get_flux_pid_gains()
                .unwrap_or([Pid { p: 0, i: 0 }; N]),
            torque_pid_gains: actuator
                .get_torque_pid_gains()
                .unwrap_or([Pid { p: 0, i: 0 }; N]),
            velocity_pid_gains: actuator
                .get_velocity_pid_gains()
                .unwrap_or([Pid { p: 0, i: 0 }; N]),
            position_pid_gains: actuator
                .get_position_pid_gains()
                .unwrap_or([Pid { p: 0, i: 0 }; N]),
            // uq_ud_limit: actuator.get_uq_ud_limit().unwrap_or([f32::NAN;N]),
            // torque_flux_limit: actuator.get_torque_flux_limit().unwrap_or([f32::NAN;N]),
            // velocity_limit: actuator.get_velocity_limit().unwrap_or([f32::NAN;N]),
            uq_ud_limit: actuator.get_uq_ud_limit().unwrap_or([0; N]),

            // torque_flux_limit: actuator.get_torque_flux_limit().unwrap_or([0.0; N]),
            // velocity_limit: actuator.get_velocity_limit().unwrap_or([0.0; N]),
            torque_flux_limit: [1.0; N],
            velocity_limit: [1.0; N],

            // uq_ud_limit_max: actuator.get_uq_ud_limit_max().unwrap_or([0; N]),
            torque_flux_limit_max: actuator.get_torque_flux_limit_max().unwrap_or([0.0; N]),
            velocity_limit_max: actuator.get_velocity_limit_max().unwrap_or([0.0; N]),

            velocity_feedforward: actuator.get_velocity_feedforward().unwrap_or([0.0; N]),

            board_temperatures: actuator.get_board_temperature().unwrap_or([0.0; N]),
            motor_temperature: [0.0; N],
            bus_voltages: actuator.get_bus_voltage().unwrap_or([0.0; N]),

            #[cfg(feature = "orbita3d")]
            index_sensor: actuator.get_index_sensor(),

            // #[cfg(feature = "orbita3d")]
            // hall_states: actuator.get_hall_states(),
            error_led: false,
            poulpe_state: PoulpeState::new(),
        };
    }

    #[allow(dead_code)]
    pub fn snapshot(&self) -> Memory<N> {
        self.inner.borrow().clone()
    }
}
