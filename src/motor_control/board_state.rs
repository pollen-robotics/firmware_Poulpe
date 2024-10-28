use crate::config;


// general board status and error codes
// - the board is iniatlly in the unknown state
// - then it goes to init state
//   - if there is an error during init, it goes to init error state
// - if everything is fine, it goes to operational state
//   - if there is a warning, it goes to operational with warning state
//   - if there is a runtime error, it goes to runtime error state
// > the init error and runtime error states are not recoverable and the board needs to be reset
#[derive(PartialEq, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum BoardStatus {
    Unknown = 0,
    Init = 1,
    InitError = 2,
    Operational = 4,
    OperationalWithWarning = 8,
    RuntimeError = 16,
}

// Error codes for the motors, we will have one error code per motor
// - None - no error
// - ConfigFail - error during the configuration of the motor
// - MotorAlignFail - error during the motor alignment
// - HighTemperatureWarning - warning for high temperature
// - OverTemperature - error due to the temperature being too high
// - OverCurrent - error due to the current being too high
// - LowBusVoltage - error due to the bus voltage being too low
// - CommunicationFail - error due to communication failure with the motor driver
#[derive(PartialEq, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum MotorError {
    None = 0,
    ConfigFail = 1,
    MotorAlignFail = 2,
    HighTemperatureWarning = 4,
    OverTemperature = 8,
    OverCurrent = 16,
    LowBusVoltage = 32,
    CommunicationFail = 64,
}

// Error codes for the homing procedure
// - None - no error
// - AxisSensorReadFail - error during the reading of the axis sensor
// - ZeroingFail - error during the zeroing of the axis positions
// - IndexSearchFail - error during the search of the index (only orbita3d)
#[derive(PartialEq, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum HomingError {
    None = 0,
    AxisSensorReadFail = 1,
    ZeroingFail = 2,
    IndexSearchFail = 4,
}


pub struct OrbitaState {
    pub status: u8,
    pub motor_errors: [u8; config::N_AXIS],
    pub homing_error: u8,
}

impl OrbitaState {
    pub fn new() -> Self {
        OrbitaState {
            status: BoardStatus::Unknown as u8,
            motor_errors: [MotorError::None as u8; config::N_AXIS],
            homing_error: HomingError::None as u8,
        }
    }

    pub fn set_status(&mut self, status: BoardStatus) {
        self.status = status as u8;
    }

    pub fn set_motor_error(&mut self, axis: usize, error: MotorError) {
        self.motor_errors[axis] &= error as u8;
    }

    pub fn set_homing_error(&mut self, error: HomingError) {
        self.homing_error &= error as u8;
    }

    pub fn clear_motor_errors(&mut self) {
        for i in 0..config::N_AXIS {
            self.motor_errors[i] = MotorError::None as u8;
        }
    }

    pub fn clear_homing_error(&mut self) {
        self.homing_error = HomingError::None as u8;
    }

    pub fn clear_errors(&mut self) {
        self.clear_motor_errors();
        self.clear_homing_error();
    }

    pub fn is_operational(&self) -> bool {
        self.status == BoardStatus::Operational as u8
    }

    pub fn is_init(&self) -> bool {
        self.status == BoardStatus::Init as u8
    }

    pub fn is_init_error(&self) -> bool {
        self.status == BoardStatus::InitError as u8
    }

    pub fn is_runtime_error(&self) -> bool {
        self.status == BoardStatus::RuntimeError as u8
    }

    pub fn is_operational_with_warning(&self) -> bool {
        self.status == BoardStatus::OperationalWithWarning as u8
    }

    pub fn is_unknown(&self) -> bool {
        self.status == BoardStatus::Unknown as u8
    }

    pub fn is_motor_error(&self, axis: usize, error: MotorError) -> bool {
        (self.motor_errors[axis] & error as u8 ) != 0
    }

    pub fn is_homing_error(&self, error: HomingError) -> bool {
        (self.homing_error & error as u8) != 0
    }
}