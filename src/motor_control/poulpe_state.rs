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
    OverTemperatureMotor = 8,
    OverTemperatureBoard = 16,
    OverCurrent = 32,
    LowBusVoltage = 64,
    CommunicationFail = 128
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
    MotorMovementCheckFail = 2,
    AxisSensorAlignFail = 4,
    ZeroingFail = 8,
    IndexSearchFail = 16,
}


#[derive(PartialEq, Clone, Copy)]
pub struct PoulpeState {
    pub status: u8,
    pub motor_errors: [u8; config::N_AXIS],
    pub homing_error: u8,
}

impl PoulpeState {
    pub fn new() -> Self {
        PoulpeState {
            status: BoardStatus::Unknown as u8,
            motor_errors: [MotorError::None as u8; config::N_AXIS],
            homing_error: HomingError::None as u8,
        }
    }

    pub fn set_status(&mut self, status: BoardStatus) {
        self.status = status as u8;
    }

    pub fn set_status_u8(&mut self, status: u8) {
        self.status = status as u8;
    }

    pub fn set_motor_error(&mut self, axis: usize, error: MotorError) {
        self.motor_errors[axis] |= error as u8;
    }

    pub fn set_homing_error(&mut self, error: HomingError) {
        self.homing_error |= error as u8;
    }

    pub fn clear_motor_errors(&mut self) {
        for i in 0..config::N_AXIS {
            self.motor_errors[i] = MotorError::None as u8;
        }
    }

    pub fn clear_homing_errors(&mut self) {
        self.homing_error = HomingError::None as u8;
    }

    pub fn clear_errors(&mut self) {
        self.clear_motor_errors();
        self.clear_homing_errors();
    }

    pub fn clear_motor_error(&mut self, axis: usize, error: MotorError) {
        self.motor_errors[axis] &= !(error as u8);
    }

    pub fn clear_homing_error(&mut self, error: HomingError) {
        self.homing_error &= !(error as u8);
    }

    pub fn set_init(&mut self) {
        self.status = BoardStatus::Init as u8;
    }
    pub fn set_operational(&mut self) {
        self.status = BoardStatus::Operational as u8;
    }

    pub fn is_operational(&self) -> bool {
        self.status == BoardStatus::Operational as u8 || self.status == BoardStatus::OperationalWithWarning as u8
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

    pub fn is_error(&self) -> bool {
        self.is_init_error() || self.is_runtime_error()
    }

    pub fn is_warning(&self) -> bool {
        self.status == BoardStatus::OperationalWithWarning as u8
    }

    pub fn is_unknown_state(&self) -> bool {
        self.status == BoardStatus::Unknown as u8
    }

    pub fn is_motor_error(&self, axis: usize) -> bool {
        self.motor_errors[axis] != MotorError::None as u8
    }

    pub fn is_homing_error(&self) -> bool {
        self.homing_error != HomingError::None as u8
    }

    pub fn check_motor_error(&self, axis: usize, error: MotorError) -> bool {
        (self.motor_errors[axis] & (error as u8)) == (error as u8)
    }
        
    pub fn check_homing_error(&self, error: HomingError) -> bool {
        (self.homing_error & (error as u8)) == (error as u8)
    }

    // create a u32 with the bitmap of homing and motor errors
    pub fn error_u32(&self) -> u32 {
        let mut error_code = self.homing_error as u32;
        for i in 0..config::N_AXIS {
            error_code |= (self.motor_errors[i] as u32) << i*8 ;
        }
        return error_code;
    }

    // create the u8 state for the status
    pub fn status_u8(&self) -> u8 {
        return self.status;
    }

    pub fn get_status(&self) -> BoardStatus {
        match self.status {
            0 => BoardStatus::Unknown,
            1 => BoardStatus::Init,
            2 => BoardStatus::InitError,
            4 => BoardStatus::Operational,
            8 => BoardStatus::OperationalWithWarning,
            16 => BoardStatus::RuntimeError,
            _ => BoardStatus::Unknown,
        }
    }

    pub fn get_homing_error(&self) -> [Option<HomingError>; 8] {
        let mut errors = [None; 8];
        if self.homing_error == 0 {
            errors[0] = Some(HomingError::None);
            return errors;
        }
        for i in 0..8 {
            errors[i] = match self.homing_error & (1 << i) {
                0 => None,
                1 => Some(HomingError::AxisSensorReadFail),
                2 => Some(HomingError::MotorMovementCheckFail),
                4 => Some(HomingError::AxisSensorAlignFail),
                8 => Some(HomingError::ZeroingFail),
                16 => Some(HomingError::IndexSearchFail),
                _ => None,
            };
        }
        return errors;
    }

    pub fn get_motor_errors(&self, axis: usize) -> [Option<MotorError>; 8] {
        let mut errors = [None; 8];
        if self.motor_errors[axis] == 0{
            errors[0] = Some(MotorError::None);
            return errors;
        }
        for i in 0..8 {
            errors[i] = match self.motor_errors[axis] & (1 << i) {
                0 => None,
                1 => Some(MotorError::ConfigFail),
                2 => Some(MotorError::MotorAlignFail),
                4 => Some(MotorError::HighTemperatureWarning),
                8 => Some(MotorError::OverTemperatureMotor),
                16 => Some(MotorError::OverTemperatureBoard),
                32 => Some(MotorError::OverCurrent),
                64 => Some(MotorError::LowBusVoltage),
                128 => Some(MotorError::CommunicationFail),
                _ => None,
            };
        }
        return errors;
    }

}

// nice formatting for the PoulpeState
impl defmt::Format for PoulpeState{
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "PoulpeState {{\n status: {:?},\n motor_errors: [", self.get_status());
        for i in 0..config::N_AXIS {
            defmt::write!(f, "Motor:{} - [", i);
            let errors = self.get_motor_errors(i);
            for e in errors {
                if let Some(error) = e {
                    defmt::write!(f, "{:?}, ", error);
                }
            }
            defmt::write!(f, "], ");
        }
        defmt::write!(f, "],\n homing_error: [");
        let errors = self.get_homing_error();
        for e in errors {
            if let Some(error) = e {
                defmt::write!(f, "{:?}, ", error);
            }
        }
        defmt::write!(f, "]}}");
    }
}