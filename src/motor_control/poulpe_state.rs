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
pub enum MotorErrorFlag {
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
pub enum HomingErrorFlag {
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
    pub motor_error_flags: [u8; config::N_AXIS],
    pub homing_error_flags: u8,
}

impl PoulpeState {
    pub fn new() -> Self {
        PoulpeState {
            status: BoardStatus::Unknown as u8,
            motor_error_flags: [MotorErrorFlag::None as u8; config::N_AXIS],
            homing_error_flags: HomingErrorFlag::None as u8,
        }
    }

    pub fn set_status(&mut self, status: BoardStatus) {
        self.status = status as u8;
    }

    pub fn set_status_u8(&mut self, status: u8) {
        self.status = status as u8;
    }

    pub fn set_motor_error_flag(&mut self, axis: usize, error: MotorErrorFlag) {
        self.motor_error_flags[axis] |= error as u8;
    }

    pub fn set_homing_error_flag(&mut self, error: HomingErrorFlag) {
        self.homing_error_flags |= error as u8;
    }

    pub fn clear_motor_error_flags(&mut self) {
        for i in 0..config::N_AXIS {
            self.motor_error_flags[i] = MotorErrorFlag::None as u8;
        }
    }

    pub fn clear_homing_error_flags(&mut self) {
        self.homing_error_flags = HomingErrorFlag::None as u8;
    }

    pub fn clear_errors(&mut self) {
        self.clear_motor_error_flags();
        self.clear_homing_error_flags();
    }

    pub fn clear_motor_error_flag(&mut self, axis: usize, error: MotorErrorFlag) {
        self.motor_error_flags[axis] &= !(error as u8);
    }

    pub fn clear_homing_error_flag(&mut self, error: HomingErrorFlag) {
        self.homing_error_flags &= !(error as u8);
    }

    pub fn set_init_state(&mut self) {
        self.status = BoardStatus::Init as u8;
    }
    pub fn set_operational_state(&mut self) {
        self.status = BoardStatus::Operational as u8;
    }

    pub fn set_error_state(&mut self) {
        if self.status == BoardStatus::Init as u8 {
            self.status = BoardStatus::InitError as u8;
        } else {
            self.status = BoardStatus::RuntimeError as u8;
        }
    }

    pub fn set_warning_state(&mut self){
        if self.status == BoardStatus::Operational as u8{
            self.status = BoardStatus::OperationalWithWarning as u8;
        }
    }

    pub fn clear_warning_state(&mut self){
        if self.status == BoardStatus::OperationalWithWarning as u8{
            self.status = BoardStatus::Operational as u8;
        }
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
        self.motor_error_flags[axis] != MotorErrorFlag::None as u8
    }

    pub fn is_homing_error(&self) -> bool {
        self.homing_error_flags != HomingErrorFlag::None as u8
    }

    pub fn check_motor_error_flag(&self, axis: usize, error: MotorErrorFlag) -> bool {
        (self.motor_error_flags[axis] & (error as u8)) == (error as u8)
    }
        
    pub fn check_homing_error_flag(&self, error: HomingErrorFlag) -> bool {
        (self.homing_error_flags & (error as u8)) == (error as u8)
    }

    // create a u32 with the bitmap of homing and motor errors
    pub fn error_flags_to_u32(&self) -> u32 {
        let mut error_code = self.homing_error_flags as u32;
        for i in 0..config::N_AXIS {
            error_code |= (self.motor_error_flags[i] as u32) << i*8 ;
        }
        return error_code;
    }

    // create the u8 state for the status
    pub fn status_to_u8(&self) -> u8 {
        return self.status;
    }

    // convert the PoulpeState to a byte array
    pub fn to_byte_array(&self) -> [u8; 5] {
        let mut state = [0; 5];
        state[0] = self.status;
        state[1] = self.homing_error_flags;
        for i in 0..config::N_AXIS {
            state[i+2] = self.motor_error_flags[i];
        }
        return state;
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

    pub fn get_homing_error_flags(&self) -> [Option<HomingErrorFlag>; 8] {
        let mut errors = [None; 8];
        if self.homing_error_flags == 0 {
            errors[0] = Some(HomingErrorFlag::None);
            return errors;
        }
        for i in 0..8 {
            errors[i] = match self.homing_error_flags & (1 << i) {
                0 => None,
                1 => Some(HomingErrorFlag::AxisSensorReadFail),
                2 => Some(HomingErrorFlag::MotorMovementCheckFail),
                4 => Some(HomingErrorFlag::AxisSensorAlignFail),
                8 => Some(HomingErrorFlag::ZeroingFail),
                16 => Some(HomingErrorFlag::IndexSearchFail),
                _ => None,
            };
        }
        return errors;
    }

    pub fn get_motor_errors_flags(&self, axis: usize) -> [Option<MotorErrorFlag>; 8] {
        let mut errors = [None; 8];
        if self.motor_error_flags[axis] == 0{
            errors[0] = Some(MotorErrorFlag::None);
            return errors;
        }
        for i in 0..8 {
            errors[i] = match self.motor_error_flags[axis] & (1 << i) {
                0 => None,
                1 => Some(MotorErrorFlag::ConfigFail),
                2 => Some(MotorErrorFlag::MotorAlignFail),
                4 => Some(MotorErrorFlag::HighTemperatureWarning),
                8 => Some(MotorErrorFlag::OverTemperatureMotor),
                16 => Some(MotorErrorFlag::OverTemperatureBoard),
                32 => Some(MotorErrorFlag::OverCurrent),
                64 => Some(MotorErrorFlag::LowBusVoltage),
                128 => Some(MotorErrorFlag::CommunicationFail),
                _ => None,
            };
        }
        return errors;
    }


}

// nice formatting for the PoulpeState
impl defmt::Format for PoulpeState{
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "PoulpeState {{\n status: {:?},\n motor_error_flags: [", self.get_status());
        for i in 0..config::N_AXIS {
            defmt::write!(f, "Motor:{} - [", i);
            let errors = self.get_motor_errors_flags(i);
            for e in errors {
                if let Some(error) = e {
                    defmt::write!(f, "{:?}, ", error);
                }
            }
            defmt::write!(f, "], ");
        }
        defmt::write!(f, "],\n homing_error_flags: [");
        let errors = self.get_homing_error_flags();
        for e in errors {
            if let Some(error) = e {
                defmt::write!(f, "{:?}, ", error);
            }
        }
        defmt::write!(f, "]}}");
    }
}