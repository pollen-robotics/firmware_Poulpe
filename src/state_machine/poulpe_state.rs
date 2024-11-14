use crate::config;
use crate::state_machine::cia402_state_machine::{CiA402StateMachine, CiA402State};

use super::CiA402Command;


// Error codes for the motors, we will have one error code per motor
// - None - no error
// - ConfigFail - error during the configuration of the motor
// - MotorAlignFail - error during the motor alignment
// - HighTemperatureWarning - warning for high temperature
// - OverTemperature - error due to the temperature being too high
// - OverCurrent - error due to the current being too high
// - LowBusVoltage - error due to the bus voltage being too low
// - DriverFault - error due to a fault in the driver
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
    DriverFault = 128,
}

// Error codes for the homing procedure
// - None - no error
// - AxisSensorReadFail - error during the reading of the axis sensor
// - MotorMovementCheckFail - error during the check of the motor movement
// - AxisSensorAlignFail - error during the alignment of the axis sensor
// - ZeroingFail - error during the zeroing of the axis positions
// - IndexSearchFail - error during the search of the index (only orbita3d)
// - CommunicationFail - error due to communication failure with the motor driver
#[derive(PartialEq, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum HomingErrorFlag {
    None = 0,
    AxisSensorReadFail = 1,
    MotorMovementCheckFail = 2,
    AxisSensorAlignFail = 4,
    ZeroingFail = 8,
    IndexSearchFail = 16,
    CommunicationFail = 32,
}


#[derive(PartialEq, Clone, Copy)]
pub struct PoulpeState {
    pub state_machine: CiA402StateMachine,
    pub motor_error_flags: [u8; config::N_AXIS],
    pub homing_error_flags: u8,
}

impl PoulpeState {
    pub const fn default() -> Self {
        Self {
            state_machine: CiA402StateMachine::default(),
            motor_error_flags: [MotorErrorFlag::None as u8; config::N_AXIS],
            homing_error_flags: HomingErrorFlag::None as u8,
        }
    }
}

impl PoulpeState {
    pub fn new() -> Self {
        PoulpeState {
            state_machine: CiA402StateMachine::new(),
            motor_error_flags: [MotorErrorFlag::None as u8; config::N_AXIS],
            homing_error_flags: HomingErrorFlag::None as u8,
        }
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
        self.state_machine.set_state(CiA402State::NotReadyToSwitchOn);
    }
    pub fn notify_init_success(&mut self) {
        self.state_machine.set_state(CiA402State::SwitchOnDisabled);
    }


    pub fn set_fault_state(&mut self) {
        #[cfg(feature = "ignore_errors")]
        {
            return;
        }
        if self.state_machine.state == CiA402State::NotReadyToSwitchOn || self.state_machine.state == CiA402State::Fault{
            self.state_machine.set_state(CiA402State::Fault);
        } else {
            self.state_machine.set_state(CiA402State::FaultReactionActive);
        }
    }

    pub fn set_warning_state(&mut self){
        self.state_machine.set_warning_state();
    }

    pub fn clear_warning_state(&mut self){
        self.state_machine.clear_warning_state();
    }


    pub fn is_preoperation_state(&self) -> bool {
        self.state_machine.state == CiA402State::SwitchOnDisabled || 
        self.state_machine.state == CiA402State::ReadyToSwitchOn || 
        self.state_machine.state == CiA402State::SwitchedOn
    }

    pub fn is_operation_enabled(&self) -> bool {
        self.state_machine.state == CiA402State::OperationEnabled
    }

    pub fn is_init(&self) -> bool {
        self.state_machine.state == CiA402State::NotReadyToSwitchOn
    }

    pub fn is_fault(&self) -> bool {
        self.state_machine.state == CiA402State::Fault
    }

    pub fn is_fault_reaction_state(&self) -> bool {
        self.state_machine.state == CiA402State::FaultReactionActive
    }

    pub fn notify_fault_reaction_done(&mut self) {
        self.state_machine.set_state(CiA402State::Fault);
    }

    pub fn is_warning(&self) -> bool {
        self.state_machine.warning_active
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

    pub fn error_flags_to_u8(&self) -> [u8; 4] {
        let mut error_code = [0; 4];
        error_code[0] = self.homing_error_flags;
        for i in 0..config::N_AXIS {
            error_code[i+1] = self.motor_error_flags[i];
        }
        return error_code;
    }

    // create the u8 state for the status
    pub fn status_to_statusword(&self) -> [u8;2] {
        if self.state_machine.warning_active {
            // set the warning bit (bit 7)
            return [self.state_machine.state as u8 | 0x80, 0 ];
        }else{
            return [self.state_machine.state as u8, 0];
        }
        
    }

    // convert the PoulpeState to a byte array
    pub fn to_byte_array(&self) -> [u8; 6] {
        let mut state = [0; 6];
        state[0] = 0;
        state[1] = self.state_machine.state as u8;
        state[2] = self.homing_error_flags;
        for i in 0..config::N_AXIS {
            state[i+3] = self.motor_error_flags[i];
        }
        return state;
    }
    
    pub fn get_state(&self) -> CiA402State {
        self.state_machine.state 
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
                32 => Some(HomingErrorFlag::CommunicationFail),
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
                128 => Some(MotorErrorFlag::DriverFault),
                _ => None,
            };
        }
        return errors;
    }



    pub fn process_command(&mut self, cmd: u16) {
        self.state_machine.update_state_with_command(CiA402Command::from_u16(cmd));
    }

}

// nice formatting for the PoulpeState
impl defmt::Format for PoulpeState{
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "PoulpeState {{\n state: {:?},warning: {:?}\n status_bits:[", self.get_state(), self.is_warning());
        for bit in self.state_machine.get_status_bits().iter() {
            if let Some(status_bit) = bit {
                defmt::write!(f, "{:?}, ", status_bit);
            }
        }
        defmt::write!(f, "]\n motor_error_flags: [");
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