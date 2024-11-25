// Cia 402 state machine implementaiton for poulpe
//
// Cia 402 state machine status word codes
// Bit      Description
// -----------------------------------------
// 0        Ready to switch on
// 1        Switched on
// 2        Operation enabled
// 3        Fault
// 4        Voltage enabled
// 5        Quick stop
// 6        Switch on disabled
// 7        Warning (Optional)
// ----------------------------- We use it till here
// 8        Manufacturer specific (Optional)
// 9        Remote (Optional)
// 10       Target reached (Optional)
// 11       Internal limit active (Optional)
// 12 - 13  Operation mode specific (Optional)
// 14 - 15  Manufacturer specific (Optional)

use core::ops::BitAnd;

use defmt::{info, warn};

use crate::utils::conversion::bit;  

#[derive(PartialEq, Clone, Copy, defmt::Format)]
#[repr(u16)]
pub enum CiA402State {
    NotReadyToSwitchOn = 0b00000000, // initialisation and test of the drive is not yet completed
    SwitchOnDisabled = 0b01000000,   // init passed successfully
    ReadyToSwitchOn = 0b00100001, // init sucess + switch off received - (more or less saying that the EtherCAT is connected)
    SwitchedOn = 0b00100011,      // init sucess + switch on received
    //  - in our case we send operation enabled and switch on at the same time, so we dont really use this state
    OperationEnabled = 0b00110111, // switched on + enable operation received
    QuickStopActive = 0b00000111, // quick stop procedure going to Switch_on_disabled state ( we don't use quick stop )
    FaultReactionActive = 0b00011111, // fault reaction going to Fault state
    Fault = 0b00001000,           // fault state
}

#[derive(Debug, PartialEq, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum CiA402StatusBit {
    ReadyToSwitchOn = 0,
    SwitchedOn = 1,
    OperationEnabled = 2,
    Fault = 3,
    VoltageEnabled = 4,
    QuickStop = 5,
    SwitchedOnDisabled = 6,
    Warning = 7,
    Reserved8 = 8,
    Remote = 9,
    OperatingModeSpecific10 = 10,
    InternalLimitActive = 11,
    OperatingModeSpecific12 = 12,
    OperatingModeSpecific13 = 13,
    Reserved14 = 14,
    PositionReferencedToHomePosition = 15,
}

impl CiA402StatusBit {
    pub fn from_bit(bit: u8) -> CiA402StatusBit {
        match bit {
            0 => CiA402StatusBit::ReadyToSwitchOn,
            1 => CiA402StatusBit::SwitchedOn,
            2 => CiA402StatusBit::OperationEnabled,
            3 => CiA402StatusBit::Fault,
            4 => CiA402StatusBit::VoltageEnabled,
            5 => CiA402StatusBit::QuickStop,
            6 => CiA402StatusBit::SwitchedOnDisabled,
            7 => CiA402StatusBit::Warning,
            8 => CiA402StatusBit::Reserved8,
            9 => CiA402StatusBit::Remote,
            10 => CiA402StatusBit::OperatingModeSpecific10,
            11 => CiA402StatusBit::InternalLimitActive,
            12 => CiA402StatusBit::OperatingModeSpecific12,
            13 => CiA402StatusBit::OperatingModeSpecific13,
            14 => CiA402StatusBit::Reserved14,
            15 => CiA402StatusBit::PositionReferencedToHomePosition,
            _ => CiA402StatusBit::Reserved14,
        }
    }
}

// Cia 402 state machine implementaiton for poulpe
// Controlword codes
// Bit      Description
// --------------------------
// 0        Switch on
// 1        Enable voltage
// 2        Quick stop
// 3        Enable operation
// 4-6      Not used
// 7        Fault reset
// 8        Halt
// 9-15     Not used
#[derive(PartialEq, Clone, Copy, defmt::Format)]
#[repr(u16)]
pub enum CiA402Command {
    Shutdown,
    SwitchOn,       // the same as DisableOperation so NOT USED
    DisableVoltage, // NOT USED
    EnableOperation,
    DisableOperation,
    QuickStop, // NOT USED
    FaultReset,
    Unknown,
}

impl CiA402Command {
    // Command             Controlword
    // -----------------------------------------
    // Shutdown            0xxxx110
    // Switch on           0xxx0111
    // Disable voltage     0xxxxx0x  (not used)
    // Quick stop          0xxxx01x  
    // Disable operation   0xxx0111
    // Enable operation    0xxx1111
    // Fault reset         1xxxxxxx
    pub fn from_u16(cmd: u16) -> CiA402Command {
        if bit(cmd, 7) { // 1xxxxxxx
            return CiA402Command::FaultReset; 
        } else if !bit(cmd, 0) && bit(cmd, 1) && bit(cmd, 2) {
            return CiA402Command::Shutdown; // 0xxxx110
        } else if bit(cmd, 1) && !bit(cmd, 2){
            return CiA402Command::QuickStop; // 0xxxx01x  
        } else if bit(cmd, 0) && bit(cmd, 1) && bit(cmd, 2) && bit(cmd, 3) {
            return CiA402Command::EnableOperation; // 0xxx1111
        } else if bit(cmd, 0) && bit(cmd, 1) && bit(cmd, 2) && !bit(cmd, 3) {
            return CiA402Command::DisableOperation; //0xxx0111
        } else {
            return CiA402Command::Unknown;
        }
    }
}

#[derive(PartialEq, Clone, Copy)]
pub struct CiA402StateMachine {
    pub state: CiA402State,
    pub warning_active: bool,
}

impl CiA402StateMachine {
    pub fn new() -> Self {
        CiA402StateMachine {
            state: CiA402State::NotReadyToSwitchOn,
            warning_active: false,
        }
    }

    pub const fn default() -> Self {
        CiA402StateMachine {
            state: CiA402State::NotReadyToSwitchOn,
            warning_active: false,
        }
    }

    // transitions between states
    pub fn update_state_with_command(&mut self, command: CiA402Command) {
        match self.state {
            CiA402State::Fault => {
                if command == CiA402Command::FaultReset {
                    self.state = CiA402State::NotReadyToSwitchOn;
                }
            }
            CiA402State::SwitchOnDisabled => {
                if command == CiA402Command::Shutdown {
                    self.state = CiA402State::ReadyToSwitchOn;
                } else if command == CiA402Command::SwitchOn
                    || command == CiA402Command::DisableOperation
                {
                    self.state = CiA402State::SwitchedOn;
                }
            }
            CiA402State::ReadyToSwitchOn => {
                if command == CiA402Command::SwitchOn || command == CiA402Command::DisableOperation
                {
                    self.state = CiA402State::SwitchedOn;
                } else if command == CiA402Command::EnableOperation {
                    self.state = CiA402State::OperationEnabled;
                }
            }
            CiA402State::SwitchedOn => {
                if command == CiA402Command::EnableOperation {
                    self.state = CiA402State::OperationEnabled;
                }else if command == CiA402Command::QuickStop{
                    self.state = CiA402State::SwitchOnDisabled;
                }
            }
            CiA402State::OperationEnabled => {
                if command == CiA402Command::DisableOperation {
                    self.state = CiA402State::SwitchedOn;
                } else if command == CiA402Command::Shutdown {
                    self.state = CiA402State::SwitchOnDisabled;
                } else if command == CiA402Command::QuickStop {
                    warn!("QuickStop");
                    self.state = CiA402State::QuickStopActive;
                }
            }

            _ => {} // QuickStopActive, FaultReactionActive, NotReadyToSwitchOn cannot be changed from here
        }
    }

    pub fn set_warning_state(&mut self) {
        self.warning_active = true;
    }
    pub fn clear_warning_state(&mut self) {
        self.warning_active = false;
    }

    pub fn set_fault_state(&mut self) {
        if self.state == CiA402State::NotReadyToSwitchOn {
            self.state = CiA402State::Fault;
        } else {
            self.state = CiA402State::FaultReactionActive;
        }
    }

    pub fn set_state(&mut self, state: CiA402State) {
        self.state = state;
    }

    pub fn get_status_bits(&self) -> [Option<CiA402StatusBit>; 8] {
        let mut status_bits = [None; 8];
        let status_word = self.state as u16;
        for i in 0..8 {
            if status_word & (1 << i) != 0 {
                status_bits[i] = Some(CiA402StatusBit::from_bit(i as u8));
            }
        }
        status_bits
    }
}
