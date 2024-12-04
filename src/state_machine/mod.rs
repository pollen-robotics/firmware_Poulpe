pub mod cia402_state_machine;
pub mod poulpe_state;
pub use cia402_state_machine::{CiA402Command, CiA402State, CiA402StateMachine, CiA402StatusBit};
use poulpe_state::{HomingErrorFlag, MotorErrorFlag, PoulpeState};

pub mod cia402_registers;
pub use cia402_registers::CiA402ModeOfOperation;
