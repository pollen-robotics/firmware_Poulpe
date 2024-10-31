pub mod cia402_state_machine;
pub mod poulpe_state;
pub use cia402_state_machine::{CiA402StateMachine, CiA402State, CiA402Command};
use poulpe_state::{PoulpeState, HomingErrorFlag, MotorErrorFlag};


