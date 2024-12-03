use crate::motor_control::foc::MotionMode;
use defmt::*;

// Mode of operation
#[derive(PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum CiA402ModeOfOperation {
    NoMode = 0,
    ProfilePositionMode = 1,
    // VelocityMode = 2, // not used
    ProfileVelocityMode = 3,
    ProfileTorqueMode = 4,
    // HomingMode = 6,
    // InterpolatedPositionMode = 7,
    // CyclicSynchronousPositionMode = 8,
    // CyclicSynchronousVelocityMode = 9,
    // CyclicSynchronousTorqueMode = 10,
}

impl CiA402ModeOfOperation {
    pub fn from_u8(mode: u8) -> CiA402ModeOfOperation {
        match mode {
            0 => CiA402ModeOfOperation::NoMode,
            1 => CiA402ModeOfOperation::ProfilePositionMode,
            3 => CiA402ModeOfOperation::ProfileVelocityMode,
            4 => CiA402ModeOfOperation::ProfileTorqueMode,
            _ => CiA402ModeOfOperation::NoMode,
        }
    }

    pub fn to_u8(&self) -> u8 {
        match self {
            CiA402ModeOfOperation::NoMode => 0,
            CiA402ModeOfOperation::ProfilePositionMode => 1,
            CiA402ModeOfOperation::ProfileVelocityMode => 3,
            CiA402ModeOfOperation::ProfileTorqueMode => 4,
        }
    }

    pub fn to_tmc4671_mode(&self) -> MotionMode {
        match self {
            CiA402ModeOfOperation::NoMode => MotionMode::Stopped,
            CiA402ModeOfOperation::ProfilePositionMode => MotionMode::Position,
            CiA402ModeOfOperation::ProfileVelocityMode => MotionMode::Velocity,
            CiA402ModeOfOperation::ProfileTorqueMode => MotionMode::Torque,
        }
    }

    pub fn from_tmc4671_mode(mode: MotionMode) -> CiA402ModeOfOperation {
        match mode {
            MotionMode::Stopped => CiA402ModeOfOperation::NoMode,
            MotionMode::Position => CiA402ModeOfOperation::ProfilePositionMode,
            MotionMode::Velocity => CiA402ModeOfOperation::ProfileVelocityMode,
            MotionMode::Torque => CiA402ModeOfOperation::ProfileTorqueMode,
            _ => {
                error!("Mode {:?} not supported!", mode);
                CiA402ModeOfOperation::NoMode
            }
        }
    }
}
