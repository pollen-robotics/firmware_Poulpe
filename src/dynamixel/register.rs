pub enum DynamixelRegister {
    ModelNumber,
    FirmwareVersion,
    Id,

    BoardState,

    FluxPID,
    TorquePID,
    VelocityPID,
    PositionPID,

    UqUdLimit,
    TorqueFluxLimit,
    VelocityLimit,
    TorqueFluxLimitMax,
    VelocityLimitMax,

    TorqueEnable,

    CurrentPosition,
    CurrentVelocity,
    CurrentTorque,
    TargetTorque,
    TargetVelocity,
    FeedforwardVelocity,
    TargetPosition,
    TargetPositionWithVelocityFF,
    TargetPositionEstimateVelocityFF,

    Temperature,
    BusVoltage,

    AxisSensor,

    #[cfg(feature = "orbita3d")]
    IndexSensor,

    FullState,
}

impl DynamixelRegister {
    pub fn with_address(address: u8) -> Option<Self> {
        match address {
            0 => Some(DynamixelRegister::ModelNumber),
            6 => Some(DynamixelRegister::FirmwareVersion),
            7 => Some(DynamixelRegister::Id),

            10 => Some(DynamixelRegister::VelocityLimit),
            12 => Some(DynamixelRegister::VelocityLimitMax),
            14 => Some(DynamixelRegister::TorqueFluxLimit),
            16 => Some(DynamixelRegister::TorqueFluxLimitMax),
            18 => Some(DynamixelRegister::UqUdLimit),

            20 => Some(DynamixelRegister::FluxPID),
            24 => Some(DynamixelRegister::TorquePID),
            28 => Some(DynamixelRegister::VelocityPID),
            32 => Some(DynamixelRegister::PositionPID),

            40 => Some(DynamixelRegister::TorqueEnable),
            51 => Some(DynamixelRegister::CurrentVelocity),
            50 => Some(DynamixelRegister::CurrentPosition),
            54 => Some(DynamixelRegister::FeedforwardVelocity),
            52 => Some(DynamixelRegister::CurrentTorque),
            60 => Some(DynamixelRegister::TargetPosition),
            62 => Some(DynamixelRegister::TargetPositionWithVelocityFF),
            64 => Some(DynamixelRegister::TargetPositionEstimateVelocityFF),

            70 => Some(DynamixelRegister::Temperature),
            72 => Some(DynamixelRegister::BusVoltage),

            80 => Some(DynamixelRegister::BoardState),

            90 => Some(DynamixelRegister::AxisSensor),

            #[cfg(feature = "orbita3d")]
            99 => Some(DynamixelRegister::IndexSensor),

            100 => Some(DynamixelRegister::FullState),

            _ => None,
        }
    }
}
