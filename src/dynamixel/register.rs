pub enum DynamixelRegister {
    ModelNumber,
    FirmwareVersion,
    Id,


    FluxPID,
    TorquePID,
    VelocityPID,
    PositionPID,

    UqUdLimit,
    TorqueFluxLimit,
    VelocityLimit,

    TorqueEnable,

    CurrentPosition,
    CurrentVelocity,
    CurrentTorque,
    TargetTorque,
    TargetVelocity,
    TargetPosition,

    AxisSensor,

    FullState,


}

impl DynamixelRegister {
    pub fn with_address(address: u8) -> Option<Self> {
        match address {
	    0 => Some(DynamixelRegister::ModelNumber),
	    6 => Some(DynamixelRegister::FirmwareVersion),
	    7 => Some(DynamixelRegister::Id),




	    10 => Some(DynamixelRegister::VelocityLimit),
	    14 => Some(DynamixelRegister::TorqueFluxLimit),
	    18 => Some(DynamixelRegister::UqUdLimit),

	    20 => Some(DynamixelRegister::FluxPID),
	    24 => Some(DynamixelRegister::TorquePID),
	    28 => Some(DynamixelRegister::VelocityPID),
	    32 => Some(DynamixelRegister::PositionPID),



            40 => Some(DynamixelRegister::TorqueEnable),
            50 => Some(DynamixelRegister::CurrentPosition),
            51 => Some(DynamixelRegister::CurrentVelocity),
            52 => Some(DynamixelRegister::CurrentTorque),
            60 => Some(DynamixelRegister::TargetPosition),

	    90 => Some(DynamixelRegister::AxisSensor),

	    100 => Some(DynamixelRegister::FullState),

	    _ => None


        }
    }
}
