pub enum DynamixelRegister {
    ModelNumber,
    FirmwareVersion,
    Id,
    VelocityLimit,
    TorqueLimit,
    // FluxPID,
    TorquePID,
    VelocityPID,
    PositionPID,

    TorqueEnable,

    CurrentPosition,
    CurrentVelocity,
    CurrentTorque,
    TargetTorque,
    TargetVelocity,
    TargetPosition,

    AxisSensor
}

impl DynamixelRegister {
    pub fn with_address(address: u8) -> Option<Self> {
        match address {
	    0 => Some(DynamixelRegister::ModelNumber),
	    6 => Some(DynamixelRegister::FirmwareVersion),
	    7 => Some(DynamixelRegister::Id),




	    10 => Some(DynamixelRegister::VelocityLimit),
	    14 => Some(DynamixelRegister::TorqueLimit),




            40 => Some(DynamixelRegister::TorqueEnable),
            50 => Some(DynamixelRegister::CurrentPosition),
            51 => Some(DynamixelRegister::CurrentVelocity),
            52 => Some(DynamixelRegister::CurrentTorque),
            60 => Some(DynamixelRegister::TargetPosition),

	    90 => Some(DynamixelRegister::AxisSensor),

	    _ => None


        }
    }
}
