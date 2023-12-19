pub enum DynamixelRegister {
    TorqueEnable,
    CurrentPosition,
    CurrentVelocity,
    CurrentTorque,
    TargetPosition,
    // VelocityLimit,
    // TorqueLimit,

    // PIDGains,
    AxisSensor
}

impl DynamixelRegister {
    pub fn with_address(address: u8) -> Option<Self> {
        match address {
            40 => Some(DynamixelRegister::TorqueEnable),
            50 => Some(DynamixelRegister::CurrentPosition),
            51 => Some(DynamixelRegister::CurrentVelocity),
            52 => Some(DynamixelRegister::CurrentTorque),
            60 => Some(DynamixelRegister::TargetPosition),

            // 70 => Some(DynamixelRegister::VelocityLimit),
            // 71 => Some(DynamixelRegister::TorqueLimit),

            // 80 => Some(DynamixelRegister::PIDGains),
	    90 => Some(DynamixelRegister::AxisSensor),

	    _ => None


        }
    }
}
