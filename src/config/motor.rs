pub struct BrushlessMotor {
    pid_flux_p_flux_i: u32,
    pid_torque_p_torque_i: u32,
    pid_velocity_p_velocity_i: u32,
    pid_position_p_position_i: u32,
}

impl BrushlessMotor {
    #[allow(dead_code)]
    pub fn ecx22() -> Self {
        Self {
            pid_flux_p_flux_i: 0x03200080,
            pid_torque_p_torque_i: 0x03200000,
            pid_velocity_p_velocity_i: 0x01000080,
            pid_position_p_position_i: 0x00400010,
        }
    }
    #[allow(dead_code)]
    pub fn ec60() -> Self {
        Self {
            pid_flux_p_flux_i: 0x03200000,
            pid_torque_p_torque_i: 0x03200000,
            pid_velocity_p_velocity_i: 0x01F401C2,
            pid_position_p_position_i: 0x00500000,
        }
    }
    #[allow(dead_code)]
    pub fn ec45() -> Self { //TODO
        Self {
            pid_flux_p_flux_i: 0x03200000,
            pid_torque_p_torque_i: 0x03200000,
            pid_velocity_p_velocity_i: 0x01F401C2,
            pid_position_p_position_i: 0x00500000,
        }
    }
}

impl BrushlessMotor {
    pub fn pid_flux_p_flux_i(&self) -> u32 {
        self.pid_flux_p_flux_i
    }
    pub fn pid_torque_p_torque_i(&self) -> u32 {
        self.pid_torque_p_torque_i
    }
    pub fn pid_velocity_p_velocity_i(&self) -> u32 {
        self.pid_velocity_p_velocity_i
    }
    pub fn pid_position_p_position_i(&self) -> u32 {
        self.pid_position_p_position_i
    }
}
