pub struct BrushlessMotor {
    motor_type_n_pole_pairs: u32,
    adc_i0_scale_offset: u32,
    adc_i1_scale_offset: u32,

    pid_flux_p_flux_i: u32,
    pid_torque_p_torque_i: u32,
    pid_velocity_p_velocity_i: u32,
    pid_position_p_position_i: u32,
}

impl BrushlessMotor {
    #[allow(dead_code)]
    pub fn ecx22() -> Self {
        Self {
	    motor_type_n_pole_pairs: 0x00030004,

	    //TODO!
	    adc_i0_scale_offset: 0x010081FA,
	    adc_i1_scale_offset: 0x0100826C,

            pid_flux_p_flux_i: 0x03200080,
            pid_torque_p_torque_i: 0x03200000,
            pid_velocity_p_velocity_i: 0x01000080,
            pid_position_p_position_i: 0x00400010,
        }
    }
    #[allow(dead_code)]
    pub fn ec60() -> Self {
        Self {
	    motor_type_n_pole_pairs: 0x00030007,

	    //TODO!
	    adc_i0_scale_offset: 0x010081FA,
	    adc_i1_scale_offset: 0x0100826C,

            pid_flux_p_flux_i: 0x03200000,
            pid_torque_p_torque_i: 0x03200000,
            pid_velocity_p_velocity_i: 0x01F401C2,
            pid_position_p_position_i: 0x00500000,
        }
    }

    #[allow(dead_code)]
    pub fn ec45() -> Self {
        Self {
	    motor_type_n_pole_pairs: 0x00030008,
	    adc_i0_scale_offset: 0x010081FA,
	    adc_i1_scale_offset: 0x0100826C,

            pid_flux_p_flux_i: 0x01000000,
            pid_torque_p_torque_i: 0x01000000,
            pid_velocity_p_velocity_i: 0x01000400,
            pid_position_p_position_i: 0x00800010,
        }
    }

}

/*
#[cfg(feature = "ec45")]
pub mod MotorConfig {
    pub const MOTOR_TYPE_N_POLE_PAIRS: u32 = 0x00030008;
    pub const ADC_I0_SCALE_OFFSET: u32 = 0x010081FA;
    pub const ADC_I1_SCALE_OFFSET: u32 = 0x0100826C;

    pub const PID_FLUX_P_FLUX_I: u32 = 0x01000000;
    pub const PID_TORQUE_P_TORQUE_I: u32 = 0x01000000;
    pub const PID_VELOCITY_P_VELOCITY_I: u32 = 0x01000400;
    pub const PID_POSITION_P_POSITION_I: u32 = 0x00800010;
}
*/

impl BrushlessMotor {
    pub fn motor_type_n_pole_pairs(&self) -> u32 {
        self.motor_type_n_pole_pairs
    }
    pub fn adc_i0_scale_offset(&self) -> u32 {
	self.adc_i0_scale_offset
    }
    pub fn adc_i1_scale_offset(&self) -> u32 {
	self.adc_i1_scale_offset
    }

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
