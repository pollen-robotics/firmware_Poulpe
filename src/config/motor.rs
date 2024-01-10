pub struct BrushlessMotor {
    motor_type_n_pole_pairs: u32,
    adc_i0_scale_offset: u32,
    adc_i1_scale_offset: u32,

    pid_flux_p_flux_i: u32,
    pid_torque_p_torque_i: u32,
    pid_velocity_p_velocity_i: u32,
    pid_position_p_position_i: u32,

    gearbox_ratio: f32,
    axis_ratio: f32,

}

impl BrushlessMotor {
    #[allow(dead_code)]
    pub fn ecx22() -> Self {
        Self {
	    motor_type_n_pole_pairs: 0x00030004,

	    //TODO!
	    adc_i0_scale_offset: 0x010081D3,
	    adc_i1_scale_offset: 0x0100825B,


            pid_flux_p_flux_i: 0x02000200,
            pid_torque_p_torque_i: 0x02000200,
            pid_velocity_p_velocity_i: 0x02000008,
            pid_position_p_position_i: 0x02000000,

	    gearbox_ratio: 1.0/35.0,
	    axis_ratio: 12.0/64.0,

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
	    gearbox_ratio: 1.0/35.0,
	    axis_ratio: 20.0/38.0,

        }
    }
    #[allow(dead_code)]
    pub fn ec45() -> Self {
        Self {
	    motor_type_n_pole_pairs: 0x00030004,
	    // adc_i0_scale_offset: 0x002A819E, //Ventouse?
	    // adc_i1_scale_offset: 0x002A821C,
	    adc_i0_scale_offset: 0x0100819E,
	    adc_i1_scale_offset: 0x0100821C,



            // pid_flux_p_flux_i: 0x02000200,
            pid_flux_p_flux_i: 0x01000100,
            pid_torque_p_torque_i: 0x01000100,

            pid_velocity_p_velocity_i: 0x04000040,
            pid_position_p_position_i: 0x00800000,
	    gearbox_ratio: 1.0/35.0,
	    axis_ratio: 20.0/38.0,



        }
    }
}

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

    pub fn gearbox_ratio(&self) -> f32 {
	self.gearbox_ratio
    }
    pub fn axis_ratio(&self) -> f32 {
	self.axis_ratio
    }


}
