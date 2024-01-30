pub struct BrushlessMotor {
    // 0x00030004: 3-phase brushless motor, 4 pole pairs - register MOTOR_TYPE_N_POLE_PAIRS
    motor_type_n_pole_pairs: u32,
    // adc offset and scale values - register ADC_I0_SCALE_OFFSET and ADC_I1_SCALE_OFFSET
    adc_i0_scale_offset: u32,
    adc_i1_scale_offset: u32,

    // pid flux and torque P and I gains - register PID_FLUX_P_FLUX_I and PID_TORQUE_P_TORQUE_I
    pid_flux_p_flux_i: u32,
    pid_torque_p_torque_i: u32,
    // pid velocity and position P and I gains - register PID_VELOCITY_P_VELOCITY_I and PID_POSITION_P_POSITION_I
    pid_velocity_p_velocity_i: u32,
    pid_position_p_position_i: u32,
    // The encoder PPR value - register ABN_DECODER_PPR
    abn_decoder_ppr: u32,
    // ratio of motor's gearbox
    gearbox_ratio: f32,
    // additional reduction ration of the axis
    axis_ratio: f32,

}

impl BrushlessMotor {
    #[allow(dead_code)]
    pub fn ecx22() -> Self {
        Self {
            // 0x00030004: 3-phase brushless motor, 4 pole pairs
            motor_type_n_pole_pairs: 0x00030004,

            //TODO!
            adc_i0_scale_offset: 0x010081D3,
            adc_i1_scale_offset: 0x0100825B,

            // the encoder with 4096 ppr
            abn_decoder_ppr: 0x00001000,
            // PI controller params
            pid_flux_p_flux_i: 0x02000200,
            pid_torque_p_torque_i: 0x02000200,
            pid_velocity_p_velocity_i: 0x02000008,
            pid_position_p_position_i: 0x02000000,

            // gearing ratios
            gearbox_ratio: 1.0/35.0,
            axis_ratio: 12.0/64.0
        }
    }
    #[allow(dead_code)]
    pub fn ec60() -> Self {
        Self {
            // 0x00030007: 3-phase brushless motor, 7 pole pairs
            motor_type_n_pole_pairs: 0x00030007,

            //TODO!
            adc_i0_scale_offset: 0x010081FA,
            adc_i1_scale_offset: 0x0100826C,

            // the encoder with 4096 ppr
            abn_decoder_ppr: 0x00001000,
            // PI controller params
            pid_flux_p_flux_i: 0x03200000,
            pid_torque_p_torque_i: 0x03200000,
            pid_velocity_p_velocity_i: 0x01F401C2,
            pid_position_p_position_i: 0x00500000,

            // gearing ratios
            gearbox_ratio: 1.0/25.0,
            axis_ratio: 20.0/38.0

        }
    }
    #[allow(dead_code)]
    pub fn ec45() -> Self {
        Self {
            // 0x00030007: 3-phase brushless motor, 8 pole pairs
            motor_type_n_pole_pairs: 0x00030008,

            // adc_i0_scale_offset: 0x002A819E, //Ventouse?
            // adc_i1_scale_offset: 0x002A821C,
            adc_i0_scale_offset: 0x0100819E,
            adc_i1_scale_offset: 0x0100821C,

            // the encoder with 4096 ppr
            abn_decoder_ppr: 0x00001000,

            // PI controller params
            // pid_flux_p_flux_i: 0x02000200,
            pid_flux_p_flux_i: 0x01000100,
            pid_torque_p_torque_i: 0x01000100,
            pid_velocity_p_velocity_i: 0x08000000,
            pid_position_p_position_i: 0x01000000,

            // gearing ratios
            gearbox_ratio: 1.0,
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
    pub fn pole_pairs(&self) -> f32 {
	(self.motor_type_n_pole_pairs & 0x0000FFFF) as f32
    }
    // transformation gain form mechanical to electrical revolutions 
    pub fn mech_to_elec(&self) -> f32 {
	self.pole_pairs()/self.gearbox_ratio()
    }

    pub fn abn_decoder_ppr(&self) -> u32 {
	self.abn_decoder_ppr
    }


}
