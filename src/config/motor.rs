use crate::motor_control::Pid;

pub struct BrushlessMotor {
    // number of pole pairs 
    n_pole_pairs: u32,

    // PID gains of the motor controllers
    // using only P and I gains
    pid_flux: Pid,
    pid_torque:Pid,
    pid_velocity: Pid,
    pid_position: Pid,
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
            // 4 pole pairs for the ecx22
            n_pole_pairs: 4,

            // the encoder with 4096 ppr
            abn_decoder_ppr: 0x00001000,
            // PI controller params
            pid_flux: Pid{p:0x200, i:0x200},
            pid_torque: Pid{p:0x200, i:0x200},
            pid_velocity: Pid{p:0x200, i:0x8},
            pid_position: Pid{p:0x200, i:0x0},

            // gearing ratios
            gearbox_ratio: 1.0/35.0,
            axis_ratio: 12.0/64.0,

        }
    }
    #[allow(dead_code)]
    pub fn ec60() -> Self {
        Self {
            // 7 pole pairs for the ec60
            n_pole_pairs: 7,

            // the encoder with 4096 ppr
            abn_decoder_ppr: 0x00001000,
            // PI controller params
            pid_flux: Pid{p:44, i:120},
            pid_torque: Pid{p:44, i:120},
            pid_velocity: Pid{p:600, i:400},
            pid_position: Pid{p:50, i:0},

            // gearing ratios
            gearbox_ratio: 1.0/25.01,
            axis_ratio: 28.0/52.0,
        }
    }
    #[allow(dead_code)]
    pub fn ec45() -> Self {
        Self {
            // 8 pole pairs for the ec45
            n_pole_pairs: 8,

            // the encoder with 4096 ppr
            abn_decoder_ppr: 0x00001000,

            // PI controller params
            pid_flux: Pid{p:0x100, i:0x100},
            pid_torque: Pid{p:0x100, i:0x100},
            pid_velocity: Pid{p:0x800, i:0},
            pid_position: Pid{p:0x100, i:0},

            // gearing ratios
            gearbox_ratio: 1.0,
            axis_ratio: 20.0/38.0,
        }
    }
}

impl BrushlessMotor {

    // 0x00030004: 3-phase brushless motor, 4 pole pairs - register MOTOR_TYPE_N_POLE_PAIRS
    pub fn motor_type_n_pole_pairs(&self) -> u32 {
        0x00030000 | (self.n_pole_pairs & 0x0000FFFF)
    }

    pub fn pid_flux_p_flux_i(&self) -> u32{
        self.pid_to_reg(self.pid_flux)
    }
    pub fn pid_torque_p_torque_i(&self) -> u32{
        self.pid_to_reg(self.pid_torque)
    }
    pub fn pid_position_p_position_i(&self) -> u32{
        self.pid_to_reg(self.pid_position)
    }
    pub fn pid_velocity_p_velocity_i(&self) -> u32{
        self.pid_to_reg(self.pid_velocity)
    }

    pub fn gearbox_ratio(&self) -> f32 {
	    self.gearbox_ratio
    }
    pub fn axis_ratio(&self) -> f32 {
	    self.axis_ratio
    }
    pub fn pole_pairs(&self) -> f32 {
	    self.n_pole_pairs as f32
    }
    
    pub fn abn_decoder_ppr(&self) -> u32 {
	    self.abn_decoder_ppr
    }


    // conversion of the pid class to the TMC register value
    // bits  0..15 - I gain
    // bits 16..32 - P gain
    fn pid_to_reg(&self, pid: Pid) -> u32{
        return ((pid.p as u32) << 16) | ((pid.i  as u32) & 0xFFFF)
    }

    // conversion from electrical to mechanical angle
    // depending on the features enabled, 
    // feature "gearbox_output" returns the angle after the gearbox
    // feature "axis_output" returns the angle after the gearbox and axis
    // if neither of the above features are enabled, the motor angle is returned
    pub fn angle_elec_to_mech(&self, angle: f32) -> f32 {
        #[cfg(feature = "gearbox_output")]
        return self.elec_to_gearbox(angle);
        #[cfg(feature = "axis_output")]
        return self.elec_to_axis(angle);
        // ide neither of the above features are enabled, the motor angle is returned
        #[cfg(not(any(feature = "gearbox_output", feature = "axis_output")))]
        return self.elec_to_shaft(angle);
    }
    // conversion from mechanical to electrical angle
    // depends on the features enabled
    // feature "gearbox_output" considers the angle after the gearbox
    // feature "axis_output" considers the angle after the gearbox and axis
    // if neither of the above features are enabled, the motor angle is returned
    pub fn angle_mech_to_elec(&self, angle: f32) -> f32 {

        #[cfg(feature = "gearbox_output")]
        return self.gearbox_to_elec(angle);
        #[cfg(feature = "axis_output")]
        return self.axis_to_elec(angle);
        // ide neither of the above features are enabled, the motor angle is returned
        #[cfg(not(any(feature = "gearbox_output", feature = "axis_output")))]
        return self.shaft_to_elec(angle);
    }

    // conversion from electrical to mechanical angle
    pub fn elec_to_shaft(&self, angle: f32) -> f32 {
        angle / self.pole_pairs()
    }
    // from electrical angle to the angle of the motor after the gearbox
    pub fn elec_to_gearbox(&self, angle: f32) -> f32 {
        self.elec_to_shaft(angle) * self.gearbox_ratio
    }
    // from electrical angle to the angle of the motor after the gearbox and axis
    pub fn elec_to_axis(&self, angle: f32) -> f32 {
        self.elec_to_gearbox(angle) * self.axis_ratio
    }
    // conversion from mechanical to electrical angle
    pub fn shaft_to_elec(&self, angle: f32) -> f32 {
        angle * self.pole_pairs() 
    }
    // from mechanical angle after the gearbox to electrical angle
    pub fn gearbox_to_elec(&self, angle: f32) -> f32 {
        self.shaft_to_elec(angle) / self.gearbox_ratio
    }
    // from mechanical angle after the gearbox and axis to electrical angle
    pub fn axis_to_elec(&self, angle: f32) -> f32 {
        self.gearbox_to_elec(angle) / self.axis_ratio
    }

}