use defmt::debug;

pub struct CurrentSensing {
    // current sensing parameters
    // Shunt resistor value
    resistance_shunt: f32, // [Ohms]
    // gain of the amplifier
    amp_gain: f32,    // [V/V]
    amp_voltage: f32, // [V]

    // adc offset and scale values - register ADC_I0_SCALE_OFFSET and ADC_I1_SCALE_OFFSET
    adc_i0_scale_offset: u32,
    adc_i1_scale_offset: u32,
    
}

impl CurrentSensing {
    #[allow(dead_code)]
    pub fn ventouse_bob() -> Self {
        Self {
            // current sensing parameters
            resistance_shunt: 0.003, // 0.003 [Ohms]
            amp_gain: 20.0,          // [V/V]gain of the amplifier
            amp_voltage: 5.0,        // [V]
            // middle of the range  and scale 1
            // values ignored - configured automatically 
            adc_i0_scale_offset: 0x01000000 | (0x8000), 
            adc_i1_scale_offset: 0x01000000 | (0x8000), 
        }
    }
    #[allow(dead_code)]
    pub fn ventouse_2d() -> Self{
        Self {
            // current sensing parameters
            resistance_shunt: 0.01, // 0.01 [Ohms]
            amp_gain: 20.0,          // [V/V]gain of the amplifier
            amp_voltage: 5.0,        // [V]
            // middle of the range  and scale 1
            // values ignored - configured automatically 
            adc_i0_scale_offset: 0x01000000 | (0x8000), 
            adc_i1_scale_offset: 0x01000000 | (0x8000), 
        }
    }
    #[allow(dead_code)]
    pub fn ventouse_3d() -> Self{
        Self {
            // current sensing parameters
            resistance_shunt: 1.0,  // no shunt resistor
            amp_gain: 0.15,         // 0.15 [V/A] gain of the DRV8316 amplifier 
                                    // IMPORTANT: real gain is of DRV8316 is 0.3 V/A
                                    //  but the current measured is just a half of the real value
                                    //  This is an empirical conclusion, done in comparison with the BOB!!!
            amp_voltage: 5.0,       // [V]
            // middle of the range  and scale 1
            // values ignored - configured automatically 
            adc_i0_scale_offset: 0x01000000 | (0x8000), 
            adc_i1_scale_offset: 0x01000000 | (0x8000), 
        }
    }
}

impl CurrentSensing {
    pub fn adc_i0_scale_offset(&self) -> u32 {
        self.adc_i0_scale_offset
    }
    pub fn adc_i1_scale_offset(&self) -> u32 {
        self.adc_i1_scale_offset
    }
    pub fn resistance_shunt(&self) -> f32 {
        self.resistance_shunt
    }
    pub fn amp_gain(&self) -> f32 {
        self.amp_gain
    }
    pub fn amp_voltage(&self) -> f32 {
        self.amp_voltage
    }

    pub fn set_adc_offsets(&mut self, i0: u32, i1: u32) {
        self.adc_i0_scale_offset = 0x01000000 | i0;
        self.adc_i1_scale_offset = 0x01000000 | i1;
    }
    // transforming the raw adc counts to milliamps
    pub fn adc_to_mAmps(&self, adc_raw: f32, adc_resolution: f32) -> f32 {
        adc_raw * (self.amp_voltage / adc_resolution) / (self.amp_gain * self.resistance_shunt)
            * 1000.0 // amp_volt/adc_res -> counts to volts, 1/shunt*gain -> volts to amps
    }
    // transforming milliamps to raw adc counts
    pub fn mAmps_to_adc(&self, amps: f32, adc_resolution: f32) -> f32 {
        amps / 1000.0 * (self.amp_gain * self.resistance_shunt)
            / (self.amp_voltage / adc_resolution) // shunt*gain -> amps to volts, adc_res/amp_volt -> volts to adc counts
    }
}
