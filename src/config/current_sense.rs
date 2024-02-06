pub struct CurrentSensing {
    
    // current sensing parameters
    // Shunt resistor value
    resistance_shunt: f32, // [Ohms]
    // gain of the amplifier
    amp_gain: f32, // [V/V]
    amp_voltage: f32, // [V]

    // adc offset and scale values - register ADC_I0_SCALE_OFFSET and ADC_I1_SCALE_OFFSET
    adc_i0_scale_offset: u32,
    adc_i1_scale_offset: u32,
}

impl CurrentSensing {
    #[allow(dead_code)]
    pub fn wailer_B2() -> Self {
        let mut return_struct = Self {
            // current sensing parameters
            resistance_shunt: 0.003, // [Ohms]
            amp_gain: 20.0, // [V/V]gain of the amplifier
            amp_voltage: 5.0, // [V]
            // middle of the range  and scale 1
            adc_i0_scale_offset: 0x01000000 | (32768u32),
            adc_i1_scale_offset: 0x01000000 | (32768u32),
        };
        // update the default offset values
        // this will be automated later on 
        #[cfg(feature = "ecx22")]
        return_struct.set_adc_offsets(0x81D3, 0x825B);
        #[cfg(feature = "ec60")]
        return_struct.set_adc_offsets(0x81FA,0x826C);
        #[cfg(feature = "ec45")]
        return_struct.set_adc_offsets(0x819E, 0x821C);

        return return_struct

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
    pub fn adc_to_mAmps(&self, adc_raw: f32, adc_resolution:f32) -> f32{
        adc_raw * ( self.amp_voltage / adc_resolution)/(self.amp_gain*self.resistance_shunt) * 1000.0 // amp_volt/adc_res -> counts to volts, 1/shunt*gain -> volts to amps
    }
    // transforming milliamps to raw adc counts
    pub fn mAmps_tp_adc(&self, amps: f32, adc_resolution:f32) -> f32{
        amps / 1000.0 * (self.amp_gain*self.resistance_shunt) / (self.amp_voltage / adc_resolution) // shunt*gain -> amps to volts, adc_res/amp_volt -> volts to adc counts 
    }
}
