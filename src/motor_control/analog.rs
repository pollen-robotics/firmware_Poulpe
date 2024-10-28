use embassy_stm32::adc::{Adc, AdcPin, Instance, SampleTime};
use embassy_stm32::gpio::Pin;
use embassy_stm32::peripherals as p;
use embassy_time::{Delay, Timer};
use libm::log;

use defmt::*;

use crate::motor_control::motors_io::IOError;

pub struct AnalogInputConfig<T, P>
where
    T: Instance,
    P: AdcPin<T> + Pin,
{
    pub adc: T,
    pub pin: P,
}

pub struct AnalogInput<'d, T, P>
where
    T: Instance,
    P: AdcPin<T> + Pin,
{
    adc: Adc<'d, T>,
    pin: P,
}

impl<'d, T, P> AnalogInput<'d, T, P>
where
    T: Instance,
    P: AdcPin<T> + Pin,
{
    pub fn new(config: AnalogInputConfig<T, P>) -> Self {
        let mut adc = Adc::new(config.adc, &mut Delay);
        adc.set_sample_time(SampleTime::Cycles32_5);
        Self {
            adc,
            pin: config.pin,
        }
    }

    pub fn read(&mut self) -> f32 {
        let value: u16 = self.adc.read(&mut self.pin);
        let value = value as f32;
        value
    }

    pub fn read_voltage(&mut self) -> f32 {
        let value: u16 = self.adc.read(&mut self.pin);
        let value = value as f32;
        value * 3.3 / 65535.0
    }

    pub fn read_temperature(&mut self) -> Result<(f32), IOError> {
        let voltage: f32 = self.read_voltage();
        // Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
        let r_div: f32 = 4700.0;
        let beta: f32 = 3425.0;
        let room_temp_inv: f32 = 1.0 / 298.15; //[K]
        let r_t: f32 = r_div * ((3.3 / voltage) - 1.0);
        let r_25: f32 = 5000.0;

        let mut t: f32 = 1.0 / (((log((r_t / r_25) as f64) as f32) / beta) + room_temp_inv);

        info!("Temperature: {}, volt: {}", t, voltage);

        match t {
            t if t.is_nan() => Err(IOError::InvalidData),
            _ => Ok((t as f32) - 273.15), // final conversion to Celsius
        }
    }
}
