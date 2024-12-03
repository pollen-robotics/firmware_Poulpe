use embassy_stm32::adc::{Adc, AdcPin, Instance, SampleTime};
use embassy_stm32::gpio::Pin;
use embassy_stm32::peripherals as p;
use embassy_time::{Delay, Timer};
use libm::log;

use crate::utils::errors::{IOError, Result};

pub struct AnalogInputConfig<T, P>
where
    T: Instance,
    P: AdcPin<T> + Pin,
{
    pub adc: T,
    pub pin1: P,
}

pub struct Orbita2dTemperatureConfig<T, P1, P2>
where
    T: Instance,
    P1: AdcPin<T> + Pin,
    P2: AdcPin<T> + Pin,
{
    pub adc: T,
    pub pin1: P1,
    pub pin2: P2,
}

pub struct Orbita3dTemperatureConfig<T, P1, P2, P3>
where
    T: Instance,
    P1: AdcPin<T> + Pin,
    P2: AdcPin<T> + Pin,
    P3: AdcPin<T> + Pin,
{
    pub adc: T,
    pub pin1: P1,
    pub pin2: P2,
    pub pin3: P3,
}

pub fn adc_setup<T: Instance>(adc: &mut T) -> Adc<T> {
    let mut adc = Adc::new(adc, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles32_5);
    adc
}

pub fn adc_read_input<T: Instance, P: AdcPin<T> + Pin>(adc: &mut Adc<T>, pin: &mut P) -> f32 {
    let value: u16 = adc.read(pin);
    let value = value as f32;
    value
}

pub fn adc_read_voltage<T: Instance, P: AdcPin<T> + Pin>(adc: &mut Adc<T>, pin: &mut P) -> f32 {
    let value: f32 = adc_read_input(adc, pin);
    let voltage: f32 = value * 3.3 / 65535.0;
    voltage
}

pub fn adc_read_temperature<T: Instance, P: AdcPin<T> + Pin>(
    adc: &mut Adc<T>,
    pin: &mut P,
) -> Result<f32> {
    let voltage: f32 = adc_read_voltage(adc, pin);
    // Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
    let r_div: f32 = 4700.0;
    #[cfg(all(feature = "pvt", feature = "orbita3d"))]
    let beta: f32 = 3435.0;
    #[cfg(not(all(feature = "pvt", feature = "orbita3d")))]
    let beta: f32 = 3425.0;
    let room_temp_inv: f32 = 1.0 / 298.15; //[K]
    let r_t: f32 = r_div * ((3.3 / voltage) - 1.0);
    #[cfg(all(feature = "pvt", feature = "orbita3d"))]
    let r_25: f32 = 10000.0;
    #[cfg(not(all(feature = "pvt", feature = "orbita3d")))]
    let r_25: f32 = 5000.0;

    let mut t: f32 = 1.0 / (((log((r_t / r_25) as f64) as f32) / beta) + room_temp_inv);

    match t {
        t if t.is_nan() => Err(IOError::InvalidData),
        _ => Ok((t as f32) - 273.15), // final conversion to Celsius
    }
}
