use defmt::Format;
use embassy_stm32::i2c;
use embassy_stm32::spi;

use crate::utils::errors::{Result, IOError};

use embassy_time::{ Duration, Instant, Ticker, Timer};

use super::sensors;

use micromath::F32Ext;
use defmt::*;

pub trait RawSensorsIO<const N: usize> {
    /// Get sensors value
    fn get_axis_sensors(&mut self) -> Result<[f32; N]>;
    // fn get_index_sensors(&mut self) -> Result<[u16; N]>;

}


 // read the axis sensors
// This function reads the sensors n_read times and takes the average and deviation to check if the sensor is stable
// If the deviation is too high, it retries n_read_tries times
pub async fn get_axis_sensors_robust<'d, const N: usize>(
    sensor: &mut dyn RawSensorsIO<N>,
    n_read_tries: u8,
    n_read: u32,
) -> Result<[f32; N]> {

    // stop for a bit to avoid the noise
    Timer::after(Duration::from_micros(100000)).await;

    let n_tries_max = n_read_tries;
    let mut n_read_tries = n_read_tries;
    // make a few tries to avoid nan values:
    let sensor_reads = loop {
        n_read_tries = n_read_tries - 1;
        if n_read_tries == 0 {
            error!("Error reading axis sensors: too many tries ({}), retrying...", n_tries_max);
            return Err(IOError::SensorError);
        }

        // We read n_read time the sensor and take the average and deviation to check if the sensor is stable
        let mut sensor_reads_avg: [f32; N] = [0.0; N];
        let mut sensor_reads_std: [f32; N] = [0.0; N];
        let mut sensor_reads_M2: [f32; N] = [0.0; N];

        let mut n: f32 = 0.0;
        for _ in 0..n_read {
            n = n + 1.0;
            match sensor.get_axis_sensors() {
                Ok(sensors) => {
                    if sensors.iter().any(|x| x.is_nan()) {
                        error!("Nan values in sensors, retrying...");
                        #[cfg(not(feature = "ignore_errors"))] // dont wait if ignoring errors
                        Timer::after(Duration::from_micros(100000)).await; // wait for a bit
                        continue;
                    }
                    // break sensors;
                    let mut delta: [f32; N] = [0.0; N];
                    for s in 0..N {
                        delta[s] = sensors[s] - sensor_reads_avg[s];
                        sensor_reads_avg[s] = sensor_reads_avg[s] + delta[s] / n;
                        sensor_reads_M2[s] = sensor_reads_M2[s]
                            + F32Ext::sqrt(delta[s] * (sensors[s] - sensor_reads_avg[s]));
                        sensor_reads_std[s] = sensor_reads_M2[s] / n;
                    }
                }
                Err(e) => {
                    error!("Error reading axis sensors: {:?}", e);
                    #[cfg(not(feature = "ignore_errors"))] // dont wait if ignoring errors
                    Timer::after(Duration::from_micros(100000)).await;
                    continue; //  retry the init if the read
                }
            }
        }
        info!(
            "Sensor avg: {:?} sensor deviation: {:?}",
            sensor_reads_avg, sensor_reads_std
        );
        let mut should_retry: bool = false;
        for s in 0..N {
            if sensor_reads_std[s] > 1e-3 {
                error!("Sensor deviation is to high!");
                should_retry = true;
            }
        }
        if should_retry {
            continue;
        }
        break sensor_reads_avg;
    };

    Ok(sensor_reads)
}