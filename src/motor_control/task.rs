use embassy_time::{Duration, Timer};

use crate::{config, SHARED_MEMORY};

use super::{Actuator, RawMotorsIO};

#[embassy_executor::task]
pub async fn control_loop(actuator: Actuator<{ config::N_AXIS }>) {
    let mut actuator = actuator;

    actuator.init().await;

    loop {
        let pos = actuator.get_current_position().unwrap();
        {
            SHARED_MEMORY.lock().await.set_current_position(pos)
        }

        let target = { SHARED_MEMORY.lock().await.get_target_position() };
        actuator.set_target_position(target).unwrap();

        Timer::after(Duration::from_millis(1)).await;
    }
}
