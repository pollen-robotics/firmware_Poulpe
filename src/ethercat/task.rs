use crate::{
    config::{self, LAN9252Config, N_AXIS},
    SHARED_MEMORY,
};
use core::{cell::RefCell, default, mem::take};
use defmt::{debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
};
use embassy_stm32::{spi};

use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Duration, Instant, Timer};
use crate::state_machine::poulpe_state::{PoulpeState};
use crate::ethercat::lan9252::{Lan9252, Lan9252Registers, WDOG_STATUS, AL_STATUS};

// the addresses of the motors in the LAN9252 memory
pub enum Lan9252Memory {
    OrbitaIn = 0x1000, // default write address 0x1000
    OrbitaStatus = 0x1200,
    OrbitaOut = 0x1300,
}

#[embassy_executor::task]
pub async fn messsage_handler(ethconf: LAN9252Config, spi_config: spi::Config) {
    warn!("ETHERCAT TASK");

    let spi = spi::Spi::new(
        ethconf.peri,
        ethconf.sck,
        ethconf.mosi,
        ethconf.miso,
        NoDma,
        NoDma,
        spi_config,
    );
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));

    let eth_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(ethconf.cs, Level::High, Speed::High),
        spi_config,
    );

    let mut lan9252 = Lan9252::new(eth_spi);
    match lan9252.init().await {
        Ok(_) => {
            debug!("LAN9252 init done");
        }
        Err(e) => {
            error!("LAN8252 init error: {:?}", e);
        }
    }
    Timer::after(Duration::from_micros(1)).await;

    match lan9252
        .read_register_direct(Lan9252Registers::BYTE_TEST as u16, 4)
        .await
    {
        Ok(test) => {
            debug!("Byte test: {:#x}", test)
        }
        Err(e) => {
            error!("Read test error {:?}", e)
        }
    }
    Timer::after(Duration::from_micros(1)).await;

    match lan9252.read_register_indirect(WDOG_STATUS, 1).await {
        Ok(wdg) => {
            debug!("Watchdog: {:#x}", wdg[0])
        }
        Err(e) => {
            error!("Read watchdog error {:?}", e)
        }
    }
    Timer::after(Duration::from_micros(1)).await;

    match lan9252.read_register_indirect(AL_STATUS, 1).await {
        Ok(al) => {
            debug!("Status: {:#x}", al[0] & 0x0F)
        }
        Err(e) => {
            error!("Read status error {:?}", e)
        }
    }
    Timer::after(Duration::from_micros(1)).await;

    // before we start the loop write zeros to the OrbitaIn memory
    // to avoid old data being read
    debug!("Reinitialise the OrbitaIn memory!");
    let data = [0; 1 + 12 * N_AXIS];
    match lan9252.write_bytes(&data, Lan9252Memory::OrbitaIn as u16).await {
        Ok(_) => {}
        Err(e) => {
            error!("Write data error! {:?}", e)
        }
    }

    let mut poulpe_state = { SHARED_MEMORY.lock().await.get_poulpe_state() };
    let mut downsample_state_cnt: u32 = 0;
    loop {
        let t0 = Instant::now();
        // #[cfg(feature = "ignore_errors")]
        // let receive_commands = true;
        // #[cfg(not(feature = "ignore_errors"))]
        // let receive_commands = state == BoardStatus::Ok || state == BoardStatus::HighTemperatureState;
        if !poulpe_state.is_init() {
            let mut control_word : u16 = 0;
            let mut mode_of_operation: u8 = 0;
            let mut target_position = [0.0; N_AXIS];
            let mut target_velocity = [0.0; N_AXIS]; // not used
            let mut target_torque = [0.0; N_AXIS]; // not used
            let mut velocity_limits = [0.0; N_AXIS];
            let mut torque_limits = [0.0; N_AXIS];
            match lan9252
                .read_bytes(3 + 20 * N_AXIS, Lan9252Memory::OrbitaIn as u16)
                .await
            {
                Ok(data) => {
                    // control word is the first 2 bytes
                    control_word = u16::from_le_bytes(data[0..2].try_into().unwrap());
                    // info!("Control word: {:#x}", control_word);
                    // then the mode of operation
                    mode_of_operation = data[2];

                    // then the next N_AXIS f32 are the 
                    // target positions (3 - N_AXIS*4+3
                    // then the target velocity (N_AXIS*4+3 - 2*N_AXIS*4+3)
                    // then the velocity limits (2*N_AXIS*4+3 - 3*N_AXIS*4+3)
                    // then the target torque (3*N_AXIS*4+3 - 4*N_AXIS*4+3)
                    // then the torque limits (4*N_AXIS*4+3 - 5*N_AXIS*4+3)
                    // NOTE: 
                    //  - max length (for orbita3d) is 3 + 20*3 = 63Bytes
                    //  - If it would be more than 64 bytes, we would need to split the read in two parts 
                    for i in 0..N_AXIS {
                        target_position[i] = f32::from_le_bytes(data[3 + i * 4..3 + (i + 1) * 4].try_into().unwrap());
                        target_velocity[i] = f32::from_le_bytes(data[3 + N_AXIS * 4 + i * 4..3 + N_AXIS * 4 + (i + 1) * 4].try_into().unwrap());
                        velocity_limits[i] = f32::from_le_bytes(data[3 + 2 * N_AXIS * 4 + i * 4..3 + 2 * N_AXIS * 4 + (i + 1) * 4].try_into().unwrap());
                        target_torque[i] = f32::from_le_bytes(data[3 + 3 * N_AXIS * 4 + i * 4..3 + 3 * N_AXIS * 4 + (i + 1) * 4].try_into().unwrap());
                        torque_limits[i] = f32::from_le_bytes(data[3 + 4 * N_AXIS * 4 + i * 4..3 + 4 * N_AXIS * 4 + (i + 1) * 4].try_into().unwrap());
                    }
                }
                Err(e) => {
                    error!("Read data error! {:?}", e)
                }
            }
            // info!("Motors - Torque on: {:?}, Target: {:?}",  torque_on, target_position);
            if !poulpe_state.is_fault() && !poulpe_state.is_fault_reaction_state() {
                let shared_memory = SHARED_MEMORY.lock().await;
                shared_memory.set_control_word(control_word);
                // shared_memory.set_torque_on(torque_on);
                // shared_memory.set_control_mode(mode_of_operation);
                shared_memory.set_target_position(target_position);
                shared_memory.set_target_velocity(target_velocity);
                shared_memory.set_target_torque(target_torque);
                shared_memory.set_velocity_limit(velocity_limits);
                shared_memory.set_torque_flux_limit(torque_limits);
            }

            let shared_memory = SHARED_MEMORY.lock().await;
            let control_mode = { shared_memory.get_control_mode() };
            let torque_on = shared_memory.get_torque_on();
            let current_position = shared_memory.get_current_position();
            let current_velocity = shared_memory.get_current_velocity();
            let current_torque = shared_memory.get_current_torque();
            let axis_sensors = shared_memory.get_axis_sensor();

            // write back the read data to be read by the main ethercat loop
            // the data are written in the OrbitaOut memory
            // the data are written in the following format:
            // 1. statusword 2 bytes
            // 2. mode of operation 1 byte
            // then we send the actual data for each axis
            // 3. actual_postiion (N_AXIS * 4 bytes)
            // 4. actual_velocity (N_AXIS * 4 bytes)
            // 5. actual_torque (N_AXIS * 4 bytes)
            // 6. axis sensors (N_AXIS * 4 bytes)

            // NOTE: 
            //  - max length (for orbita3d) is 3 + 16*3 = 51Bytes
            //  - If it would be more than 64 bytes, we would need to split the write in two parts    
            let mut data: [u8; 3 + 16 * N_AXIS] = [0; 3 + 16 * N_AXIS];
            // statusword
            data[0..2].copy_from_slice(&poulpe_state.status_to_statusword());
            // mode of operation
            data[2] = control_mode[0].to_u8();
            // actual values
            for n in 0..N_AXIS {
                data[3 + n * 4..3 + (n + 1) * 4].copy_from_slice(&current_position[n].to_le_bytes());
                data[3 + N_AXIS * 4 + n * 4..3 + N_AXIS * 4 + (n + 1) * 4].copy_from_slice(&current_velocity[n].to_le_bytes());
                data[3 + 2 * N_AXIS * 4 + n * 4..3 + 2 * N_AXIS * 4 + (n + 1) * 4].copy_from_slice(&current_torque[n].to_le_bytes());
                data[3 + 3 * N_AXIS * 4 + n * 4..3 + 3 * N_AXIS * 4 + (n + 1) * 4].copy_from_slice(&axis_sensors[n].to_le_bytes());
            }
            match lan9252.write_bytes(&data, Lan9252Memory::OrbitaOut  as u16).await {
                Ok(_) => {}
                Err(e) => {
                    error!("Write data error! {:?}", e)
                }
            }
            Timer::after(Duration::from_micros(1)).await;
        }
        if downsample_state_cnt >= 100 {
            match lan9252.read_register_indirect(AL_STATUS, 1).await {
                Ok(al) => {
                    debug!("Status: {:#x}", al[0] & 0x0F)
                }
                Err(e) => {
                    error!("Read status error {:?}", e)
                }
            }
            Timer::after(Duration::from_micros(1)).await;

            // write the state to the OrbitaStatus memory
            // lower frequency than the rest of the data
            // at 0.1Hz more or less
            // the data are written in the OrbitaStatus memory
            // the data are written in the following format:
            // 1. error flags 4 bytes
            // 2. number of axis 1 byte
            // then we send the zeros for each axis
            // 3. axis zeros (N_AXIS * 4 bytes)
            // then we send the current temperatures
            // 4. board temperature 4 bytes
            // 5. motor temperature 4 bytes
            poulpe_state = { SHARED_MEMORY.lock().await.get_poulpe_state() };
            let board_temperture = { SHARED_MEMORY.lock().await.get_board_temperature() };
            let motor_temperture = { SHARED_MEMORY.lock().await.get_motor_temperature() };
            let mut data: [u8; 5+3*N_AXIS*4] = [N_AXIS as u8; 5+3*N_AXIS*4];
            data[0..4].copy_from_slice(&poulpe_state.error_flags_to_u8());
            data[4] = N_AXIS as u8;
            let axis_zeros = { SHARED_MEMORY.lock().await.get_hardware_zeros() };
            for i in 0..N_AXIS {
                data[5+i*4..9+i*4].copy_from_slice(&axis_zeros[i].to_le_bytes());
                data[5+N_AXIS*4+i*4..9+N_AXIS*4+i*4].copy_from_slice(&board_temperture[i].to_le_bytes());
                data[5+2*N_AXIS*4+i*4..9+2*N_AXIS*4+i*4].copy_from_slice(&motor_temperture[i].to_le_bytes());
            }

            match lan9252
                .write_bytes(&data, Lan9252Memory::OrbitaStatus as u16)
                .await
            {
                Ok(_) => {}
                Err(e) => {
                    error!("Write data error! {:?}", e)
                }
            }
            Timer::after(Duration::from_micros(1)).await;
            downsample_state_cnt = 0;
        }
        // Timer::after(Duration::from_millis(1)).await;
        debug!("Ethercat loop time: {:?}us", t0.elapsed().as_micros());
        downsample_state_cnt += 1;
    }
}
