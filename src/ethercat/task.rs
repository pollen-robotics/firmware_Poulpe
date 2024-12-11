use crate::{
    config::{self, LAN9252Config, N_AXIS},
    state_machine::{poulpe_state, CiA402Command},
    SHARED_MEMORY,
};
use core::{cell::RefCell, default, mem::take};
use defmt::{debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::spi;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
};

use crate::ethercat::lan9252::{Lan9252, Lan9252Registers, AL_STATUS, WDOG_STATUS, READY};
use crate::state_machine::poulpe_state::PoulpeState;
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Duration, Instant, Timer};

use embassy_time::Ticker;

use crate::motor_control::foc::MotionMode;
use crate::state_machine::cia402_registers::CiA402ModeOfOperation;

// the addresses of the motors in the LAN9252 memory
pub enum Lan9252Memory {
    OrbitaIn = 0x1000, // default write address 0x1000
    OrbitaStatus = 0x1200,
    OrbitaOut = 0x1300,
}

// parse the watchdog counter from the bits 11-15 of the controlword
// these bits are manufacturer specific
pub fn parse_watchdog_counter(controlword: u16) -> u8 {
    (controlword >> 11) as u8
}

// set the watchdog counter to the bits 8, 14 and 15 of the statusword
// these bits are manufacturer specific
pub fn write_watchdog_counter(statusword: [u8; 2], counter: u8) -> [u8; 2] {
    let mut status_upper = statusword[1];
    status_upper |= counter & 0b1;
    status_upper |= (counter >> 1) << 6;
    return [statusword[0], status_upper];
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
    match lan9252
        .write_bytes(&data, Lan9252Memory::OrbitaIn as u16)
        .await
    {
        Ok(_) => {}
        Err(e) => {
            error!("Write data error! {:?}", e)
        }
    }

    // the vatiables to keep track of the watchdog counter
    let mut last_watchdog_counter = 0;
    let mut last_watchdog_counter_timestamp = Instant::now();
    // master connected flag
    let mut last_master_connected_timestamp = Instant::now();


    // create a ticker to run the loop at a fixed frequency of 1kHz
    let mut ticker = Ticker::every(Duration::from_micros(1000));

    // get the initial state of the poulpe state machine
    let mut poulpe_state = { SHARED_MEMORY.lock().await.get_poulpe_state() };
    let mut downsample_state_cnt: u32 = 0;
    let mut t0 = Instant::now();

    // variables to display the state of the ethercat communication
    let mut loop_cnt_display: u32 = 0;
    let mut t_display = Instant::now();

    // variables to be read from the OrbitaIn memory
    let mut control_word: u16 = 0;
    let mut mode_of_operation: CiA402ModeOfOperation = CiA402ModeOfOperation::NoMode;
    let mut target_position = [0.0; N_AXIS];
    let mut target_velocity = [0.0; N_AXIS]; // not used
    let mut target_torque = [0.0; N_AXIS]; // not used
    let mut velocity_limits = [0.0; N_AXIS];
    let mut torque_limits = [0.0; N_AXIS];
    let mut watchdog_counter = 0;

    loop {
        let t_loop = Instant::now();
        // get the satet of the poulpe state machine
        poulpe_state = { SHARED_MEMORY.lock().await.get_poulpe_state() };
            
        // check the ethercat state of the LAN9252
        let ethercat_state = match lan9252.read_register_indirect(AL_STATUS, 1).await {
            Ok(al) => {
                debug!("Status: {:#x}", al[0] & 0x0F);
                al[0] & 0x0F
            }
            Err(e) => {
                error!("Read status error {:?}", e);
                0
            }
        };
        // a a microsecond delay to avoid reading the same data
        Timer::after(Duration::from_micros(1)).await;

        // check if the master if the master is connected 
        // by checking the watchdog status
        let master_connected = match lan9252.read_register_indirect(WDOG_STATUS, 1).await {
            Ok(al) => {
                debug!("Watchdog Status: {:#x}", al[0]);
                al[0] != 0  // if the watchdog is not 0, the master is connected
            }
            Err(e) => {
                error!("Watchdog  status error {:?}", e);
                false
            }
        };
        // a a microsecond delay to avoid reading the same data
        Timer::after(Duration::from_micros(1)).await;

        if master_connected {
            last_master_connected_timestamp = Instant::now();
        }

        if !poulpe_state.is_init() && ethercat_state == READY {

            // if master is not connected we dont update the read data
            if master_connected {
                match lan9252
                    .read_bytes(3 + 20 * N_AXIS, Lan9252Memory::OrbitaIn as u16)
                    .await
                {
                    Ok(data) => {
                        // control word is the first 2 bytes
                        control_word = u16::from_le_bytes(data[0..2].try_into().unwrap());
                        // info!("Control word: {:#x}", control_word);
                        // then the mode of operation
                        mode_of_operation = CiA402ModeOfOperation::from_u8(data[2]);

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
                            target_position[i] = f32::from_le_bytes(
                                data[3 + i * 4..3 + (i + 1) * 4].try_into().unwrap(),
                            );
                            target_velocity[i] = f32::from_le_bytes(
                                data[3 + N_AXIS * 4 + i * 4..3 + N_AXIS * 4 + (i + 1) * 4]
                                    .try_into()
                                    .unwrap(),
                            );
                            velocity_limits[i] = f32::from_le_bytes(
                                data[3 + 2 * N_AXIS * 4 + i * 4..3 + 2 * N_AXIS * 4 + (i + 1) * 4]
                                    .try_into()
                                    .unwrap(),
                            );
                            target_torque[i] = f32::from_le_bytes(
                                data[3 + 3 * N_AXIS * 4 + i * 4..3 + 3 * N_AXIS * 4 + (i + 1) * 4]
                                    .try_into()
                                    .unwrap(),
                            );
                            torque_limits[i] = f32::from_le_bytes(
                                data[3 + 4 * N_AXIS * 4 + i * 4..3 + 4 * N_AXIS * 4 + (i + 1) * 4]
                                    .try_into()
                                    .unwrap(),
                            );
                        }
                    }
                    Err(e) => {
                        error!("Read data error! {:?}", e)
                    }
                }
            }
            // a a microsecond delay to avoid reading the same data
            Timer::after(Duration::from_micros(1)).await;

            // watchdog counter is in the controlword
            watchdog_counter = parse_watchdog_counter(control_word);
            //info!("Watchdog counter: {}, {}", watchdog_counter, master_connected);
            if last_watchdog_counter != watchdog_counter {
                // update the last watchdog counter and the timestamp
                last_watchdog_counter_timestamp = Instant::now();
                last_watchdog_counter = watchdog_counter;
            }

            // send data only if the master is connected
            if master_connected {
                let shared_memory = SHARED_MEMORY.lock().await;
                let control_mode = { shared_memory.get_control_mode_display() };
                let torque_on = shared_memory.get_torque_on();
                let current_position = shared_memory.get_current_position();
                let current_velocity = shared_memory.get_current_velocity();
                let current_torque = shared_memory.get_current_torque();
                let axis_sensors = shared_memory.get_axis_sensor();
                poulpe_state = shared_memory.get_poulpe_state();

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
                let mut statusword = poulpe_state.status_to_statusword_byte_array();
                // write the watchdog counter to the statusword (copied from the controlword)
                statusword = write_watchdog_counter(statusword, watchdog_counter);
                data[0..2].copy_from_slice(&statusword);
                // mode of operation
                data[2] = CiA402ModeOfOperation::from_tmc4671_mode(control_mode).to_u8();
                // actual values
                for n in 0..N_AXIS {
                    data[3 + n * 4..3 + (n + 1) * 4]
                        .copy_from_slice(&current_position[n].to_le_bytes());
                    data[3 + N_AXIS * 4 + n * 4..3 + N_AXIS * 4 + (n + 1) * 4]
                        .copy_from_slice(&current_velocity[n].to_le_bytes());
                    data[3 + 2 * N_AXIS * 4 + n * 4..3 + 2 * N_AXIS * 4 + (n + 1) * 4]
                        .copy_from_slice(&current_torque[n].to_le_bytes());
                    data[3 + 3 * N_AXIS * 4 + n * 4..3 + 3 * N_AXIS * 4 + (n + 1) * 4]
                        .copy_from_slice(&axis_sensors[n].to_le_bytes());
                }
                match lan9252
                    .write_bytes(&data, Lan9252Memory::OrbitaOut as u16)
                    .await
                {
                    Ok(_) => {}
                    Err(e) => {
                        error!("Write data error! {:?}", e)
                    }
                }
                // a a microsecond delay to avoid reading the same data
                Timer::after(Duration::from_micros(1)).await;
            }
        }


        // allow changing the target position ans weel as the velocity and torque limits
        // only if not in fault state, not in fault reaction state and not in quick stop state
        if poulpe_state.is_preoperation_state() || poulpe_state.is_operation_enabled() {
            if last_watchdog_counter_timestamp.elapsed()
                    > Duration::from_millis(config::MAX_WATCHDOG_DOWN_TIME_MS) ||
                last_master_connected_timestamp.elapsed()
                    > Duration::from_millis(config::MAX_WATCHDOG_DOWN_TIME_MS)
            {
                // if the watchdog counter is not updated for more than 100ms
                // we go to the quick stop reaction state
                control_word = CiA402Command::QuickStop.to_u16();
                debug!("Master not connected , our watchdog");
            }
            
            let shared_memory = SHARED_MEMORY.lock().await;
            shared_memory.set_control_word(control_word);
            // motion mode not used for now!!!
            #[cfg(feature = "allow_mode_change")]
            {
                shared_memory.set_control_mode(CiA402ModeOfOperation::to_tmc4671_mode(
                    &mode_of_operation,
                ));
                // dont update the target velocity and torque if the
                // control mode cannot be changed
                // saving some time
                shared_memory.set_target_velocity(target_velocity);
                shared_memory.set_target_torque(target_torque);
            }
            shared_memory.set_target_position(target_position);
            shared_memory.set_velocity_limit(velocity_limits);
            shared_memory.set_torque_flux_limit(torque_limits);
        } else {
            // if we are not in the preoperation state or in the operation enabled state
            last_watchdog_counter_timestamp = Instant::now();
            last_watchdog_counter = 255; // set to a value that is not possible
        }

        // mailbox PDO write at around 10Hz
        if downsample_state_cnt >= 100 && ethercat_state == READY && master_connected {
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
            let board_temperture = { SHARED_MEMORY.lock().await.get_board_temperature() };
            let motor_temperture = { SHARED_MEMORY.lock().await.get_motor_temperature() };
            let mut data = [0 as u8; 2 + 2 * N_AXIS + 1 + 3 * N_AXIS * 4];
            let error_flags = poulpe_state.error_flags_to_byte_array();
            let mut nb_bytes = error_flags.len(); // number of bytes for the error flags
            data[0..nb_bytes].copy_from_slice(&error_flags);
            data[nb_bytes] = N_AXIS as u8;
            let axis_zeros = { SHARED_MEMORY.lock().await.get_hardware_zeros() };
            nb_bytes += 1;
            for i in 0..N_AXIS {
                data[nb_bytes + i * 4..nb_bytes + 4 + i * 4]
                    .copy_from_slice(&axis_zeros[i].to_le_bytes());
                data[nb_bytes + N_AXIS * 4 + i * 4..nb_bytes + 4 + N_AXIS * 4 + i * 4]
                    .copy_from_slice(&board_temperture[i].to_le_bytes());
                data[nb_bytes + 2 * N_AXIS * 4 + i * 4..nb_bytes + 4 + 2 * N_AXIS * 4 + i * 4]
                    .copy_from_slice(&motor_temperture[i].to_le_bytes());
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
            downsample_state_cnt = 0;
            // a a microsecond delay to avoid reading the same data
            Timer::after(Duration::from_micros(1)).await;
        }

        // display the state of the ethercat communication to the console
        if t_display.elapsed() > Duration::from_millis(2000) {
            // display the state of the ethercat communicatio
            if ethercat_state == READY {
                if master_connected {
                    info!("ETHERCAT State: READY\t Master: connected\tFrequency: {}Hz", loop_cnt_display*1000/((t_display.elapsed().as_millis()) as u32));
                } else {
                    warn!("ETHERCAT State: READY\tMaster: not connected");
                }
            } else {
                    warn!("ETHERCAT State: NOT_READY: {:#x} (ready {:#x})", ethercat_state, READY);
            }
            t_display = Instant::now(); // reset the display timer
            loop_cnt_display = 0; // reset the counter
        }

        
        #[cfg(feature = "debug_execution_time")]
        {
            info!("Ethercat loop elapsed time: {}us \t time between loops: {}us",t_loop.elapsed().as_micros(), t0.elapsed().as_micros());
            t0 = Instant::now();
        }
        downsample_state_cnt += 1; // increment the downsample counter for state update (PDO mailbox)
        loop_cnt_display += 1; // increment the counter for display
        ticker.next().await;
    }
}
