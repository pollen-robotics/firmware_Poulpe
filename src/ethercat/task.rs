use crate::{
    config::{self, LAN9252Config, N_AXIS},
    state_machine::{poulpe_state, CiA402Command},
    SHARED_MEMORY,
};
use core::{cell::RefCell, default,error,  ops::DerefMut};
use defmt::{debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::spi;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
};
use embedded_hal::watchdog;

use crate::ethercat::lan9252::*;
use crate::state_machine::poulpe_state::PoulpeState;
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, raw::ThreadModeRawMutex,  Mutex};
use embassy_sync::mutex::MutexGuard;
use embassy_time::{Duration, Instant, Timer};

use embassy_time::Ticker;

use crate::motor_control::foc::MotionMode;
use crate::state_machine::cia402_registers::CiA402ModeOfOperation;


// firmware update imports
use embassy_stm32::peripherals::FLASH;
use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareUpdater, FirmwareUpdaterConfig};
use embassy_stm32::flash::{Flash, WRITE_SIZE};
use embedded_storage::nor_flash::NorFlash;
use embassy_boot_stm32::State;

// mailbox treating imports
use super::coe::*;
use super::mailbox::*;
use super::foe::*;


// the addresses of the motors in the LAN9252 memory
pub enum Lan9252Memory {
    MBoxInput = 0x1000, // corresponding to MBoxOut - mailbox input data
    MBoxOutput = 0x1180, // corresponging to MBoxIn - mailbox output data
    OrbitaIn = 0x1300, // OrbitaIn PDO
    OrbitaOut = 0x1400, // OrbitaOut PDO
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

pub fn prepare_pdo_state(
    shared_memory: &MutexGuard<ThreadModeRawMutex, crate::SharedMemory<{config::N_AXIS}>>,
) -> [u8; 2 + 2 * N_AXIS + 1 + 3 * N_AXIS * 4]
{

    let board_temperture = shared_memory.get_board_temperature();
    let motor_temperture = shared_memory.get_motor_temperature();
    let axis_zeros = shared_memory.get_hardware_zeros();
    let poulpe_state = shared_memory.get_poulpe_state();

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
    let mut data = [0 as u8; 2 + 2 * N_AXIS + 1 + 3*N_AXIS*4];
    let error_flags = poulpe_state.error_flags_to_byte_array();
    let mut nb_bytes = error_flags.len(); // number of bytes for the error flags
    data[0..nb_bytes].copy_from_slice(&error_flags);
    data[nb_bytes] = N_AXIS as u8;
    nb_bytes += 1;
    for i in 0..N_AXIS {
        data[nb_bytes + i * 4..nb_bytes + 4 + i * 4]
            .copy_from_slice(&axis_zeros[i].to_le_bytes());
        data[nb_bytes + N_AXIS * 4 + i * 4..nb_bytes + 4 + N_AXIS * 4 + i * 4]
            .copy_from_slice(&board_temperture[i].to_le_bytes());
        data[nb_bytes + 2 * N_AXIS * 4 + i * 4..nb_bytes + 4 + 2 * N_AXIS * 4 + i * 4]
            .copy_from_slice(&motor_temperture[i].to_le_bytes());
    }
    return data;
    
}


pub fn prepare_pdo_outputs(
    watchdog_counter: u8,
    shared_memory: &MutexGuard<ThreadModeRawMutex, crate::SharedMemory<{config::N_AXIS}>>,
)-> [u8; 3 + 16 * N_AXIS]
{
    let control_mode = { shared_memory.get_control_mode_display() };
    let torque_on = shared_memory.get_torque_on();
    let current_position = shared_memory.get_current_position();
    let current_velocity = shared_memory.get_current_velocity();
    let current_torque = shared_memory.get_current_torque();
    let axis_sensors = shared_memory.get_axis_sensor();
    let poulpe_state = shared_memory.get_poulpe_state();
    
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
    return data;
}


pub fn parse_pdo_inputs(
    data: &[u8], 
    control_word: &mut u16, 
    mode_of_operation: &mut CiA402ModeOfOperation, 
    target_position: &mut [f32; N_AXIS], 
    target_velocity: &mut [f32; N_AXIS], 
    velocity_limits: &mut [f32; N_AXIS], 
    target_torque: &mut [f32; N_AXIS], 
    torque_limits: &mut [f32; N_AXIS]) {

    // control word is the first 2 bytes
    *control_word = u16::from_le_bytes(data[0..2].try_into().unwrap());
    // info!("Control word: {:#x}", control_word);
    // then the mode of operation
    *mode_of_operation = CiA402ModeOfOperation::from_u8(data[2]);

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


#[embassy_executor::task]
pub async fn messsage_handler(ethconf: LAN9252Config, spi_config: spi::Config, flash: FLASH) {
    warn!("ETHERCAT TASK");

    // initialise the variables for the firmware update
    let flash = Flash::new_blocking(flash);
    let flash = Mutex::new(RefCell::new(flash));
    // a bit of delay to avoid the firmware updater to start before Flash is ready
    Timer::after(Duration::from_micros(100)).await; 
    // // Firmware updater
    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash);
    let mut magic = AlignedBuffer([0; WRITE_SIZE]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut magic.0);
    match updater.get_state(){
        Ok(state) => {
            match state {
                State::Boot => {
                    info!("Bootloader state: Boot");
                },
                State::Swap => {
                    info!("Bootloader state: Swap");
                    match updater.mark_booted(){
                        Ok(_) => {
                            info!("Marked booted!");
                        },
                        Err(e) => {
                            error!("Mark booted error: {:?}", e);
                        }
                    }
                },
            }
        }
        Err(e) => {
            error!("Bootloader state error {:?}", e);
        }
    }
    
    // TODO
    // this unwrap is problematic as it can end up in a panic
    // but this unwrap is unlikely to fail if the previous lines pass. 
    // I left it this way because I did not know what to do if the writer fails. 
    let writer = updater.prepare_update().unwrap();


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

    // create a ticker to run the loop at a fixed frequency of 1kHz
    let mut ticker = Ticker::every(Duration::from_micros(1000));

    // the vatiables to keep track of the watchdog counter
    let mut last_watchdog_counter = 0;
    let mut last_watchdog_counter_timestamp = Instant::now();
    // master connected flag
    let mut last_pdos_enabled_timestamp = Instant::now();
    
    // get the initial state of the poulpe state machine
    let mut poulpe_state = { SHARED_MEMORY.lock().await.get_poulpe_state() };
    let mut downsample_state_pdos_cnt: u32 = 0;
    let mut pdos_enabled = false;

    // variables to display the state of the ethercat communication
    let mut loop_cnt_display: u32 = 0;
    let mut t_display = Instant::now();
    let mut t0 = Instant::now();

    // variables to be read from the OrbitaIn memory
    let mut control_word: u16 = 0;
    let mut mode_of_operation: CiA402ModeOfOperation = CiA402ModeOfOperation::NoMode;
    let mut target_position = [0.0; N_AXIS];
    let mut target_velocity = [0.0; N_AXIS]; // not used
    let mut target_torque = [0.0; N_AXIS]; // not used
    let mut velocity_limits = [0.0; N_AXIS];
    let mut torque_limits = [0.0; N_AXIS];
    let mut watchdog_counter = 0;


    // firmware update variables
    let mut file = FoEObject::empty(); 
    let mut buf = AlignedBuffer([0; 4096]);
    let mut firmware_update_done: bool = false;


    // ethercat state varaibles
    let mut ethercat_state = EthercatState::Unknown;
    let mut downsample_ethercat_state_cnt: u32 = 0;
    loop {
        let t_loop = Instant::now();
        // get the satet of the poulpe state machine
        poulpe_state = { SHARED_MEMORY.lock().await.get_poulpe_state() };
            

        // check the ethercat state of the LAN9252
        ethercat_state = match lan9252.read_register_indirect(AL_STATUS, 1).await {
            Ok(al) => {
                debug!("Status: {:#x}", al[0] & 0x0F);
                EthercatState::from_u8(al[0] & 0x0F)
            }
            Err(e) => {
                error!("Read status error {:?}", e);
                EthercatState::Unknown
            }
        };
        // a a microsecond delay to avoid reading the same data
        // Timer::after(Duration::from_micros(1)).await;
        
        // if !poulpe_state.is_init() && ethercat_state == READY {
        match ethercat_state {
            EthercatState::OP => {
                //
                // PDOs are only updated if the ethercat state is READY
                //
                // if !poulpe_state.is_init() { }

                // check if the master if the master is connected 
                // and reading/sending pdos
                // by checking the watchdog status
                pdos_enabled = match lan9252.read_register_indirect(WDOG_STATUS, 1).await {
                    Ok(al) => {
                        debug!("Watchdog Status: {:#x}", al[0]);
                        let enabled = al[0] != 0;  // if the watchdog is not 0, the master is connected
                        if enabled {
                            last_pdos_enabled_timestamp = Instant::now();
                        }
                        enabled
                    }
                    Err(e) => {
                        error!("Watchdog  status error {:?}", e);
                        false
                    }
                };


                // if master is not connected we dont update the read data
                if pdos_enabled {
                    match lan9252
                        .read_bytes(3 + 20 * N_AXIS, Lan9252Memory::OrbitaIn as u16)
                        .await
                    {
                        Ok(data) => {
                            parse_pdo_inputs(data, 
                                &mut control_word, 
                                &mut mode_of_operation, 
                                &mut target_position, 
                                &mut target_velocity, 
                                &mut velocity_limits, 
                                &mut target_torque, 
                                &mut torque_limits);
                        }
                        Err(e) => {
                            error!("Read data error! {:?}", e)
                        }
                    }
                }

                // watchdog counter is in the controlword
                watchdog_counter = parse_watchdog_counter(control_word);
                //info!("Watchdog counter: {}, {}", watchdog_counter, pdos_enabled);
                if last_watchdog_counter != watchdog_counter {
                    // update the last watchdog counter and the timestamp
                    last_watchdog_counter_timestamp = Instant::now();
                    last_watchdog_counter = watchdog_counter;
                }

                // send data only if the master is connected
                if pdos_enabled {
                    // write back the read data to be read by the main ethercat loop
                    let data_outs = prepare_pdo_outputs(
                        watchdog_counter,
                        &SHARED_MEMORY.lock().await
                    );  

                    let mut data = [0; 2 + 2 * N_AXIS + 1 + 3*N_AXIS*4 + 3 + 16 * N_AXIS];
                    let mut data_len = data_outs.len();
                    data[0..data_len].copy_from_slice(&data_outs);

                    // update the state only every 100ms
                    // write the state to the OrbitaStatus memory
                    let data_state = prepare_pdo_state(
                        &SHARED_MEMORY.lock().await
                    );

                    data[data_len..].copy_from_slice(&data_state);

                    // write the data to the OrbitaOut memory
                    match lan9252
                        .write_bytes_large(&data, Lan9252Memory::OrbitaOut as u16)
                        .await
                    {
                        Ok(_) => {}
                        Err(e) => {
                            error!("Write data error! {:?}", e)
                        }
                    }
                }

                // write the PDO values to the shared memory
                // allow changing the target position as well as the velocity and torque limits
                // only if not in fault state, not in fault reaction state and not in quick stop state
                if poulpe_state.is_preoperation_state() || poulpe_state.is_operation_enabled() {
                    if last_watchdog_counter_timestamp.elapsed()
                            > Duration::from_millis(config::MAX_WATCHDOG_DOWN_TIME_MS) ||
                        last_pdos_enabled_timestamp.elapsed()
                            > Duration::from_millis(config::MAX_WATCHDOG_DOWN_TIME_MS)
                    {
                        // if the watchdog counter is not updated for more than 100ms
                        // we go to the quick stop reaction state
                        control_word = CiA402Command::QuickStop.to_u16();
                        debug!("Master not connected, watchdog too old");
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
            },
            EthercatState::PREOP => {
                // 
                // SDO communication is only possible in the preoperation state
                //  - it is done through mailbox communication
                // 

                // check if the master sent a new mailbox data
                let mailbox_data_received: bool = match lan9252.read_register_indirect(0x805 as u16, 1).await {
                    Ok(al) => {
                        // debug!("Watchdog Status: {:#x}", al[0]&0b1000);
                        al[0] & 0b1000 != 0  // if the watchdog is not 0, the master is connected
                    }
                    Err(e) => {
                        error!("Watchdog  status error {:?}", e);
                        false
                    }
                };

                // if mailbox data is received parse it
                if mailbox_data_received {
                    // read the received mailbox data
                    let mut data_copy = [0; 128];
                    match lan9252
                    .read_bytes_large(128, &mut data_copy, Lan9252Memory::MBoxInput as u16)
                    .await
                    {
                        Ok(_) => {
                            debug!("OrbitaIn: {:?}", data_copy);
                        }
                        Err(e) => {
                            error!("Read data error! {:?}", e);
                        }
                    };
        
                    let data = data_copy.clone();
                    let mailbox_frame = match MailboxFrame::new(&data){
                        Ok(frame) => frame,
                        Err(_) => {
                            error!("Error parsing mailbox frame!");
                            continue;
                        }
                    };
        
                    // check which kind of mailbox protocol is received
                    match mailbox_frame.get_mailbox_type() {
                        MailboxType::CoE => {
                            // 
                            // if Can over ethercat protocol (CoE) 
                            // this is an SDO communication request either read or write
                            //
                            info!("CoE protocol received!");
                                       
                            let coe_frame = match CoEFrame::new(&data){
                                Ok(frame) => frame,
                                Err(_) => {
                                    error!("Error parsing CoE frame!");
                                    continue;
                                }
                            };
                            info!("CoE header received: {:?}", coe_frame);
        
                            if !coe_frame.is_request() {
                                warn!("Not SDO request, skipping! Type: {}", coe_frame.header.request_type);
                                continue;
                            }
        
                            let (index,sub_index) = coe_frame.get_sdo_entry();
                            debug!("Mailbox data received (CoE): {:?}", &data);
                            let mut data_write = coe_prepare_dataframe(index, sub_index, true);                
                            
                            if coe_frame.is_sdo_write(){
                                //  
                                // SDO write commands are handled here
                                //
        
                                match (index, sub_index){
                                    (0x100, 1) => {
                                        // command signaling the end of the firmware upload 
                                        // after this command has been received (and if the numebr of bytes received is correct)
                                        // the mcu will restart and update the firmware
                                        let data_rec = u32::from_le_bytes(coe_frame.data.try_into().unwrap_or([0u8;4]));
                                        info!("Firmware upload end received command!");
                                        let firmware_upload_size = data_rec;
                                        if firmware_upload_size == 0{
                                            error!("Firmware upload size is 0!");
                                        }else if file.no_received_bytes == 0 {
                                            error!("No data received yet!");
                                        }else if firmware_upload_size == file.no_received_bytes {
                                            firmware_update_done = true;
                                            info!("Received file size: {:?}, expected: {:?}", file.no_received_bytes, firmware_upload_size);
                                            info!("Proceeding to firmware update!");
                                        }else{
                                            error!("Received file size: {:?}, expected: {:?}", file.no_received_bytes, firmware_upload_size);
                                        }
                                    },
                                    _ => {
                                        error!("Unknown index and subindex! {:?}", (index, sub_index));
                                    }
                                }
                                coe_prepare_down_response(&mut data_write);
                            
                            }else if coe_frame.is_sdo_read(){
                                //  
                                // SDO read commands are handled here
                                //
                                match (index, sub_index){
                                    (0x100, 1) => {
                                        // FoE protocol, read the number of received bytes in the file
                                        // number of firmware bytes received
                                        coe_prepare_up_response(
                                            &mut data_write, 
                                            &file.no_received_bytes.to_le_bytes(), 
                                            true
                                        );    
                                    },
                                    (0x200, 1) => {
                                        // display the current git hash of the firmware in the device
                                        coe_prepare_up_response(
                                            &mut data_write, 
                                            &config::GIT_HASH.as_bytes(), 
                                            false
                                        );    
                                    },
                                    (0x201, 1) => {
                                        // display the current Dynamixel ID]
                                        let id = {SHARED_MEMORY.lock().await.get_board_id()};
                                        coe_prepare_up_response(
                                            &mut data_write, 
                                            &id.to_le_bytes(), 
                                            true
                                        );
                                    },
                                    (0x202, _) => {
                                        // display the hardware zeros
                                        let hardware_zeros = {SHARED_MEMORY.lock().await.get_hardware_zeros()};
                                        if sub_index >= config::N_AXIS as u8 {
                                            error!("Subindex out of range! {:?}", sub_index);
                                        }else{
                                            coe_prepare_up_response(
                                                &mut data_write, 
                                                &hardware_zeros[sub_index as usize].to_le_bytes(), 
                                                true
                                            );
                                        }
                                    },
                                    (0x203, 1) => {
                                        // display the axis number of the slave
                                        coe_prepare_up_response(
                                            &mut data_write, 
                                            &config::N_AXIS.to_le_bytes(), 
                                            true
                                        );
                                    },
                                    _ => {
                                        error!("Unknown index and subindex! {:?}", (index, sub_index));
                                    }
                                }
                            
                            }else{
                                error!("Unknown command! {:?}", coe_frame.header.request_type);
                            }
                            
                            // construct the datagram to acknowledge the data received
                            debug!("Mailbox data response (CoE): {:?}", &data_write);
                            Timer::after(Duration::from_millis(1)).await;
                            // heder is 6 bytes and then the 
                            match lan9252.write_bytes_large(&data_write, Lan9252Memory::MBoxOutput as u16).await {
                                Ok(_) => {}
                                Err(e) => {
                                    error!("Write data error! {:?}", e)
                                }
                            }

                            // if the firmare has been downloaded, and the firmware update signal is received
                            // mark the firmware as updated and reboot the device
                            if firmware_update_done {
                                firmware_update_done= false;
                                match updater.mark_updated(){
                                    Ok(_) => {
                                        info!("Firmware ready for update!");
                                    },
                                    Err(e) => {
                                        error!("Firmware update error: {:?}", e);
                                    }
                                }
                                Timer::after(Duration::from_millis(1000)).await;
                                info!("Rebooting the device!");
                                cortex_m::peripheral::SCB::sys_reset();
                            }
                        },
                        MailboxType::FoE => {
                            // 
                            // if File over ethercat protocol (FoE) 
                            // this is the firmware update request
                            //
                            info!("FoE protocol received!");
        
                            let foe_frame = match FoEFrame::new(&data){
                                Ok(frame) => frame,
                                Err(_) => {
                                    error!("Error parsing FoE frame!");
                                    continue;
                                }
                            };

                            // by default we send an acknowledge
                            let mut data_write = [0x00u8; 128];
                            foe_prepare_acknowledge(data_write.as_mut());
        
                            match foe_frame.get_request_type(){
                                FoERequestType::WriteRequest => {
                                    info!("New file write request!");
                                    if file.no_written_bytes == 0 {
                                        file = match FoEObject::new(){
                                            Ok(file) => {
                                                info!("New file: {:?}", file.name);
                                                file
                                            },
                                            Err(_) => {
                                                error!("Error parsing file name!");
                                                continue;
                                            }
                                        };
                                    }else{
                                        error!("File already written, rejecting new file!");
                                        // send an error message
                                        foe_prepare_error(data_write.as_mut());
                                    }
                                },
                                FoERequestType::Data => {
                                    let data_chunk_lenght = foe_frame.get_data_size();
                                    debug!("Data chunk length: {:?}", data_chunk_lenght);
        
                                    let added_to_buffer = file.fill_buffer(&foe_frame.data[0..data_chunk_lenght as usize]) as u16;
                                    if data_chunk_lenght > added_to_buffer {
                                        // buffer is full
                                        let rest_in_buf = data_chunk_lenght - added_to_buffer;
                                        buf.as_mut().copy_from_slice(file.buffer.data.as_ref());
                                        match writer.write(file.no_written_bytes, buf.as_ref()){
                                            Ok(_) => {
                                                file.no_written_bytes += file.buffer.size as u32;
                                                file.clear_buffer();
                                                file.fill_buffer(&foe_frame.data[(added_to_buffer as usize)..(added_to_buffer + rest_in_buf) as usize]);
                                            },
                                            Err(e) => {
                                                error!("Write data error! {:?}", e);
                                                // send an error message
                                                foe_prepare_error(data_write.as_mut());
                                            }
                                        }
                                        
                                    }      
        
                                    // if the data chunk is less than 116 bytes, it is the end of the file
                                    if foe_frame.is_full_packet() {
                                        if file.buffer.data_len > 0 {
                                            buf.as_mut().fill(255);
                                            buf.as_mut()[0..file.buffer.data_len as usize].copy_from_slice(&file.buffer.data[0..file.buffer.data_len as usize]);
                                            //match updater.write_firmware(file.no_written_bytes  as usize, buf.as_ref()){
                                            match writer.write(file.no_written_bytes, &buf.as_ref()){
                                                Ok(_) => {
                                                    file.no_written_bytes += file.buffer.data_len as u32;
                                                    info!("End of file {} received! Received data in bytes: {:?}B, written bytes: {:?}B", file.name, file.no_received_bytes, file.no_written_bytes);
                                                },
                                                Err(e) => {
                                                    error!("End of fileWrite data error! {:?}", e);
                                                    // send an error message
                                                    foe_prepare_error(data_write.as_mut());
                                                }
                                            }
                                            
                                        }
                                    }
                                },
                                _ =>{
                                    warn!("Unsupported FoE request type! {:?}", foe_frame.get_request_type());
                                    continue;
                                }
                            }
                
                            // heder is 6 bytes and then the 
                            match lan9252.write_bytes_large(&data_write, Lan9252Memory::MBoxOutput as u16).await {
                                Ok(_) => {debug!("Data ack sent!");}
                                Err(e) => {
                                    error!("Write data error! {:?}", e)
                                }
                            }
                        },
                        _ => {
                            error!("Unknown protocol! {:?}", data[5]);
                        }
                    }
                }
            },
            _ => {
                if poulpe_state.is_preoperation_state() || poulpe_state.is_operation_enabled() {
                    let control_word = CiA402Command::QuickStop.to_u16();
                    let shared_memory = SHARED_MEMORY.lock().await;
                    shared_memory.set_control_word(control_word);
                }
            }

        }


        // display the state of the ethercat communication to the console
        if t_display.elapsed() > Duration::from_millis(2000) {
            match ethercat_state{
                EthercatState::OP => {
                    if pdos_enabled {
                        info!("ETHERCAT State: OP\t PDOs: connected\tFrequency: {}Hz", loop_cnt_display*1000/((t_display.elapsed().as_millis()) as u32));
                    } else {
                        warn!("ETHERCAT State: OP\t PDOs: not connected");
                    }
                },
                EthercatState::PREOP  => {
                    warn!("ETHERCAT State: PREOP, only FoE and CoE available!");
                },
                _ => {
                    warn!("ETHERCAT State: NOT_READY: {:#x} ", ethercat_state);
                }
            }
            t_display = Instant::now(); // reset the display timer
            loop_cnt_display = 0; // reset the counter
        }

        
        #[cfg(feature = "debug_execution_time")]
        {
            error!("Ethercat loop elapsed time: {}us \t time between loops: {}us",t_loop.elapsed().as_micros(), t0.elapsed().as_micros());
            t0 = Instant::now();
        }
        downsample_state_pdos_cnt += 1; // increment the downsample counter for state update (PDO mailbox)
        loop_cnt_display += 1; // increment the counter for display


        // TODO do not remove this delay
        // it is necessary for thread sincronization
        // I dont understand why though
        Timer::after(Duration::from_micros(1)).await;
        // 1ms frequency
        ticker.next().await;
    }
}
