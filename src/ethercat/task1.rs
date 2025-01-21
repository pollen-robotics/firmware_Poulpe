use crate::{
    config::{self, LAN9252Config, N_AXIS},
    state_machine::{poulpe_state, CiA402Command},
    SHARED_MEMORY,
};
use core::{cell::RefCell, default, error, str, f32::consts::E, mem::take};
use defmt::{debug, error, info, trace, warn};
use embassy_embedded_hal::{adapter, flash, shared_bus::blocking::spi::SpiDeviceWithConfig};
use embassy_stm32::spi;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
};

use crate::ethercat::lan9252::*;
use crate::state_machine::poulpe_state::PoulpeState;
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
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

use super::lan9252::{self, ESM_PREOP};
use super::coe::*;
use super::mailbox::*;
use super::foe::*;

// the addresses of the motors in the LAN9252 memory
pub enum Lan9252Memory {
    OrbitaIn = 0x1000, // default write address 0x1000
    OrbitaInAck = 0x1180, // default write address 0x1000
    OrbitaStatus = 0x1500,
    OrbitaOut = 0x1800,
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
pub async fn messsage_handler(ethconf: LAN9252Config, spi_config: spi::Config, flash: FLASH) {
    warn!("ETHERCAT TASK");

    error!("Firmware updater initalisaiton");

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
    let writer     = updater.prepare_update().unwrap();

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


    let mut file = FoEObject::empty(); 
    let mut buf = AlignedBuffer([0; 4096]);
    let mut firmware_update_done: bool = false;

    loop {

        // check if the lan9252 is in boot mode
        match lan9252.read_register_indirect(AL_STATUS, 1).await {
            Ok(al) => {
                debug!("Status: {:#x}", al[0] & 0x0F);
                if al[0] != ESM_PREOP{
                    warn!("LAN9252 is not in PREOP mode!");
                    Timer::after(Duration::from_millis(1000)).await;
                    continue;
                }
            }
            Err(e) => {
                error!("Read status error {:?}", e)
            }
        }

        Timer::after(Duration::from_micros(1)).await;
        
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
        // a a microsecond delay to avoid reading the same data
        Timer::after(Duration::from_micros(1)).await;

        // if mailbox data received, read the data and respond
        if mailbox_data_received {
            let mut data_copy = [0; 128];
            match lan9252
            .read_bytes_large(128, &mut data_copy, Lan9252Memory::OrbitaIn as u16)
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

            match mailbox_frame.get_mailbox_type() {
                MailboxType::CoE => {
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
                    
                    if coe_frame.is_download(){

                        match (index, sub_index){
                            (0x100, 1) => {
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
                    
                    }else if coe_frame.is_upload(){

                        match (index, sub_index){
                            (0x100, 1) => {
                                coe_prepare_up_response(&mut data_write, &file.no_received_bytes.to_le_bytes(), true);    
                            },
                            (0x200, 1) => {
                                coe_prepare_up_response(&mut data_write, &config::GIT_HASH.as_bytes(), false);    
                            },
                            (0x201, 1) => {
                                coe_prepare_up_response(&mut data_write, &config::DXL_ID.to_le_bytes(), true);
                            },
                            (0x202, _) => {
                                if sub_index >= config::N_AXIS as u8 {
                                    error!("Subindex out of range! {:?}", sub_index);
                                }else{
                                    coe_prepare_up_response(&mut data_write, &config::HARDWARE_ZEROS[sub_index as usize].to_le_bytes(), true);
                                }
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
                    match lan9252.write_bytes_large(&data_write, Lan9252Memory::OrbitaInAck as u16).await {
                        Ok(_) => {}
                        Err(e) => {
                            error!("Write data error! {:?}", e)
                        }
                    }
                },
                MailboxType::FoE => {
                    info!("FoE protocol received!");

                    let foe_frame = match FoEFrame::new(&data){
                        Ok(frame) => frame,
                        Err(_) => {
                            error!("Error parsing FoE frame!");
                            continue;
                        }
                    };

                    match foe_frame.get_request_type(){
                        FoERequestType::WriteRequest => {
                            info!("New file write request!");
                            file = match FoEObject::new(){//foe_frame.data){
                                Ok(file) => {
                                    info!("New file: {:?}", file.name);
                                    file
                                },
                                Err(_) => {
                                    error!("Error parsing file name!");
                                    continue;
                                }
                            };
                        },
                        FoERequestType::Data => {
                            let data_chunk_lenght = foe_frame.get_data_size();
                            debug!("Data chunk length: {:?}", data_chunk_lenght);

                            
                            let added_to_buffer = file.fill_buffer(&foe_frame.data[0..data_chunk_lenght as usize]) as u16;
                            if data_chunk_lenght > added_to_buffer {
                                // buffer is full
                                let rest_in_buf = data_chunk_lenght - added_to_buffer;
                                buf.as_mut().copy_from_slice(file.buffer.data.as_ref());
                                //match updater.write_firmware(file.no_written_bytes as usize, buf.as_ref()){
                                match writer.write(file.no_written_bytes, buf.as_ref()){
                                    Ok(_) => {
                                        file.no_written_bytes += file.buffer.size as u32;
                                        file.clear_buffer();
                                        file.fill_buffer(&foe_frame.data[(added_to_buffer as usize)..(added_to_buffer + rest_in_buf) as usize]);
                                    },
                                    Err(e) => {
                                        error!("Write data error! {:?}", e);
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

                    let mut data_write = [0x00u8; 128];
                    foe_prepare_acknowledge(data_write.as_mut());

                    // heder is 6 bytes and then the 
                    match lan9252.write_bytes_large(&data_write, Lan9252Memory::OrbitaInAck as u16).await {
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

            // a a microsecond delay to avoid reading the same data
            Timer::after(Duration::from_micros(1)).await;
    
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
        }
    }
}
