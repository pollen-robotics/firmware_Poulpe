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
use super::coe::{coe_prepare_dataframe, coe_prepare_down_response, coe_prepare_up_response, CoEFrame};
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
                    updater.mark_booted().unwrap();
                },
            }
        }
        Err(e) => {
            error!("Bootloader state error {:?}", e);
        }
    }

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

    let mut no_received_bytes: u32 = 0;

    let mut buf = AlignedBuffer([0; 4096]);
    let mut data_in_buf: usize  = 0;
    let mut written_bytes: u32 = 0;
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

            // let datagram_header: [u8; 6] = data[0..6].try_into().unwrap();
            // let mbox_header: [u8; 6] = data[6..12].try_into().unwrap();

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

                        if index == 0x100 && sub_index == 0x01{
                            let data_rec = u32::from_le_bytes(coe_frame.data.try_into().unwrap_or([0u8;4]));
                            info!("Firmware upload end command!");
                            let firmware_upload_size = data_rec;
                            if firmware_upload_size == 0{
                                error!("Firmware upload size is 0!");
                            }else if no_received_bytes == 0 {
                                error!("No data received yet!");
                            }else if firmware_upload_size == no_received_bytes {
                                firmware_update_done = true;
                                info!("Received file size: {:?}, expected: {:?}", no_received_bytes, firmware_upload_size);
                                info!("Proceeding to firmware update!");
                            }else{
                                error!("Received file size: {:?}, expected: {:?}", no_received_bytes, firmware_upload_size);
                            }
                        }
                        coe_prepare_down_response(&mut data_write);
                    }else if coe_frame.is_upload(){
                        if index == 0x100 && sub_index == 0x01 {
                            coe_prepare_up_response(&mut data_write, &no_received_bytes.to_le_bytes(), true);    
                        }else if index == 0x200 && sub_index == 0x01 {
                            coe_prepare_up_response(&mut data_write, &config::GIT_HASH.as_bytes(), false);    
                        }else if index == 0x201 && sub_index == 0x01 {
                            coe_prepare_up_response(&mut data_write, &config::DXL_ID.to_le_bytes(), true);
                        }else if index == 0x202 {
                            if sub_index >= config::N_AXIS as u8 {
                                error!("Subindex out of range! {:?}", sub_index);
                            }else{
                                coe_prepare_up_response(&mut data_write, &config::HARDWARE_ZEROS[sub_index as usize].to_le_bytes(), true);
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
                            let filename_size = foe_frame.get_data_size();
                            let file_name_str = foe_frame.data;
                            match str::from_utf8(file_name_str.try_into().unwrap()){
                                Ok(string) => {
                                    info!("File name: {}", string);
                                },
                                Err(e) => {
                                    error!("Error parsing file name: {:?}", file_name_str);
                                }
                            }
                            no_received_bytes = 0; 
                            buf = AlignedBuffer([0; 4096]);
                            data_in_buf = 0;
                            written_bytes = 0;
                        },
                        FoERequestType::Data => {
                            let data_chunk_lenght = foe_frame.get_data_size();


                            if data_in_buf + (data_chunk_lenght as usize) > 4096 {
                                let rest_in_buf = 4096 - data_in_buf;
                                buf.as_mut()[data_in_buf..4096].copy_from_slice(foe_frame.data[0..rest_in_buf].as_ref());
                                writer.write(written_bytes, buf.as_ref()).unwrap();
                                written_bytes = written_bytes + 4096;
                                buf.as_mut().fill(255);
                                buf.as_mut()[0..(data_chunk_lenght as usize - rest_in_buf)].copy_from_slice(foe_frame.data[rest_in_buf..data_chunk_lenght as usize].as_ref());
                                data_in_buf = data_chunk_lenght as usize - rest_in_buf;
                            }
                            else{
                                buf.as_mut()[data_in_buf..(data_in_buf + data_chunk_lenght as usize)].copy_from_slice(foe_frame.data.as_ref());
                                data_in_buf = data_in_buf + data_chunk_lenght as usize;
                            }
                            no_received_bytes = no_received_bytes + data_chunk_lenght as u32;
                           info!("Data chunk length: {:?}", data_chunk_lenght);
                            if data_chunk_lenght < 116 {
                                if data_in_buf > 0 {
                                    writer.write(written_bytes, &buf.as_ref()[0..4096]).unwrap();
                                    written_bytes = written_bytes + data_in_buf as u32;
                                }
                                info!("End of file received! Received data in bytes: {:?}B, written bytes: {:?}B", no_received_bytes, written_bytes);
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
                        Ok(_) => {info!("Data ack sent!");}
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
    

            if firmware_update_done {
                firmware_update_done= false;
                updater.mark_updated().unwrap();
                Timer::after(Duration::from_millis(1000)).await;
                info!("Rebooting the device!");
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }
}
