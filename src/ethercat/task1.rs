use crate::{
    config::{self, LAN9252Config, N_AXIS},
    state_machine::{poulpe_state, CiA402Command},
    SHARED_MEMORY,
};
use core::{cell::RefCell, default, error, str, f32::consts::E, mem::take};
use defmt::{debug, error, info, trace, warn};
use embassy_embedded_hal::{flash, shared_bus::blocking::spi::SpiDeviceWithConfig};
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

        // // check if the lan9252 is in boot mode
        // match lan9252.read_register_indirect(AL_STATUS, 1).await {
        //     Ok(al) => {
        //         debug!("Status: {:#x}", al[0] & 0x0F);
        //         if al[0] != ESM_BOOT{
        //             warn!("LAN9252 is not in boot mode!");
        //             Timer::after(Duration::from_millis(1000)).await;
        //             continue;
        //         }
        //     }
        //     Err(e) => {
        //         error!("Read status error {:?}", e)
        //     }
        // }

        Timer::after(Duration::from_micros(1)).await;
        
        // check if the master if the master is connected 
        // by checking the watchdog status
        let master_connected: bool = match lan9252.read_register_indirect(0x805 as u16, 1).await {
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

        if master_connected {
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


            // let datagram_header: [u8; 6] = data[0..6].try_into().unwrap();
            // let mbox_header: [u8; 6] = data[6..12].try_into().unwrap();

            // parse the datagram 
            if data[5] == 3{
                info!("CoE protocol received: {:?}", data[5]);

                info!("Data: {:?}", data);
                let address = u16::from_le_bytes(data[9..11].try_into().unwrap());
                let sub_index = data[11];
                let data_rec = u32::from_le_bytes(data[12..16].try_into().unwrap());
                info!("Address: {:?}, subindex: {:?}, data: {:?}", address, sub_index, data_rec);

                if address == 0x100 && sub_index == 0x01{
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
                    
                // // construct the datagram to send to the master
                

                let mut data_write = [0x00u8; 128];
                // construct the datagram to acknowledge the data received
                data_write[0] = 10; // data length
                data_write[5] = 0x03; // set the CoE protocol
                let data_start_ind:usize = 6;
                data_write[data_start_ind..data_start_ind+2].copy_from_slice(&u16::to_le_bytes(12288));
                data_write[data_start_ind+2] = 0x60;
                data_write[data_start_ind+3..data_start_ind+5].copy_from_slice(&u16::to_le_bytes(address));
                data_write[data_start_ind+5] = sub_index;

                Timer::after(Duration::from_millis(1)).await;
                // heder is 6 bytes and then the 
                match lan9252.write_bytes_large(&data_write, Lan9252Memory::OrbitaInAck as u16).await {
                    Ok(_) => {}
                    Err(e) => {
                        error!("Write data error! {:?}", e)
                    }
                }

            }else if data[5] == 4 {
                info!("FoE protocol received: {:?}", data[5]);
                if data[6] == 0x2 {
                    info!("New file write request!");
                    let filename_size = u16::from_le_bytes(data[0..2].try_into().unwrap()) - 6;
                    let file_name_str = &data[12..12+filename_size as usize];
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
                }else if data[6] == 0x3 {
                    let data_chunk_lenght = u16::from_le_bytes(data[0..2].try_into().unwrap()) - 6;


                    if data_in_buf + (data_chunk_lenght as usize) > 4096 {
                        let rest_in_buf = 4096 - data_in_buf;
                        buf.as_mut()[data_in_buf..4096].copy_from_slice(data[12..(12 + rest_in_buf) as usize].as_ref());
                        writer.write(written_bytes, buf.as_ref()).unwrap();
                        written_bytes = written_bytes + 4096;
                        buf.as_mut().fill(255);
                        buf.as_mut()[0..(data_chunk_lenght as usize - rest_in_buf)].copy_from_slice(data[(12 + rest_in_buf) as usize..(12 + data_chunk_lenght) as usize].as_ref());
                        data_in_buf = data_chunk_lenght as usize - rest_in_buf;
                    }
                    else{
                        buf.as_mut()[data_in_buf..(data_in_buf + data_chunk_lenght as usize)].copy_from_slice(data[12..(12 + data_chunk_lenght) as usize].as_ref());
                        data_in_buf = data_in_buf + data_chunk_lenght as usize;
                    }
                    no_received_bytes = no_received_bytes + data_chunk_lenght as u32;
                    info!("Received data in bytes: {:?}B, written bytes: {:?}B", no_received_bytes, written_bytes);
                    info!("Data chunk length: {:?}", data_chunk_lenght);
                    if data_chunk_lenght < 116 {
                        if data_in_buf > 0 {
                            writer.write(written_bytes, &buf.as_ref()[0..4096]).unwrap();
                            written_bytes = written_bytes + data_in_buf as u32;
                        }
                        info!("End of file received!");
                    }
                }else{
                    error!("Unknown command! {:?}", data[6]);
                }

                let mut data_write = [0x00u8; 128];
                // construct the datagram to send to the master
                data_write[5] = 0x04; // set the FoE protocol

                data_write[0] = 0x01; // set the data size to 1
                data_write[6] = 0x04; // acknowledge the data received

                // heder is 6 bytes and then the 
                match lan9252.write_bytes_large(&data_write, Lan9252Memory::OrbitaInAck as u16).await {
                    Ok(_) => {info!("Data ack sent!");}
                    Err(e) => {
                        error!("Write data error! {:?}", e)
                    }
                }
            }else{
                error!("Wrong protocol! Protocol: {}", data[5]);
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
