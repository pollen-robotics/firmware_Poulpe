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

use crate::ethercat::lan9252::{Lan9252, Lan9252Registers, AL_STATUS, WDOG_STATUS, READY};
use crate::state_machine::poulpe_state::PoulpeState;
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Duration, Instant, Timer};

use embassy_time::Ticker;

use crate::motor_control::foc::MotionMode;
use crate::state_machine::cia402_registers::CiA402ModeOfOperation;

//import flash
use embassy_stm32::flash::Flash;


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

    let mut no_received_bytes: u32 = 0;
    loop {

        
        // check if the master if the master is connected 
        // by checking the watchdog status
        let master_connected: bool = match lan9252.read_register_indirect(0x805 as u16, 1).await {
            Ok(al) => {
                debug!("Watchdog Status: {:#x}", al[0]&0b1000);
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
            if data[5] != 0x4 {
                error!("Wrong FoE protocol!");
                continue;
            }

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
            }else if data[6] == 0x3 {
                let data_chunk = u16::from_le_bytes(data[0..2].try_into().unwrap()) - 6;
                no_received_bytes = no_received_bytes + data_chunk as u32;
                info!("Received data in bytes: {:?}B", no_received_bytes);
            }else{
                error!("Unknown command! {:?}", data[6]);
            }

            // a a microsecond delay to avoid reading the same data
            Timer::after(Duration::from_micros(1)).await;

            let mut data_write = [0x00u8; 128];

            // construct the datagram to send to the master
            data_write[5] = 0x04; // set the FoE protocol

            data_write[0] = 0x01; // set the data size to 1
            data_write[6] = 0x04; // acknowledge the data received
    
            // heder is 6 bytes and then the 
            match lan9252.write_bytes_large(&data_write, Lan9252Memory::OrbitaInAck as u16).await {
                Ok(_) => {}
                Err(e) => {
                    error!("Write data error! {:?}", e)
                }
            }
        }
        
    }
}
