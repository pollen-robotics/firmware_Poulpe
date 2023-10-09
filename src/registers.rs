#![no_std]
#![no_main]
use defmt::*;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use {defmt_rtt as _, panic_probe as _};

pub struct Registers {
    pub buffer: [u8; 512],
}

// impl Registers {
//     pub fn new() -> Self {
//         Self { buffer: [0; 512] }
//     }
// }

pub static REGISTERS: Mutex<ThreadModeRawMutex, Registers> =
    Mutex::new(Registers { buffer: [0; 512] });
