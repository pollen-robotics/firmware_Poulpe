// use crate::config;
// use core::sync::atomic::AtomicU8;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use modular_bitfield::prelude::*;

#[bitfield(bits = 8)]
pub struct DxlStatusError {
    input_voltage_error: bool,
    angle_limit_error: bool,
    overheating_error: bool,
    range_error: bool,
    checksum_error: bool,
    overload_error: bool,
    instruction_error: bool,
    unused: B1,
}

static DXL_STATUS_ERROR: Mutex<ThreadModeRawMutex, DxlStatusError> =
    Mutex::new(DxlStatusError::new());

#[repr(u8)]
enum MessageType {
    PingMessage = 1,
    ReadMessage = 2,
    WriteMessage = 3,
    Unknown = 0,
}
impl MessageType {
    fn from_u8(val: u8) -> MessageType {
        // sadness...
        match val {
            1 => MessageType::PingMessage,
            2 => MessageType::ReadMessage,
            3 => MessageType::WriteMessage,
            _ => MessageType::Unknown,
        }
    }
}

const MAX_DATA_LENGTH: u8 = 128;
pub const MAX_BUFFER_LENGTH: usize = 256;

pub struct DxlCom {
    id: u8,
    // rx_buffer: [u8; MAX_BUFFER_LENGTH],
    tx_buffer: [u8; MAX_BUFFER_LENGTH],
    // rx_buffer_index: usize,
    // rx_packet_length: u8,
}

#[non_exhaustive]
pub enum Error {
    BadPacket,
    BadCRC,
    BadInstruction,
}

#[repr(u8)]
pub enum RWAction<'a> {
    // Rx(u8),
    Tx(&'a [u8]),
    Ignore,
    Ok,
}
pub fn crc(data: &[u8]) -> u8 {
    // !data.iter().sum::<u8>() //error does not wrap

    let mut crc: u8 = 0;
    // debug!("DEBUG CRC DATA: {:?}", data);
    for b in data {
        crc = crc.wrapping_add(*b);
        // debug!("DEBUG CRC: b: {:?} crc: {:?}", *b, crc);
    }
    !crc
}

impl DxlCom {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            // rx_buffer: [0; 256],
            tx_buffer: [0; MAX_BUFFER_LENGTH],
            // rx_buffer_index: 0,
            // rx_packet_length: 0,
        }
    }

    pub async fn parse(&mut self, bytes: &[u8]) -> Result<RWAction, Error> {
        if bytes.len() < 6 {
            //Minimum packet size
            error!("packet size is {:?} <6", bytes.len());
            Err(Error::BadPacket)
        } else if bytes[0] == 0xff && bytes[1] == 0xff && bytes.len() == (bytes[3] + 4).into() {
            //Header is detected
            trace!("header ok",);
            if bytes[2] == self.id {
                /*
                        debug!("id is {:?}", bytes[2]);
                        debug!("len is {:?}", bytes[3]);
                        debug!("instr is {:?}", bytes[4]);
                        debug!("data: {:?}", bytes[2..bytes.len() - 1]);
                        // Dynamixel id is ok
                        debug!("crc is {:?}", crc(&bytes[2..bytes.len() - 1]));
                */
                if crc(&bytes[2..bytes.len() - 1]) != bytes[bytes.len() - 1] {
                    // debug!(
                    //     "bad crc, seen: {:?} computed {:?}",
                    //     bytes[bytes.len() - 1],
                    //     self.crc(&bytes[2..bytes.len() - 2])
                    // );
                    DXL_STATUS_ERROR.lock().await.set_checksum_error(true);
                    return Err(Error::BadCRC);
                } else {
                    trace!("Packet is ok");
                    //Packet seems ok
                    Ok(self.handle_instruction(bytes).await)

                    // Ok(RWAction::Ok) //TODO
                }
            } else {
                //Packet is not for us
                Ok(RWAction::Ignore)
            }
        } else {
            //Ill formed packet?
            error!("Malformed packet: {:?}", bytes);
            DXL_STATUS_ERROR.lock().await.set_instruction_error(true);
            Err(Error::BadPacket)
        }
    }
    pub async fn get_status_packet(&mut self) -> [u8; 6] {
        let mut status: [u8; 6] = [0, 0, 0, 0, 0, 0];
        //status packet
        status[0] = 0xff;
        status[1] = 0xff;
        status[2] = self.id;
        status[3] = 2;
        status[4] = DXL_STATUS_ERROR.lock().await.bytes[0]; //Error byte
        status[5] = crc(&status[2..5]);
        status
    }
    async fn handle_instruction(&mut self, data: &[u8]) -> RWAction {
        match MessageType::from_u8(data[4]) {
            MessageType::PingMessage => {
                //answer to the ping
                info!("PONG!");

                let errbyte = DXL_STATUS_ERROR.lock().await.bytes[0];

                // let rescrc: u8 = !(self.id + errbyte + 0x02);
                let rescrc: u8 = crc(&[self.id, errbyte, 0x02]);

                let sp = [0xff, 0xff, self.id, 0x02, errbyte, rescrc];
                self.tx_buffer[..6].copy_from_slice(&sp);
                return RWAction::Tx(&self.tx_buffer[..6]);
            }
            MessageType::ReadMessage => {
                //return the read value from register
                let addr: usize = data[5].into();
                let size: usize = data[6].into(); //TODO check that size is <MAX_DATA_LENGTH?
                trace!("Packet is READ addr: {:?} size: {:?}", addr, size);
                self.tx_buffer[0] = 0xff;
                self.tx_buffer[1] = 0xff;
                self.tx_buffer[2] = self.id;
                self.tx_buffer[3] = size as u8 + 2;
                self.tx_buffer[4] = DXL_STATUS_ERROR.lock().await.bytes[0]; //Error byte

                // {
                //     let mut registers = registers::REGISTERS.lock().await;
                //     self.tx_buffer[5..5 + size as usize]
                //         .clone_from_slice(&registers.buffer[addr..addr + size]);
                // }

                let mut buffer = [0u8; MAX_BUFFER_LENGTH]; // Ensure buffer is of appropriate size

                match crate::config::dxl_registers_read_by_address(addr, size, &mut buffer).await {
                    Ok(()) => {
                        self.tx_buffer[5..5 + size].clone_from_slice(&buffer[0..size]);
                        self.tx_buffer[size + 5] = crc(&self.tx_buffer[2..4 + size + 1]); //ICI
                        RWAction::Tx(&self.tx_buffer[0..size + 6])
                    }
                    Err(()) => {
                        // RWAction::Ok //TODO Status error?

                        DXL_STATUS_ERROR.lock().await.set_instruction_error(true);
                        //status packet
                        self.tx_buffer[0] = 0xff;
                        self.tx_buffer[1] = 0xff;
                        self.tx_buffer[2] = self.id;
                        self.tx_buffer[3] = 2;
                        self.tx_buffer[4] = DXL_STATUS_ERROR.lock().await.bytes[0]; //Error byte
                        self.tx_buffer[5] = crc(&self.tx_buffer[2..5]);

                        //FIXME: Here we immedatly clear this error
                        DXL_STATUS_ERROR.lock().await.set_instruction_error(false);

                        // RWAction::Ok
                        RWAction::Tx(&self.tx_buffer[0..6])
                    }
                }
            }
            MessageType::WriteMessage => {
                //write the value to the register
                //return status packet
                let addr: usize = data[5].into();
                let size: usize = <u8 as Into<usize>>::into(data[3]) - (3_usize);
                trace!("Packet is WRITE addr: {:?} size: {:?}", addr, size);
                let mut buffer = [0u8; MAX_BUFFER_LENGTH]; // Ensure buffer is of appropriate size
                buffer[0..size].clone_from_slice(&data[6..6 + size]);

                match crate::config::dxl_registers_write_by_address(addr, size, &buffer[0..size])
                    .await
                {
                    Ok(()) => {
                        //status packet
                        self.tx_buffer[0] = 0xff;
                        self.tx_buffer[1] = 0xff;
                        self.tx_buffer[2] = self.id;
                        self.tx_buffer[3] = 2;
                        self.tx_buffer[4] = DXL_STATUS_ERROR.lock().await.bytes[0]; //Error byte
                        self.tx_buffer[5] = crc(&self.tx_buffer[2..5]);

                        // RWAction::Ok
                        RWAction::Tx(&self.tx_buffer[0..6])
                    }
                    Err(()) => {
                        DXL_STATUS_ERROR.lock().await.set_instruction_error(true);
                        //status packet
                        self.tx_buffer[0] = 0xff;
                        self.tx_buffer[1] = 0xff;
                        self.tx_buffer[2] = self.id;
                        self.tx_buffer[3] = 2;
                        self.tx_buffer[4] = DXL_STATUS_ERROR.lock().await.bytes[0]; //Error byte
                        self.tx_buffer[5] = crc(&self.tx_buffer[2..5]);

                        //FIXME: Here we immedatly clear this error
                        DXL_STATUS_ERROR.lock().await.set_instruction_error(false);

                        // RWAction::Ok
                        RWAction::Tx(&self.tx_buffer[0..6])
                    }
                }
            }
            MessageType::Unknown => {
                debug!("Packet is UNKNOWN");
                //nothing, it should not happen...
                RWAction::Ignore
            }
        }
    }
}
