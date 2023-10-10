#![no_std]
#![no_main]

use crate::config;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

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

pub struct DxlCom {
    id: u8,
    rx_buffer: [u8; 256],
    tx_buffer: [u8; 256],
    rx_buffer_index: usize,
    rx_packet_length: u8,
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
fn crc(data: &[u8]) -> u8 {
    !data.iter().sum::<u8>()
}

impl DxlCom {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            rx_buffer: [0; 256],
            tx_buffer: [0; 256],
            rx_buffer_index: 0,
            rx_packet_length: 0,
        }
    }

    pub async fn parse(&mut self, bytes: &[u8]) -> Result<RWAction, Error> {
        if bytes.len() < 6 {
            //Minimum packet size
            debug!("packet size is {:?} <6", bytes.len());
            Err(Error::BadPacket)
        } else if bytes[0] == 0xff && bytes[1] == 0xff && bytes.len() == (bytes[3] + 4).into() {
            //Header is detected
            debug!("header ok",);
            if bytes[2] == self.id {
                debug!("id is {:?}", bytes[2]);
                // Dynamixel id is ok
                if crc(&bytes[2..bytes.len() - 1]) != bytes[bytes.len() - 1] {
                    // debug!(
                    //     "bad crc, seen: {:?} computed {:?}",
                    //     bytes[bytes.len() - 1],
                    //     self.crc(&bytes[2..bytes.len() - 2])
                    // );
                    return Err(Error::BadCRC);
                } else {
                    debug!("Packet is ok");
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
            Err(Error::BadPacket)
        }
    }

    async fn handle_instruction(&mut self, data: &[u8]) -> RWAction {
        match MessageType::from_u8(data[4]) {
            MessageType::PingMessage => {
                //answer to the ping
                info!("PONG!");
                let rescrc: u8 = !(self.id + 0x02);
                let sp = [0xff, 0xff, self.id, 0x02, 0x00, rescrc];
                self.tx_buffer[..6].copy_from_slice(&sp);
                return RWAction::Tx(&self.tx_buffer[..6]);
            }
            MessageType::ReadMessage => {
                //return the read value from register
                let addr: usize = data[5].into();
                let size: usize = data[6].into();
                self.tx_buffer[0] = 0xff;
                self.tx_buffer[1] = 0xff;
                self.tx_buffer[2] = self.id;
                self.tx_buffer[3] = size as u8 + 2;
                self.tx_buffer[4] = 0; //Error byte

                // {
                //     let mut registers = registers::REGISTERS.lock().await;
                //     self.tx_buffer[5..5 + size as usize]
                //         .clone_from_slice(&registers.buffer[addr..addr + size]);
                // }

                let mut buffer = [0u8; 255]; // Ensure buffer is of appropriate size
                crate::config::dxl_registers_read_by_address(addr, size, &mut buffer)
                    .await
                    .expect("Read failed"); //TODO
                self.tx_buffer[5..5 + size as usize].clone_from_slice(&buffer[0..size]);

                self.tx_buffer[size as usize + 5] = crc(&self.tx_buffer[2..4 + size]);

                RWAction::Tx(&self.tx_buffer[0..size as usize + 6])
            }
            MessageType::WriteMessage => {
                //write the value to the register
                RWAction::Ok
            }
            MessageType::Unknown => {
                //nothing
                RWAction::Ignore
            }
        }
    }
}
