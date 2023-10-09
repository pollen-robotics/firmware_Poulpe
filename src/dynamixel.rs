#![no_std]
#![no_main]
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

    fn crc(&mut self, data: &[u8]) -> u8 {
        !data.iter().sum::<u8>()
    }

    pub fn parse(&mut self, bytes: &[u8]) -> Result<RWAction, Error> {
        if bytes.len() < 6 {
            //Minimum packet size
            debug!("packet size is {:?} <6", bytes.len());
            Err(Error::BadPacket)
        } else if bytes[0] == 0xff && bytes[1] == 0xff {
            //Header is detected
            debug!("header ok",);
            if bytes[2] == self.id {
                debug!("id is {:?}", bytes[2]);
                // Dynamixel id is ok
                if self.crc(&bytes[2..bytes.len() - 1]) != bytes[bytes.len() - 1] {
                    // debug!(
                    //     "bad crc, seen: {:?} computed {:?}",
                    //     bytes[bytes.len() - 1],
                    //     self.crc(&bytes[2..bytes.len() - 2])
                    // );
                    return Err(Error::BadCRC);
                } else {
                    debug!("Packet is ok");
                    //Packet seems ok
                    Ok(self.handle_instruction(bytes))

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

    fn handle_instruction(&mut self, data: &[u8]) -> RWAction {
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
                RWAction::Tx(&[0]) //TODO
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
