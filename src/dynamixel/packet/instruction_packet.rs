use defmt::{Format, trace};

use super::{crc, ParsingError};

#[derive(Format)]
pub enum InstructionPacketKind<'d> {
    Ping(PingPacket),
    ReadData(ReadDataPacket),
    WriteData(WriteDataPacket<'d>),
}

impl<'d> InstructionPacketKind<'d> {
    pub fn parse(bytes: &[u8], receiver_id: u8) -> Result<InstructionPacketKind, ParsingError> {
        // [0xFF, 0xFF, ID, Length, Instruction, Param 1, ..., Checksum]
        if bytes.len() < 6 {
            return Err(ParsingError::InvalidPacket);
        }

/*
        if bytes[0] != 0xFF || bytes[1] != 0xFF {
            return Err(ParsingError::InvalidPacket);
        }

        let id = bytes[2];
        if id != receiver_id {
            return Err(ParsingError::IgnorePacket(receiver_id, id));
        }

        let length = bytes[3];
        if length as usize != bytes.len() - 4 {
            return Err(ParsingError::InvalidPacket);
        }

        let instruction = bytes[4];

        let params = &bytes[5..bytes.len() - 1];

        let received_crc = *bytes.last().unwrap();
        let calculated_crc = crc(&bytes[2..bytes.len() - 1]);
*/

	//At least it is easy to find a complete packet inside a buffer
	let mut idx:usize = 0;

	while  !(bytes[idx] == 0xFF && bytes[idx+1] == 0xFF)
	{
	    idx+=1;
	    if bytes.len() - idx < 6 {
		return Err(ParsingError::InvalidPacket);
	    }

	}

        let id = bytes[idx+2];
        if id != receiver_id {
            return Err(ParsingError::IgnorePacket(receiver_id, id));
        }

        let length = bytes[idx+3];
        if length as usize != bytes.len()-idx - 4 {
            return Err(ParsingError::InvalidPacket);
        }

        let instruction = bytes[idx+4];
        let params = &bytes[idx+5..idx+5+(length as usize) -2];
	let received_crc = bytes[idx+5+(length as usize) -2];
        let calculated_crc = crc(&bytes[idx+2..idx+5+(length as usize) - 2]);

        if received_crc != calculated_crc {
            return Err(ParsingError::InvalidChecksum);
        }

        match instruction {
            0x01 => Ok(InstructionPacketKind::Ping(PingPacket { id })),
            0x02 => Ok(InstructionPacketKind::ReadData(ReadDataPacket {
                id,
                address: params[0],
                data_length: params[1],
            })),
            0x03 => Ok(InstructionPacketKind::WriteData(WriteDataPacket {
                id,
                address: params[0],
                data: &params[1..],
            })),
            i => Err(ParsingError::UnkownInstruction(i)),
        }
    }
}

#[derive(Format)]
pub struct PingPacket {
    pub id: u8,
}

#[derive(Format)]
pub struct ReadDataPacket {
    pub id: u8,
    pub address: u8,
    pub data_length: u8,
}

#[derive(Format)]
pub struct WriteDataPacket<'d> {
    pub id: u8,
    pub address: u8,
    pub data: &'d [u8],
}
