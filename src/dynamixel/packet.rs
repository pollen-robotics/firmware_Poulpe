use super::{v1::DynamixelError, Register};

pub enum InstructionPacket {
    Ping,
    ReadData(Register),
    WriteData(Register, [u8; 16]),
}

impl InstructionPacket {
    pub fn parse(data: &[u8], id: u8) -> Result<Self, DynamixelError> {
        if data.len() < 6 {
            return Err(DynamixelError {});
        }

        if data[0] != 0xFF || data[1] != 0xFF {
            return Err(DynamixelError {});
        }

        if data[2] != id {
            return Err(DynamixelError {});
        }

        let len = data[3] as usize;
        if data.len() != len + 4 {
            return Err(DynamixelError {});
        }

        let msg_crc = *data.last().unwrap();
        let calc_crc = crc(&data[2..data.len() - 1]);
        if msg_crc != calc_crc {
            return Err(DynamixelError {});
        }

        let instruction = data[4];
        let params = &data[5..data.len() - 1];

        match instruction {
            0x01 => Ok(InstructionPacket::Ping),
            0x02 => Ok(InstructionPacket::ReadData(Register::from_addr(params[0]))),
            0x03 => {
                let mut data = [0u8; 16];
                data[..params.len()].copy_from_slice(&params[1..]);

                Ok(InstructionPacket::WriteData(
                    Register::from_addr(params[0]),
                    data,
                ))
            }
            _ => Err(DynamixelError {}),
        }
    }
}

pub struct StatusPacket {
    id: u8,
    error: u8,
    params: [u8; 64],
}

impl StatusPacket {
    fn with_params(id: u8, error: u8, params: &[u8]) -> Self {
        let mut buff = [0u8; 64];
        buff[..params.len()].copy_from_slice(params);

        StatusPacket {
            id,
            error,
            params: buff,
        }
    }

    pub fn pong() -> Self {
        StatusPacket::with_params(0, 0, &[])
    }

    pub fn ack() -> Self {
        StatusPacket::with_params(0, 0, &[])
    }

    pub fn with_register(reg: Register, data: &[u8]) -> Self {
        StatusPacket::with_params(0, 0, &[])
    }

    pub fn to_bytes(&self, buffer: &mut [u8]) -> usize {
        buffer[0] = 0xFF;
        buffer[1] = 0xFF;
        buffer[2] = self.id;
        buffer[3] = 2 + self.params.len() as u8;
        buffer[4] = self.error;

        buffer[5..5 + self.params.len()].copy_from_slice(&self.params);
        buffer[5 + self.params.len() + 1] = crc(&buffer[2..5 + self.params.len()]);

        5 + self.params.len() + 2
    }
}

fn crc(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for b in data {
        crc = crc.wrapping_add(*b);
    }
    !crc
}
