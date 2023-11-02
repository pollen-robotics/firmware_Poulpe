use defmt::Format;

use super::crc;

#[derive(Format)]
pub struct StatusPacket<const N: usize> {
    id: u8,
    error: u8,
    params: [u8; N],
}

impl StatusPacket<0> {
    pub fn ack(id: u8, error: u8) -> Self {
        StatusPacket {
            id,
            error,
            params: [0; 0],
        }
    }
}

impl<const N: usize> StatusPacket<N> {
    pub fn with_value(id: u8, error: u8, value: [u8; N]) -> Self {
        StatusPacket {
            id,
            error,
            params: value,
        }
    }
}

impl<const N: usize> StatusPacket<N> {
    pub fn to_bytes(&self) -> [u8; N + 6]
    where
        [u8; N + 6]: Sized,
    {
        let mut bytes = [0; N + 6];
        bytes[0] = 0xFF;
        bytes[1] = 0xFF;
        bytes[2] = self.id;
        bytes[3] = 2 + N as u8;
        bytes[4] = self.error;
        bytes[5..N + 5].copy_from_slice(&self.params);
        bytes[N + 5] = crc(&bytes[2..N + 5]);
        bytes
    }
}
