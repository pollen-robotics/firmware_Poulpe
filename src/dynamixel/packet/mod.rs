mod error;
pub use error::ParsingError;

mod instruction_packet;
pub use instruction_packet::InstructionPacketKind;

mod status_packet;
pub use status_packet::StatusPacket;

pub fn crc(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for b in data {
        crc = crc.wrapping_add(*b);
    }
    !crc
}
