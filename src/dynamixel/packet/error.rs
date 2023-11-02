use defmt::Format;

#[derive(Format)]
pub enum ParsingError {
    IgnorePacket(u8, u8),
    InvalidChecksum,
    InvalidPacket,
    UnkownInstruction(u8),
}
