use defmt::Format;

// Split Register into read and read/write
// Use a macro to generate the enum
// register!(CurrentPosition, Read);
// register!(TargetPosition, ReadWrite);

#[derive(Format)]
pub enum Register {
    CurrentPosition,
    TargetPosition,
}

impl Register {
    pub fn from_addr(addr: u8) -> Self {
        match addr {
            0x24 => Register::CurrentPosition,
            0x2A => Register::TargetPosition,
            _ => panic!("Invalid register address"),
        }
    }
}
