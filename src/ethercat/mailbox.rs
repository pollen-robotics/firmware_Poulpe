// - The Mailbox has a 6 Byte header
//     - 2 Byte data size
//     - 5th Byte frame type (0x3 for CoE, 0x4 for FoE)
// - The data follows the header

use defmt::error;

#[derive(Debug, defmt::Format)]
pub struct MailboxFrame {
    pub header: MailboxFrameHeader,
}

impl MailboxFrame {
    pub fn new(data: &[u8]) -> Result<Self, ()> {
        Ok(MailboxFrame {
            header: MailboxFrameHeader {
                // mailbox has a 6 byte header
                // first 2 bytes are the data size
                // 5th byte is the frame type (0x3 for CoE, 0x4 for FoE)
                data_size: u16::from_be_bytes([data[0], data[1]]),
                frame_type: match data[5] {
                    0x3 => MailboxType::CoE,
                    0x4 => MailboxType::FoE,
                    _ => {
                        error!("Unknown mailbox type");
                        return Err(());
                    }
                },
            }
        })
    }

    pub fn get_mailbox_type(&self) -> MailboxType {
        self.header.frame_type
    }
}

#[derive(Debug, defmt::Format)]
pub struct MailboxFrameHeader {
    pub data_size: u16,
    pub frame_type: MailboxType,
}

#[derive(Debug, defmt::Format, Clone, Copy)]
pub enum MailboxType {
    CoE,
    FoE,
}