use defmt::{error, info};
 // - FoE has the Mailbox type 0x4.
//     - It has 6 Byte header



#[derive(Debug, defmt::Format)]
pub struct FoEFrameHeader {
    pub request_type: FoERequestType,
    pub data_size: u16,
    pub packet_number: u32,
}

pub struct FoEFrame<'a> {
    pub header: FoEFrameHeader,
    pub data: &'a [u8],
}

impl<'a> FoEFrame<'a>{
    pub fn new(mailbox_data: &'a [u8]) -> Result<Self, ()> {
        let header = match foe_parse_header(mailbox_data){
            Ok(header) => header,
            Err(_) => {
                error!("Failed to parse FoE header");
                return Err(());
            }
        };

        info!("FoE header: {:?}", header);

        // mailbox has 6 bytes header
        // foe has 6 bytes header
        let data_ind = 6 + 6;
        let size = header.data_size as usize  - 6;
        // the data follows after the header
        Ok(FoEFrame {
            header,
            data: mailbox_data[data_ind..data_ind+size as usize].try_into().unwrap()
        })
    }

    pub fn get_request_type(&self) -> FoERequestType {
        self.header.request_type
    }

    pub fn get_data_size(&self) -> u16 {
        self.data.len() as u16
    }
}

//- request type (uint8) : byte 0
// - 0x1 - read request  (WRQ)
// - 0x2 - write request (RRQ)
// - 0x3 - data (DATA)
// - 0x4 - acknowledge (ACK)
// - 0x5 - error
// - 0x6 busy
#[derive(Debug, defmt::Format, Clone, Copy)]
pub enum FoERequestType {
    ReadRequest = 0x1,
    WriteRequest = 0x2,
    Data = 0x3,
    Acknowledge = 0x4,
    Error = 0x5,
    Busy = 0x6,
}

pub fn foe_parse_header(data: &[u8]) -> Result<FoEFrameHeader, ()>{
    
    let data_size = u16::from_le_bytes(data[0..2].try_into().unwrap());
    // mailbox has 6 bytes header
    let foe_data_index = 6;
    
    let request_type = match data[foe_data_index] {
        0x1 => FoERequestType::ReadRequest,
        0x2 => FoERequestType::WriteRequest,
        0x3 => FoERequestType::Data,
        0x4 => FoERequestType::Acknowledge,
        0x5 => FoERequestType::Error,
        0x6 => FoERequestType::Busy,
        _ => {
            error!("Unknown FoE request type");
            return Err(());
        }
    };

    let packet_number = u32::from_le_bytes(data[foe_data_index+2..foe_data_index+6].try_into().unwrap());

    Ok(FoEFrameHeader {
        request_type,
        data_size,
        packet_number,
    })
}

pub fn foe_prepare_acknowledge(data: &mut [u8]){
    data[0] = 1; // data length
    data[5] = 0x04; // set the FoE protocol
    data[6] = FoERequestType::Acknowledge as u8; // acknowledge the data received
}

pub fn foe_prepare_error(data: &mut [u8]){
    data[0] = 1; // data length
    data[5] = 0x04; // set the FoE protocol
    data[6] = FoERequestType::Error as u8; // error
}