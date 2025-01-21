use defmt::{error, info, debug};
use core::str;
 // - FoE has the Mailbox type 0x4.
//     - It has 6 Byte header



#[derive(Debug, defmt::Format, Copy, Clone)]
pub struct FoEFrameHeader {
    pub request_type: FoERequestType,
    pub data_size: u16,
    pub packet_number: u32,
}

#[derive(Debug, defmt::Format, Clone, Copy)]
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

        debug!("FoE header: {:?}", header);

        // mailbox has 6 bytes header
        // foe has 6 bytes header
        let data_ind = 6 + 6;
        let size = header.data_size as usize  - 6;

        let data = match mailbox_data[data_ind..data_ind+size].try_into(){
            Ok(data) => data,
            Err(_) => {
                error!("Failed to parse data");
                return Err(());
            }
        };

        // the data follows after the header
        Ok(FoEFrame {
            header,
            data
        })
    }

    pub fn get_request_type(&self) -> FoERequestType {
        self.header.request_type
    }

    pub fn get_data_size(&self) -> u16 {
        self.data.len() as u16
    }

    pub fn is_full_packet(&self) -> bool {
        self.data.len() < 128 - 6 - 6  // 128 is the max size of the mailbox 
                                       // 6 is the mailbox header size
                                       // 6 is the FoE header size
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
    
    let data_size = u16::from_le_bytes(
        match data[0..2].try_into(){
            Ok(data_size) => data_size,
            Err(_) => {
                error!("Failed to parse data size");
                return Err(());
            }
        }
    );
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


    let packet_number = u32::from_le_bytes(
        match data[foe_data_index+2..foe_data_index+6].try_into(){
            Ok(packet_number) => packet_number,
            Err(_) => {
                error!("Failed to parse packet number");
                return Err(());
            }
        }
    );

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

pub struct FoEObject<'a> {
    pub name: &'a str,
    pub buffer: FoEBuffer,
    pub no_received_packets: u32,
    pub no_received_bytes: u32,
    pub no_written_bytes : u32
}

impl<'a> FoEObject<'a>{
    // pub fn new(filename: &'a [u8]) -> Result<Self, ()> {
        
    //     let name = match str::from_utf8(filename.try_into().unwrap()){
    //         Ok(string) => {
    //             debug!("File name: {}", string);
    //             string
    //         },
    //         Err(e) => {
    //             error!("Error parsing file name: {:?}", filename);
    //             return Err(());
    //         }
    //     };

    pub fn new() -> Result<Self, ()> {

        Ok(FoEObject {
            name: "",
            buffer: FoEBuffer {
                data: [255; 4096],
                size: 4096,
                data_len: 0
            },
            no_received_packets: 0,
            no_received_bytes: 0,
            no_written_bytes: 0
        })
    }

    pub fn empty() -> Self {
        FoEObject {
            name: "",
            buffer: FoEBuffer {
                data: [255; 4096],
                size: 4096,
                data_len: 0
            },
            no_received_packets: 0,
            no_received_bytes: 0,
            no_written_bytes: 0
        }
    }


    pub fn get_filesize(&self) -> u32 {
        self.no_received_bytes
    }

    pub fn fill_buffer(&mut self, data: &[u8]) -> usize {
        
        let mut data_len = data.len();
        if data_len  + self.buffer.data_len > self.buffer.size {
            data_len = self.buffer.size - self.buffer.data_len;
        }

        self.buffer.data[self.buffer.data_len..self.buffer.data_len+data_len].copy_from_slice(&data[0..data_len]);
        self.buffer.data_len += data_len;

        self.no_received_packets += 1;
        self.no_received_bytes += data_len as u32;

        data_len
    }

    pub fn clear_buffer(&mut self){
        self.buffer.data_len = 0;
    }

}

pub struct FoEBuffer{
    pub data: [u8; 4096],
    pub size: usize,
    pub data_len: usize
}