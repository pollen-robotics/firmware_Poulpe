use defmt::info;

 // - CoE has the Mailbox type 0x3.
//     - It has 3 Byte header
//         - bytes 0-1: request type (uint16)
//             - 0x2 << 12 (0x40) SDO request
//             - 0x3 << 12 (0x60) SDO response
//             - 0x8 << 12 
//         - byte 2:
//             - bits 0-1: size indicator
//                 - 0x0 - no indicaiton
//                 - 0x1 - normal
//                 - 0x3 - expedited, size specified
//             - bits 2-3: size in bytes
//             - bits 5-6: request type
//                 - 0x1: download request
//                 - 0x2: upload request
//     - After the header it has the index of the SDO we want to read or write
//         - 2 byte index (uint16)
//         - 1 byte subindex (uint8)
//     - If its the download response or upload request, there is no more data in the datagram
//     - If its the download request or upload response, the data follows
//          - If the data is a 1 to 4 byte number (uint8, uint16, uint32), 
//              - the size indicator is 0x3 and size in bits is set the same as the data size
//              - the number is written directly after the index and the subindex
//          - If the data is longer than 4 bytes (uint64) or if its not a number (string)
//              - the size indicator is 0x1 and the data size is set to 0
//              - first 4 bytes after the index and the index are the size of the data
//              - the data follows after the size

#[derive(Debug, defmt::Format)]
pub struct CoEFrame<'a> {
    pub header: CoEFrameHeader,
    pub data: &'a [u8]
}

impl<'a> CoEFrame<'a> {
    pub fn new(mailbox_data: &'a [u8]) -> Result<Self, ()> {
        let header = coe_parse_header(mailbox_data);

        info!("CoE header: {:?}", header);

        // mailbox has 6 bytes header
        // coe has 3 bytes header
        // coe then has 3 bytes of index and subindex
        let data_ind = 6 + 6;
        // check if expedited or normal
        if header.is_expedited() && (header.is_request() || header.is_response()){
            let size = header.size_bytes as usize;
            // then the data follows after the index and subindex in 
            // expedited mode
            Ok(CoEFrame {
                header,
                data: mailbox_data[data_ind..data_ind+size as usize].try_into().unwrap()
            })
        }else if header.is_normal() && (header.is_request() || header.is_response()){
            // the data size is in the first 4 bytes after the index and subindex
            let data_size = u32::from_le_bytes(mailbox_data[data_ind..data_ind+4].try_into().unwrap());
            // the data follows after the size
            Ok(CoEFrame {
                header,
                data: mailbox_data[data_ind+4..data_ind+4+data_size as usize].try_into().unwrap()
            })
        }else{
            Ok(CoEFrame {
                header,
                data: &[]
            })
        }
    }
}

#[derive(Debug, defmt::Format)]
pub struct CoEFrameHeader {
    pub request_type: u16,
    pub size_indicator: u8,
    pub size_bytes: u8,
    pub download: bool,
    pub index: u16,
    pub sub_index: u8,
}

impl<'a> CoEFrame<'a>{
    
    pub fn is_request(&self) -> bool {
        self.header.request_type == 0x2
    }

    pub fn is_response(&self) -> bool {
        self.header.request_type == 0x3
    }

    pub fn is_download(&self) -> bool {
        self.header.download
    }

    pub fn is_upload(&self) -> bool {
        !self.header.download
    }

    pub fn is_expedited(&self) -> bool {
        self.header.is_expedited()
    }

    pub fn is_normal(&self) -> bool {
        self.header.is_normal()
    }

    pub fn get_index(&self) -> u16 {
        self.header.index
    }

    pub fn get_subindex(&self) -> u8 {
        self.header.sub_index
    }

    pub fn get_sdo_entry(&self) -> (u16, u8) {
        (self.header.index, self.header.sub_index)
    }

    pub fn get_data_size(&self) -> u32 {
        self.data.len() as u32
    }
}

impl CoEFrameHeader {
    pub fn is_expedited(&self) -> bool {
        self.size_indicator == 0x3
    }

    pub fn is_normal(&self) -> bool {
        self.size_indicator == 0x1
    }

    pub fn is_request(&self) -> bool {
        self.request_type == 0x2
    }

    pub fn is_response(&self) -> bool {
        self.request_type == 0x3
    }
}

pub fn coe_parse_header(data: &[u8]) -> CoEFrameHeader{
    CoEFrameHeader{
        request_type : u16::from_le_bytes(data[6..8].try_into().unwrap()) >> 12,
        size_indicator : data[8] & 0b11,
        size_bytes : if data[8] & 0b11 == 3 { 4 - ((data[8] >> 2) & 0b11)} else {0},
        download : (data[8] >> 5) & 0b11 == 0x1,
        index : u16::from_le_bytes(data[9..11].try_into().unwrap()),
        sub_index : data[11],
    }
}

pub fn coe_prepare_down_response(data_coe_write: &mut [u8]){
    // mailbox has 6 bytes header
    let mut coe_header_start_ind:usize = 6;
    // coe has 3 bytes header
    // downooad response
    data_coe_write[coe_header_start_ind+2] = 0x3 <<  5;
}

pub fn coe_prepare_up_response(data_coe_write: &mut [u8], data: &[u8], is_numeral: bool) {
    // mailbox has 6 bytes header
    let mut coe_header_start_ind:usize = 6;
    // coe has 3 bytes header + 3 bit index + subindex
    let mut data_start_ind:usize = coe_header_start_ind + 6;
    // set the inital data length to 10
    data_coe_write[0] = 10; // data length
    // check the size of the data 
    let data_sdo_len = data.len();

    // if the data is a number and the size is less than 4 bytes
    // we can write the data directly after the index and subindex
    // and set the size indicator to 0x3 and its size
    // if the data is a number and the size is more than 4 bytes or if its not a number (string)
    // we write the size of the data in the first 4 bytes and the data follows after the size
    // and set the size indicator to 0x1
    if is_numeral || data_sdo_len <= 4 {
        // upload response = download request + 0x2
        data_coe_write[coe_header_start_ind + 2] = 0x2 << 5;
        // set the size as well
        data_coe_write[coe_header_start_ind + 2] |= (4-data_sdo_len as u8) << 2;
        // set the size indicator
        data_coe_write[coe_header_start_ind + 2] |= 0x3;
    }else if !is_numeral || data_sdo_len > 4 {
        // upload response = download request + 0x2
        data_coe_write[coe_header_start_ind + 2] = 0x2 << 5;
        // set the size as well
        data_coe_write[coe_header_start_ind + 2] |= 0 << 2;
        // set the size indicator
        data_coe_write[coe_header_start_ind + 2] |= 0x1;

        // write the size to the first 4 bytes
        data_coe_write[data_start_ind..data_start_ind+4].copy_from_slice(&u32::to_le_bytes(data_sdo_len as u32));
        // inidcate that the data starts after the size (uint32)
        data_start_ind += 4;
    }
    data_coe_write[data_start_ind..data_start_ind+data_sdo_len].copy_from_slice(&data);
    data_coe_write[0] += data_sdo_len  as u8;

}


pub fn coe_prepare_dataframe(index: u16, sub_index: u8, response: bool) -> [u8; 128]{
    // start preparing the response
    let mut data_write = [0x00u8; 128];
    data_write[0] = 10; // initial data length
    data_write[5] = 0x03; // set the CoE protocol
    // mailbox header size is 6 bytes then the the coe dataframe comes
    let mut coe_start_ind:usize = 6;

    if response  {// SDO response
        data_write[coe_start_ind..coe_start_ind+2].copy_from_slice(&u16::to_le_bytes(0x3 << 12)); 
    }else{ // SDO request
        data_write[coe_start_ind..coe_start_ind+2].copy_from_slice(&u16::to_le_bytes(0x2 << 12)); 
    }
    // index and subindex
    data_write[coe_start_ind+3..coe_start_ind+5].copy_from_slice(&u16::to_le_bytes(index));
    data_write[coe_start_ind+5] = sub_index;
    return data_write;
}

