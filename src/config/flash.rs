use embassy_stm32::flash::{Flash, Async, get_flash_regions, Blocking, Bank1Region};
use embassy_stm32::peripherals::{FLASH};
use embassy_stm32::peripherals as p;
use defmt::{info, unwrap, error, Format};
use embassy_time::{block_for, Duration, Timer};

// the address of the 5th sector of the flash memory
// it can be any other sector that is not used by the program
const ADDR: u32 = 5*128*1024; // This is the offset into bank 1

// data structure to be stored in flash
// this structure can be as big as necessary
// for the moment it is just a board_id and 3 sensor offsets
#[derive(Debug, Format, Clone, Copy, PartialEq)]
pub struct FlashData {
    pub board_id: u8,
    pub sensor_offsets: [f32; 3],
}

impl FlashData {
    pub fn to_bytes(&self) -> [u8; 32] {
        let mut bytes = [0u8; 32];
        bytes[0] = self.board_id;
        for (i, offset) in self.sensor_offsets.iter().enumerate() {
            let offset_bytes = offset.to_le_bytes();
            bytes[1 + i*4..1 + (i+1)*4].copy_from_slice(&offset_bytes);
        }
        bytes
    }

    pub fn from_bytes(bytes: [u8; 32]) -> Self {
        let board_id = bytes[0];
        let mut sensor_offsets_bytes = [0u8; 12];
        sensor_offsets_bytes.copy_from_slice(&bytes[1..13]);
        let sensor_offsets = [
            f32::from_le_bytes([sensor_offsets_bytes[0], sensor_offsets_bytes[1], sensor_offsets_bytes[2], sensor_offsets_bytes[3]]),
            f32::from_le_bytes([sensor_offsets_bytes[4], sensor_offsets_bytes[5], sensor_offsets_bytes[6], sensor_offsets_bytes[7]]),
            f32::from_le_bytes([sensor_offsets_bytes[8], sensor_offsets_bytes[9], sensor_offsets_bytes[10], sensor_offsets_bytes[11]]),
        ];
        Self {
            board_id,
            sensor_offsets,
        }
    }

}


pub struct FlashManager<'d>{
    flash_region: Bank1Region<'d, Blocking>,
}

impl<'d> FlashManager<'d> {
    pub fn new(flash_config: p::FLASH) -> Self {  
        let flash = Flash::new_blocking(flash_config).into_blocking_regions().bank1_region;
        // wait for the flash to be ready
        Timer::after(Duration::from_millis(400));
        // return the flash
        Self {
            flash_region: flash,
        }
        
    }

    pub fn write(&mut self, data: FlashData) -> Result<(), embassy_stm32::flash::Error> {
        let bytes = data.to_bytes();
        info!("Erasing...");
        unwrap!(self.flash_region.blocking_erase(ADDR, ADDR + 128 * 1024));
        Timer::after(Duration::from_millis(100));
        info!("Writing...");
        match self.flash_region.blocking_write(
            ADDR,
            &bytes
        ){
            Ok(()) => info!("Write OK"),
            Err(e) => error!("Write error: {:?}", e),
        }
        Ok(())
    }

    pub fn lazy_checked_write(&mut self, data: FlashData) -> Result<(), embassy_stm32::flash::Error> {
        // verify if data already in flash 
        match self.read(){
            Ok(read_data) => {
                if read_data == data {
                    info!("Data already in flash, skipping write");
                    return Ok(());
                }
            },
            Err(_) => {
                // if error reading, continue with write
            }
        }
        match self.write(data){
            Ok(()) => {
                let read_data = self.read().unwrap();
                info!("Data read: {:?}", read_data);
                if read_data == data {
                    Ok(())
                } else {
                    error!("Data read does not match data written");
                    Err(embassy_stm32::flash::Error::Prog)
                }
            },
            Err(e) => {
                error!("Error writing data: {:?}", e);
                Err(e)
            }
        }
    }

    pub fn read(&mut self) -> Result<FlashData, embassy_stm32::flash::Error> {
        let mut buf = [0u8; 32];
        unwrap!(self.flash_region.blocking_read(ADDR, &mut buf));
        Ok(FlashData::from_bytes(buf))
    }

    
    // pub fn test(&mut self) -> Result<(), embassy_stm32::flash::Error> {
    //     // self.flash_instance.read(addr, buf)
        
    //     const ADDR: u32 = 5*128*1024; // This is the offset into bank 1, the absolute address is 0x8_0000
    //     // let mut f = Flash::new_blocking(p.FLASH).into_blocking_regions().bank1_region;
    //     info!("{}",get_flash_regions());

    //     let mut buf = [0u8; 2];
    //     unwrap!(self.flash_region.blocking_read(ADDR, &mut buf));
    //     info!("Read: {:?}", buf);

    //     info!("Erasing...");
    //     unwrap!(self.flash_region.blocking_erase(ADDR, ADDR + 128 * 1024));
    //     info!("Writing...");
    //     let mut buf = [0u8; 32];
    //     buf[0] = 0x5;
    //     buf[1] = 0x9;
    //     buf[2] = 0x3;
    //     info!("Write: {:?}", buf);
    //     match self.flash_region.blocking_write(
    //         ADDR,
    //         &buf
    //     ){
    //         Ok(()) => info!("Write OK"),
    //         Err(e) => error!("Write error: {:?}", e),
    //     }
    //     info!("Reading...");
    //     let mut buf = [0u8; 32];
    //     unwrap!(self.flash_region.blocking_read(ADDR, &mut buf));
    //     info!("Read: {:?}", buf);

    //     info!("Reading...");
    //     let mut buf = [0u8; 2];
    //     unwrap!(self.flash_region.blocking_read(ADDR, &mut buf));
    //     info!("Read: {:?}", buf);

    //     Ok(())
    // }
}