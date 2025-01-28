use defmt::{error, info, unwrap, Format};
use embassy_stm32::flash::{get_flash_regions, Async, Bank1Region, Blocking, Flash};
use embassy_stm32::peripherals as p;
use embassy_stm32::peripherals::FLASH;
use embassy_time::{block_for, Duration, Timer};

use crate::config::N_AXIS;

// the address of the 5th sector of the flash memory
// it can be any other sector that is not used by the program
const ADDR: u32 = 7 * 128 * 1024; // This is the offset into bank 1

// data structure to be stored in flash
// this structure can be as big as necessary
// for the moment it is just a board_id and 3 sensor offsets
#[derive(Debug, Format, Clone, Copy, PartialEq)]
pub struct FlashData {
    pub board_id: u8,
    pub sensor_offsets: [f32; N_AXIS],
}

/**
 * Implementation of the FlashData structure
 * It has methods to convert the data structure to a byte array and vice versa
 * It also implements a method to check if the data structure is valid
 */
impl FlashData {
    /**
     * Convert the data structure to a byte array
     * @return [u8; 32]
     */
    pub fn to_bytes(&self) -> [u8; 32] {
        let mut bytes = [0u8; 32];
        bytes[0] = self.board_id;
        for (i, offset) in self.sensor_offsets.iter().enumerate() {
            let offset_bytes = offset.to_le_bytes();
            bytes[1 + i * 4..1 + (i + 1) * 4].copy_from_slice(&offset_bytes);
        }
        bytes
    }

    /**
     * Convert a byte array to the data structure
     * @param bytes: [u8; 32]
     * @return FlashData
     */
    pub fn from_bytes(bytes: [u8; 32]) -> Self {
        let board_id = bytes[0];
        let mut sensor_offsets_bytes = [0u8; N_AXIS * 4];
        sensor_offsets_bytes.copy_from_slice(&bytes[1..N_AXIS * 4 + 1]);

        // convert bytes to f32 using a for loop
        let mut sensor_offsets = [0.0; N_AXIS];
        for i in 0..N_AXIS {
            sensor_offsets[i] = f32::from_le_bytes([
                sensor_offsets_bytes[i * 4],
                sensor_offsets_bytes[i * 4 + 1],
                sensor_offsets_bytes[i * 4 + 2],
                sensor_offsets_bytes[i * 4 + 3],
            ]);
        }
        Self {
            board_id,
            sensor_offsets,
        }
    }

    /**
     * Check if the data structure is valid
     * The data structure is valid if the board_id is 255 or if any of the sensor offsets is NaN
     *
     * @return bool
     */
    pub fn is_valid(&self) -> bool {
        self.board_id == 255 || self.sensor_offsets.iter().any(|&x| x.is_nan())
    }
}

pub struct FlashManager<'d> {
    flash_region: Bank1Region<'d, Blocking>,
}

impl<'d> FlashManager<'d> {
    pub async fn new(flash_config: p::FLASH) -> Self {
        let flash = Flash::new_blocking(flash_config)
            .into_blocking_regions()
            .bank1_region;
        // wait for the flash to be ready
        Timer::after(Duration::from_millis(400)).await;
        // return the flash
        Self {
            flash_region: flash,
        }
    }

    /**
     * Write data to flash, optionally erase the sector before writing
     * @param data: data to write to flash
     * @param erase: if true erase the sector before writing
     * @return Result<(), Error>
     */
    pub async fn write(
        &mut self,
        data: FlashData,
        erase: bool,
    ) -> Result<(), embassy_stm32::flash::Error> {
        let bytes = data.to_bytes();
        if erase {
            info!("Erasing...");
            match (self.flash_region.blocking_erase(ADDR, ADDR + 128 * 1024)) {
                Ok(()) => info!("Erase OK"),
                Err(e) => error!("Erase error: {:?}", e),
            }
        }
        Timer::after(Duration::from_millis(100)).await;
        info!("Writing...");
        match self.flash_region.blocking_write(ADDR, &bytes) {
            Ok(()) => info!("Write OK"),
            Err(e) => error!("Write error: {:?}", e),
        }
        Ok(())
    }

    /**
     * Write data to flash only if it is different from the data already in flash
     * Tried to write to flash multiple times if it fails
     *
     * @param data: data to write to flash
     * @param no_tries: number of tries to write to flash
     * @return Result<(), Error>
     */
    pub async fn lazy_checked_write(
        &mut self,
        data: FlashData,
        no_tries: i32,
    ) -> Result<(), embassy_stm32::flash::Error> {
        // verify if data already in flash
        match self.read() {
            Ok(read_data) => {
                if read_data == data {
                    info!("Data already in flash, skipping write");
                    return Ok(());
                }
            }
            Err(_) => {
                // if error reading, continue with write
            }
        }
        // try to write to flash 3 times if it fails stop the loop
        for try_i in 0..no_tries {
            info!("Saving to flash {:?}, try number {:?}", data, try_i);
            Timer::after(Duration::from_millis(100)).await;
            match self.write(data, true).await {
                Ok(()) => {
                    let read_data = match self.read() {
                        Ok(data) => data,
                        Err(e) => {
                            error!("Error reading data after write: {:?}", e);
                            continue;
                        }
                    };
                    info!("Data read: {:?}", read_data);
                    if read_data == data {
                        return Ok(());
                    } else {
                        error!("Data read does not match data written");
                    }
                }
                Err(e) => {
                    error!("Error writing data: {:?}", e);
                }
            }
        }
        error!("Failed to write to flash after {:?} tries", no_tries);
        Err(embassy_stm32::flash::Error::Prog)
    }

    /**
     * Read data from flash
     * @return Result<FlashData, Error>
     */
    pub fn read(&mut self) -> Result<FlashData, embassy_stm32::flash::Error> {
        let mut buf = [0u8; 32];
        match self.flash_region.blocking_read(ADDR, &mut buf) {
            Ok(()) => info!("Read OK"),
            Err(e) => {
                error!("Read error: {:?}", e);
                return Err(e);
            }
        }
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
