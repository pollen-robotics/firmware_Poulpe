use defmt::{debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
};
use embassy_stm32::{gpio::Pin, spi};

use libm::ceil;

use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_1::spi::SpiDevice;

#[allow(non_camel_case_types)]
#[allow(dead_code)]
pub enum Lan9252Registers {
    ECAT_PRAM_RD_DATA_START = 0x000,
    ECAT_PRAM_RD_DATA_END = 0x01C,
    ECAT_PRAM_WR_DATA_START = 0x020,
    ECAT_PRAM_WR_DATA_END = 0x03C,

    ID_REV = 0x050,
    IRQ_CFG = 0x054,
    INT_STS = 0x058,
    INT_EN = 0x05C,
    BYTE_TEST = 0x064,
    HW_CFG = 0x074,
    PMT_CTRL = 0x084,
    GPT_CFG = 0x08C,
    GPT_CNT = 0x090,
    FREE_RUN = 0x09C,

    RESET_CTL = 0x1F8,

    ECAT_CSR_DATA = 0x300,
    ECAT_CSR_CMD = 0x304,
    ECAT_PRAM_RD_ADDR_LEN = 0x308,
    ECAT_PRAM_RD_CMD = 0x30C,
    ECAT_PRAM_WR_ADDR_LEN = 0x310,
    ECAT_PRAM_WR_CMD = 0x314,

    ECAT_EEPROM_CONTROL_STATUS = 0x502,
    ECAT_EEPROM_ADDR = 0x504,
    ECAT_EEPROM_DATA = 0x508,   
}

// LAN9252 flags
pub const ECAT_CSR_BUSY: u8 = 0x80;
pub const PRAM_ABORT: u32 = 0x40000000;
pub const PRAM_BUSY: u8 = 0x80;
pub const PRAM_AVAIL: u8 = 0x01;
pub const READY: u8 = 0x08;
pub const DIGITAL_RST: u32 = 0x00000001;

//EtherCAT flags

pub const ALEVENT_CONTROL: u16 = 0x0001;
pub const ALEVENT_SM: u16 = 0x0010;

//state machine

pub const ESM_INIT: u8 = 0x01; // state machine control
pub const ESM_PREOP: u8 = 0x02; // (state request)
pub const ESM_BOOT: u8 = 0x03; //
pub const ESM_SAFEOP: u8 = 0x04; // safe-operational
pub const ESM_OP: u8 = 0x08; // operational

// SPI Command
pub const SPI_READ: u8 = 0x03;
pub const SPI_WRITE: u8 = 0x02;

// ESC Command
pub const ESC_WRITE: u8 = 0x80;
pub const ESC_READ: u8 = 0xC0;

// AL

pub const AL_CONTROL: u16 = 0x0120; // AL control
pub const AL_STATUS: u16 = 0x0130; // AL status
pub const AL_STATUS_CODE: u16 = 0x0134; // AL status code
pub const AL_EVENT: u16 = 0x0220; // AL event request
pub const AL_EVENT_MASK: u16 = 0x0204; // AL event interrupt mask

pub const WDOG_STATUS: u16 = 0x0440; // watch dog status

pub const SM0_BASE: u16 = 0x0800; // SM0 base address (output)
pub const SM1_BASE: u16 = 0x0808; // SM1 base address (input)

pub struct EthercatConfig<T, SCK, MOSI, MISO, CS>
where
    T: spi::Instance,
    SCK: spi::SckPin<T>,
    MOSI: spi::MosiPin<T>,
    MISO: spi::MisoPin<T>,
    CS: Pin,
{
    pub peri: T,
    pub sck: SCK,
    pub mosi: MOSI,
    pub miso: MISO,
    pub cs: CS,
}

pub struct Lan9252<'d, T, P>
where
    T: spi::Instance,
    P: Pin,
{
    spi: SpiDeviceWithConfig<
        'd,
        NoopRawMutex,
        spi::Spi<'static, T, NoDma, NoDma>,
        Output<'static, P>,
    >,
    data_buffer: [u8; 256],
}

impl<'d, T, P> Lan9252<'d, T, P>
where
    T: spi::Instance,
    P: Pin,
{
    pub fn new(
        spi: SpiDeviceWithConfig<
            'd,
            NoopRawMutex,
            spi::Spi<'static, T, NoDma, NoDma>,
            Output<'static, P>,
        >,
    ) -> Self {
        Self {
            spi,
            data_buffer: [0; 256],
        }
    }

    pub fn lan9252_transmit_raw_data(
        &mut self,
        write_bit: bool,
        addr: u16,
        data: &[u8],
    ) -> Result<&[u8], embassy_stm32::spi::Error> {
        // Building the array
        let mut instr: u8 = 0;
        if write_bit {
            instr = SPI_WRITE; //Write command
        } else {
            instr = SPI_READ; //read command
        }
        let addr8 = addr.to_le_bytes();
        self.data_buffer[0] = instr;
        self.data_buffer[1] = addr8[1];
        self.data_buffer[2] = addr8[0];

        self.data_buffer[3..(data.len() + 3)].copy_from_slice(data);

        // Sending data
        self.spi
            .transfer_in_place(&mut self.data_buffer[0..(3 + data.len())])
            .map_err(|e| {
                error!("!!! Error SPI {:?}!!!", e);
                embassy_stm32::spi::Error::Framing
            })?;

        let ret_data = &self.data_buffer[3..data.len() + 3];
        Ok(ret_data)
    }

    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        let mut reset_data = [0x00u8, 0x00u8, 0x00u8, 0x01u8];

        //Write RESET
        self.lan9252_transmit_raw_data(true, Lan9252Registers::RESET_CTL as u16, &reset_data)?;
        Timer::after(Duration::from_millis(100)).await;

        // Read back RESET
        let reset_state =
            self.lan9252_transmit_raw_data(false, Lan9252Registers::RESET_CTL as u16, &reset_data)?;
        debug!("RESET STATE: {:#x}", reset_state);
        Timer::after(Duration::from_millis(100)).await;

        //Check HW_CFG for READY
        loop {
            self.lan9252_transmit_raw_data(true, Lan9252Registers::HW_CFG as u16, &reset_data)?;
            Timer::after(Duration::from_millis(100)).await;
            let ready_state = self.lan9252_transmit_raw_data(
                false,
                Lan9252Registers::HW_CFG as u16,
                &reset_data,
            )?;
            debug!("READY STATE: {:#x}", ready_state);
            if ready_state[3] == 0x08 {
                break;
            }
            Timer::after(Duration::from_secs(1)).await;
        }

        // Check test byte
        let byte_test_state =
            self.lan9252_transmit_raw_data(false, Lan9252Registers::BYTE_TEST as u16, &reset_data)?;
        debug!("BYTE_TEST: {:#x}", byte_test_state);

        Ok(())
    }

    pub async fn read_register_direct(
        &mut self,
        address: u16,
        len: usize,
    ) -> Result<&[u8], embassy_stm32::spi::Error> {
        let tmpdata: [u8; 255] = [0; 255]; //pfffff
        self.lan9252_transmit_raw_data(false, address, &tmpdata[0..len])
    }

    pub async fn write_register_direct(
        &mut self,
        address: u16,
        data: &[u8],
    ) -> Result<(), embassy_stm32::spi::Error> {
        self.lan9252_transmit_raw_data(true, address, data)?;
        Ok(())
    }

    pub async fn write_register_indirect(
        &mut self,
        address: u16,
        data: &[u8],
    ) -> Result<(), embassy_stm32::spi::Error> {
        self.lan9252_transmit_raw_data(true, Lan9252Registers::ECAT_CSR_DATA as u16, data)?;

        let cmd: [u8; 4] = [
            (address & 0x00ff) as u8,
            ((address & 0xff00) >> 8) as u8,
            data.len() as u8,
            ESC_WRITE,
        ];

        self.lan9252_transmit_raw_data(true, Lan9252Registers::ECAT_CSR_CMD as u16, &cmd)?;

        // Wait for completion
        let tmpdata: [u8; 4] = [0; 4];
        loop {
            let ret = self.lan9252_transmit_raw_data(
                false,
                Lan9252Registers::ECAT_CSR_CMD as u16,
                &tmpdata,
            )?;
            if (ret[3] & ECAT_CSR_BUSY) != ECAT_CSR_BUSY {
                break;
            }
            Timer::after(Duration::from_micros(1)).await;
        }

        Ok(())
    }

    pub async fn read_register_indirect(
        &mut self,
        address: u16,
        len: usize,
    ) -> Result<&[u8], embassy_stm32::spi::Error> {
        let cmd: [u8; 4] = [
            (address & 0x00ff) as u8,
            ((address & 0xff00) >> 8) as u8,
            len as u8,
            ESC_READ,
        ];
        self.lan9252_transmit_raw_data(true, Lan9252Registers::ECAT_CSR_CMD as u16, &cmd)?;
        // Wait for completion
        let tmpdata: [u8; 4] = [0; 4];
        loop {
            let ret = self.lan9252_transmit_raw_data(
                false,
                Lan9252Registers::ECAT_CSR_CMD as u16,
                &tmpdata,
            )?;

            if (ret[3] & ECAT_CSR_BUSY) != ECAT_CSR_BUSY {
                break;
            }
            Timer::after(Duration::from_micros(1)).await;
        }

        self.read_register_direct(Lan9252Registers::ECAT_CSR_DATA as u16, len)
            .await
    }


    pub async fn read_bytes_large(
        &mut self,
        len: usize,
        buffer: &mut [u8], // Caller provides a fixed-size buffer
        address: u16,
    ) -> Result<(), embassy_stm32::spi::Error> {
        const MAX_CHUNK_SIZE: usize = 64;
        let mut current_address = address;
        let mut remaining_len = len;
        let mut offset = 0;
    
        while remaining_len > 0 {
            let chunk_size = if remaining_len > MAX_CHUNK_SIZE {
                MAX_CHUNK_SIZE
            } else {
                remaining_len
            };
    
            match self.read_bytes(chunk_size, current_address).await {
                Ok(data) => {
                    buffer[offset..offset + chunk_size].copy_from_slice(data);
                    current_address += chunk_size as u16; // Increment address by the chunk size
                    offset += chunk_size;
                    remaining_len -= chunk_size;
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }
    
        Ok(())
    }

    // FIXME! Only if < 64 bytes read
    pub async fn read_bytes(
        &mut self,
        len: usize,
        address: u16,
    ) -> Result<&[u8], embassy_stm32::spi::Error> {
        // Abort pending transfer
        let tmp_data = [0x00u8, 0x00u8, 0x00u8, 0x40u8]; //bit 30 (PRAM_READ_ABORT)
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_RD_CMD as u16, &tmp_data)
            .await?;

        Timer::after(Duration::from_micros(1)).await;

        // align the data to 4 bytes
        let round_len = match len % 4 {
            0 => len,
            _ => len + 4 - (len % 4),
        };
        // Configure starting address and data length
        let data_size: &[u8] = &(round_len as u16).to_le_bytes();

        let addres_bytes = (address).to_be_bytes();
        let tmp_data = [addres_bytes[1], addres_bytes[0], data_size[0], data_size[1]]; //Data address: 0x00001000 | data.len() <<16 TODO: check we are in the range
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_RD_ADDR_LEN as u16, &tmp_data)
            .await?;

        // Start the READ

        let tmp_data = [0x00u8, 0x00u8, 0x00u8, 0x80u8]; //bit 31 (PRAM_READ_BUSY)
                                                         // self.lan9252_transmit_raw_data(true, Lan9252Registers::ECAT_PRAM_RD_CMD as u16, &tmp_data)?;
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_RD_CMD as u16, &tmp_data)
            .await?;

        Timer::after(Duration::from_micros(1)).await;

        // LAN9252 Data Sheet page 220
        // 12.13.6 ETHERCAT PROCESS RAM READ COMMAND REGISTER (ECAT_PRAM_RD_CMD)
        // this register says how many DWORD (4-bytes) can be read
        // each time the register ECAT_PRAM_RD_DATA_START is read, the value in
        // ECAT_PRAM_RD_CMD is decremented by by the numeb of DWORDs read
        // this loop waits until all the data is available in the PRAM_RD_DATA_START register (ECAT_PRAM_RD_CMD count is len/4)
        loop {
            let ret = self
                .read_register_direct(Lan9252Registers::ECAT_PRAM_RD_CMD as u16, 2)
                .await?;
            // debug!("READ RET: {:?}", ret);
            if ret[1] == ((round_len as u8) / 4) {
                //
                break;
            }
            Timer::after(Duration::from_micros(1)).await;
        }

        match self
            .read_register_direct(Lan9252Registers::ECAT_PRAM_RD_DATA_START as u16, round_len)
            .await
        {
            Ok(data) => {
                Ok(&data[0..len]) // return only the requested data (not the padding)
            }
            Err(e) => Err(e),
        }
    }


    // writing the data to the memory 
    pub async fn write_bytes_large(
        &mut self,
        data: &[u8],
        address: u16,
    ) -> Result<(), embassy_stm32::spi::Error> {
        const MAX_CHUNK_SIZE: usize = 64;
        let mut current_address = address;
        let mut remaining_len = data.len();
        let mut offset = 0;
    
        while remaining_len > 0 {
            let chunk_size = if remaining_len > MAX_CHUNK_SIZE {
                MAX_CHUNK_SIZE
            } else {
                remaining_len
            };
    
            // Write each chunk of data
            match self.write_bytes(&data[offset..offset + chunk_size], current_address).await {
                Ok(()) => {
                    current_address += chunk_size as u16; // Increment address by the chunk size
                    offset += chunk_size;
                    remaining_len -= chunk_size;
                }
                Err(e) => {
                    return Err(e); // Return error if writing fails
                }
            }
        }
    
        Ok(())
    }

    //TODO Only if data.len()<=64
    pub async fn write_bytes(
        &mut self,
        data: &[u8],
        address: u16,
    ) -> Result<(), embassy_stm32::spi::Error> {
        // Abort pending transfer

        let tmp_data = [0x00u8, 0x00u8, 0x00u8, 0x40u8]; //bit 30 (PRAM_READ_ABORT)
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_WR_CMD as u16, &tmp_data)
            .await?;

        Timer::after(Duration::from_micros(1)).await;

        // align the data to 4 bytes
        let round_length = match data.len() % 4 {
            0 => data.len(),
            _ => data.len() + 4 - (data.len() % 4),
        };
        let mut data_round = [0; 64]; // TODO use round_length someehow
        data_round[0..data.len()].copy_from_slice(data);

        // Configure starting address and data length
        let data_size: &[u8] = &(round_length as u16).to_le_bytes();

        // page 221
        // chapter 12.13.7 ETHERCAT PROCESS RAM WRITE ADDRESS AND LENGTH REGISTER (ECAT_PRAM_WR_ADDR_LEN)
        // writing the data length to the register as well as the starting address (rage 1000h to 1FFFh)
        let addres_bytes = (address).to_be_bytes();
        let tmp_data = [addres_bytes[1], addres_bytes[0], data_size[0], data_size[1]]; //Data address: 0x00001000 | data.len() <<16 TODO: check we are in the range
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_WR_ADDR_LEN as u16, &tmp_data)
            .await?;

        // Start the WRITE
        // page 222
        // chapter 12.13.8 ETHERCAT PROCESS RAM WRITE COMMAND REGISTER (ECAT_PRAM_WR_CMD)
        // setting the PRAM_WRITE_BUSY bit to 1 (bit 31)
        let tmp_data = [0x00u8, 0x00u8, 0x00u8, 0x80u8]; //bit 31 (PRAM_READ_BUSY)
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_WR_CMD as u16, &tmp_data)
            .await?;

        Timer::after(Duration::from_micros(1)).await;

        //TODO?
        loop {
            let ret = self
                .read_register_direct(Lan9252Registers::ECAT_PRAM_WR_CMD as u16, 2)
                .await?;
            // debug!("WRITE RET: {:?}", ret);
            if ret[1] >= ceil((data.len() as f64) / 4.0) as u8 {
                break;
            }
            Timer::after(Duration::from_micros(1)).await;
        }

        // write the data
        // data length should not be more than 64 bytes
        // otherwise, the data needs to be divided into 64 bytes chunks
        self.write_register_direct(
            Lan9252Registers::ECAT_PRAM_WR_DATA_START as u16,
            &data_round,
        )
        .await
    }
}
