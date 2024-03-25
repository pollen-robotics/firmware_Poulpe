use crate::{
    config::{self, LAN9252Config},
    motor_control::{BoardStatus, Pid},
    SHARED_MEMORY,
};
use core::cell::RefCell;
use defmt::{debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::gpio::AnyPin;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
};
use embassy_stm32::{gpio::Pin, spi};

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
}

// LAN9252 flags
const ECAT_CSR_BUSY: u8 = 0x80;
const PRAM_ABORT: u32 = 0x40000000;
const PRAM_BUSY: u8 = 0x80;
const PRAM_AVAIL: u8 = 0x01;
const READY: u8 = 0x08;
const DIGITAL_RST: u32 = 0x00000001;

//EtherCAT flags

const ALEVENT_CONTROL: u16 = 0x0001;
const ALEVENT_SM: u16 = 0x0010;

//state machine

const ESM_INIT: u8 = 0x01; // state machine control
const ESM_PREOP: u8 = 0x02; // (state request)
const ESM_BOOT: u8 = 0x03; //
const ESM_SAFEOP: u8 = 0x04; // safe-operational
const ESM_OP: u8 = 0x08; // operational

// SPI Command
const SPI_READ: u8 = 0x03;
const SPI_WRITE: u8 = 0x02;

// ESC Command
const ESC_WRITE: u8 = 0x80;
const ESC_READ: u8 = 0xC0;

// AL

const AL_CONTROL: u16 = 0x0120; // AL control
const AL_STATUS: u16 = 0x0130; // AL status
const AL_STATUS_CODE: u16 = 0x0134; // AL status code
const AL_EVENT: u16 = 0x0220; // AL event request
const AL_EVENT_MASK: u16 = 0x0204; // AL event interrupt mask

const WDOG_STATUS: u16 = 0x0440; // watch dog status

const SM0_BASE: u16 = 0x0800; // SM0 base address (output)
const SM1_BASE: u16 = 0x0808; // SM1 base address (input)

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
        let reset_data = [0x00u8, 0x00u8, 0x00u8, 0x01u8];

        //Write RESET
        self.lan9252_transmit_raw_data(true, Lan9252Registers::RESET_CTL as u16, &reset_data)?;
        Timer::after(Duration::from_micros(1)).await;

        // Read back RESET
        let reset_state =
            self.lan9252_transmit_raw_data(false, Lan9252Registers::RESET_CTL as u16, &reset_data)?;
        debug!("RESET STATE: {:#x}", reset_state);
        Timer::after(Duration::from_micros(1)).await;

        //Check HW_CFG for READY
        loop {
            let ready_state = self.lan9252_transmit_raw_data(
                false,
                Lan9252Registers::HW_CFG as u16,
                &reset_data,
            )?;
            debug!("READY STATE: {:#x}", ready_state);
            if ready_state[3] == 0x08 {
                break;
            }
            Timer::after(Duration::from_micros(1)).await;
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

    //FIXME! Only if <64Bytes
    pub async fn read_bytes(&mut self, len: usize) -> Result<&[u8], embassy_stm32::spi::Error> {
        // Abort pending transfer

        let tmp_data = [0x00u8, 0x00u8, 0x00u8, 0x40u8]; //bit 30 (PRAM_READ_ABORT)
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_RD_CMD as u16, &tmp_data)
            .await?;

        Timer::after(Duration::from_micros(1)).await;

        // Configure starting address and data length
        let data_size: &[u8] = &(len as u16).to_le_bytes();

        let tmp_data = [0x00u8, 0x10u8, data_size[0], data_size[1]]; //Data address: 0x00001000 | data.len() <<16 TODO: check we are in the range
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_RD_ADDR_LEN as u16, &tmp_data)
            .await?;

        // Start the READ

        let tmp_data = [0x00u8, 0x00u8, 0x00u8, 0x80u8]; //bit 31 (PRAM_READ_BUSY)
                                                         // self.lan9252_transmit_raw_data(true, Lan9252Registers::ECAT_PRAM_RD_CMD as u16, &tmp_data)?;
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_RD_CMD as u16, &tmp_data)
            .await?;

        Timer::after(Duration::from_micros(1)).await;

        //TODO?
        loop {
            let ret = self
                .read_register_direct(Lan9252Registers::ECAT_PRAM_RD_CMD as u16, 2)
                .await?;
            // debug!("READ RET: {:?}", ret);
            if ret[1] == (len as u8) / 4 {
                break;
            }
            Timer::after(Duration::from_micros(1)).await;
        }

        self.read_register_direct(Lan9252Registers::ECAT_PRAM_RD_DATA_START as u16, len)
            .await
    }

    //TODO Only if data.len()<=64
    pub async fn write_bytes(&mut self, data: &[u8]) -> Result<(), embassy_stm32::spi::Error> {
        // Abort pending transfer

        let tmp_data = [0x00u8, 0x00u8, 0x00u8, 0x40u8]; //bit 30 (PRAM_READ_ABORT)
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_WR_CMD as u16, &tmp_data)
            .await?;

        Timer::after(Duration::from_micros(1)).await;

        // Configure starting address and data length
        let data_size: &[u8] = &(data.len() as u16).to_le_bytes();

        let tmp_data = [0x00u8, 0x12u8, data_size[0], data_size[1]]; //Data address: 0x00001000 | data.len() <<16 TODO: check we are in the range
        self.write_register_direct(Lan9252Registers::ECAT_PRAM_WR_ADDR_LEN as u16, &tmp_data)
            .await?;

        // Start the WRITE

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
            if ret[1] >= (data.len() as u8) / 4 {
                break;
            }
            Timer::after(Duration::from_micros(1)).await;
        }

        self.write_register_direct(Lan9252Registers::ECAT_PRAM_WR_DATA_START as u16, data)
            .await
    }
}

#[embassy_executor::task]
pub async fn messsage_handler(ethconf: LAN9252Config, spi_config: spi::Config) {
    warn!("ETHERCAT TASK");

    let spi = spi::Spi::new(
        ethconf.peri,
        ethconf.sck,
        ethconf.mosi,
        ethconf.miso,
        NoDma,
        NoDma,
        spi_config,
    );
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));

    let eth_spi = SpiDeviceWithConfig::new(
        &spi_bus,
        Output::new(ethconf.cs, Level::High, Speed::Medium),
        spi_config,
    );

    let mut lan9252 = Lan9252::new(eth_spi);
    match lan9252.init().await {
        Ok(_) => {
            debug!("LAN9252 init done");
        }
        Err(e) => {
            error!("LAN8252 init error: {:?}", e);
        }
    }
    Timer::after(Duration::from_micros(1)).await;

    match lan9252
        .read_register_direct(Lan9252Registers::BYTE_TEST as u16, 4)
        .await
    {
        Ok(test) => {
            info!("Byte test: {:#x}", test)
        }
        Err(e) => {
            error!("Read test error {:?}", e)
        }
    }
    Timer::after(Duration::from_micros(1)).await;

    match lan9252.read_register_indirect(WDOG_STATUS, 1).await {
        Ok(wdg) => {
            info!("Watchdog: {:#x}", wdg[0])
        }
        Err(e) => {
            error!("Read watchdog error {:?}", e)
        }
    }
    Timer::after(Duration::from_micros(1)).await;

    match lan9252.read_register_indirect(AL_STATUS, 1).await {
        Ok(al) => {
            info!("Status: {:#x}", al[0] & 0x0F)
        }
        Err(e) => {
            error!("Read status error {:?}", e)
        }
    }
    Timer::after(Duration::from_micros(1)).await;

    // let mut read_data: &[u8] = &[0, 0, 0, 0];

    let mut test_cnt: u32 = 0;
    loop {
        match lan9252.read_register_indirect(AL_STATUS, 1).await {
            Ok(al) => {
                info!("Status: {:#x}", al[0] & 0x0F)
            }
            Err(e) => {
                error!("Read status error {:?}", e)
            }
        }
        Timer::after(Duration::from_micros(1)).await;

        let ret = lan9252.read_bytes(4).await;
        info!("DATA: {:?}", ret);

        match lan9252.write_bytes(&test_cnt.to_le_bytes()).await {
            Ok(_) => {}
            Err(e) => {
                error!("Write data error! {:?}", e)
            }
        }

        test_cnt = test_cnt.wrapping_add(1);
        Timer::after(Duration::from_millis(1000)).await;
    }
}
