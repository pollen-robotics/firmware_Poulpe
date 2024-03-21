use crate::{
    config::{self, LAN9252Config},
    motor_control::{BoardStatus, Pid},
    SHARED_MEMORY,
};
use core::cell::RefCell;
use defmt::{debug, error, trace};
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
        Self { spi }
    }

    pub(crate) fn lan9252_checked_write(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<(), embassy_stm32::spi::Error> {
        self.lan9252_write_register(reg, data_w)?;
        let data_r = self.lan9252_read_register(reg)?;
        if data_r == data_w {
            Ok(())
        } else {
            error!(
                "!!! LAN9252 Error checked write addr: {:#x} {:#x}_r / {:#x}_w !!!",
                reg, data_r, data_w
            );
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    fn lan9252_write_register(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        let data_m = data_w;
        self.lan9252_transmit_raw_data(true, reg, data_m)
    }

    fn lan9252_read_register(&mut self, reg: u8) -> Result<u32, embassy_stm32::spi::Error> {
        let data_m = 0x00000000u32;
        self.lan9252_transmit_raw_data(false, reg, data_m)
    }

    fn lan9252_transmit_raw_data(
        &mut self,
        write_bit: bool,
        addr: u8,
        data: u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        // Building the array
        let mut msb_data = addr;
        let mut data_u8_array = data.to_le_bytes();
        if write_bit {
            msb_data = addr | 0b10000000;
        } else {
            data_u8_array = [0x00u8; 4];
        }
        let mut transfer_data = [
            msb_data,
            data_u8_array[3],
            data_u8_array[2],
            data_u8_array[1],
            data_u8_array[0],
        ];

        // Sending data
        self.spi
            .transfer_in_place(&mut transfer_data)
            .map_err(|e| {
                error!("!!! Error SPI {:?}!!!", e);
                embassy_stm32::spi::Error::Framing
            })?;

        let mut read_data = transfer_data[4] as u32;
        read_data += (transfer_data[3] as u32) << 8;
        read_data += (transfer_data[2] as u32) << 16;
        read_data += (transfer_data[1] as u32) << 24;

        Ok(read_data)
    }
}

#[embassy_executor::task]
pub async fn messsage_handler(ethconf: LAN9252Config, spi_config: spi::Config) {
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
}
