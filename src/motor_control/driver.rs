use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Output, Pin};
use embassy_stm32::spi::{Instance, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal_1::spi::SpiDevice;

pub struct Driver<'d, T, P>
where
    T: Instance,
    P: Pin,
{
    spi: SpiDeviceWithConfig<'d, NoopRawMutex, Spi<'static, T, NoDma, NoDma>, Output<'static, P>>,
}

impl<'d, T, P> Driver<'d, T, P>
where
    T: Instance,
    P: Pin,
{
    pub fn new(
        spi: SpiDeviceWithConfig<
            'd,
            NoopRawMutex,
            Spi<'static, T, NoDma, NoDma>,
            Output<'static, P>,
        >,
    ) -> Self {
        Self { spi }
    }

    pub(crate) fn tmc6200_checked_write(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<(), embassy_stm32::spi::Error> {
        self.tmc6200_write_register(reg, data_w)?;
        let data_r = self.tmc6200_read_register(reg)?;
        if data_r == data_w {
            Ok(())
        } else {
            info!(
                "!!! TMC6200 Error checked write addr {:#x} {:#x}_r / {:#x}_w !!!",
                reg, data_r, data_w
            );
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    fn tmc6200_write_register(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        let data_m = data_w;
        self.tmc6200_transmit_raw_data(true, reg, &data_m)
    }

    fn tmc6200_read_register(&mut self, reg: u8) -> Result<u32, embassy_stm32::spi::Error> {
        let data_m = 0x00000000u32;
        self.tmc6200_transmit_raw_data(false, reg, &data_m)
    }

    fn tmc6200_transmit_raw_data(
        &mut self,
        write_bit: bool,
        addr: u8,
        data: &u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        // Building array
        let mut msb_data = addr;
        if write_bit {
            msb_data = addr | 0b10000000;
        }
        let data_u8_array = data.to_le_bytes();
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
