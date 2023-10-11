#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::peripherals as p;
use embassy_stm32::dma::NoDma;
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::gpio::{Level, Output, Speed};



pub struct TMC6200 {
    spi: Spi<'static, p::SPI4, NoDma, NoDma>,
    cs: Output<'static, p::PC15>
}

impl TMC6200 {
    pub fn new(
        cs_p: p::PC15,
        sck_p: p::PE12,
        miso_p: p::PE5,
        mosi_p: p::PE6,
        spi: p::SPI4,
        dma_rx: NoDma,
        dma_tx: NoDma,
    ) -> Self {
        let cs = Output::new(cs_p, Level::High, Speed::Medium);
        let mut cfg = Config::default();
        cfg.mode = embassy_stm32::spi::MODE_3;
        let spi = Spi::new(spi, sck_p, mosi_p, miso_p, dma_tx, dma_rx, cfg);

        Self { cs, spi }
    }

    pub fn transmit_raw_data(
        &mut self,
        write_bit: bool,
        addr: u8,
        data: &mut u32,
    ) -> Result<[u8; 4], embassy_stm32::spi::Error> {
        // Building array
        let mut msb_data = addr;
        if write_bit == true {
            msb_data = addr | 0b10000000;
        }
        let data_u8_array = data.to_le_bytes();
        let mut transfer_data = [msb_data, data_u8_array[0], data_u8_array[1], data_u8_array[2], data_u8_array[3]];
    
        // Sending data
        &mut self.cs.set_low();
        let _result = &mut self.spi.blocking_transfer_in_place(&mut transfer_data); // Todo: the error is not treated.
        &mut self.cs.set_high();
    
        let mut read_data = [0x00u8; 4];
        read_data[0] = transfer_data[1];
        read_data[1] = transfer_data[2];
        read_data[2] = transfer_data[3];
        read_data[3] = transfer_data[4];
    
        Ok(read_data)
    }

}
