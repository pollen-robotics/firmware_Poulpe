use crate::{
    config::{self, LAN9252Config},
    motor_control::{BoardStatus, Pid},
    SHARED_MEMORY,
};
use defmt::{debug, error, trace};
use embassy_stm32::gpio::AnyPin;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
};
use embassy_stm32::{gpio::Pin, spi};
use embassy_time::{Duration, Instant, Timer};

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

// impl<'d, T, SCK, MOSI, MISO, CS> Ethercat<'d, T, SCK, MOSI, MISO, CS>
// where
//     T: spi::Instance,
//     SCK: spi::SckPin<T>,
//     MOSI: spi::MosiPin<T>,
//     MISO: spi::MisoPin<T>,
//     CS: Pin,
// {
//     pub fn new(spi: T, sck: SCK, mosi: MOSI, miso: MISO, cs: CS, spi_config: spi::Config) -> Self {
//         Self {
//             spi: spi::Spi::new(spi, sck, mosi, miso, NoDma, NoDma, spi_config),
//             CS: Output::new(cs, Level::High, Speed::Low),
//         }
//     }
// }

#[embassy_executor::task]
pub async fn messsage_handler(ethconf: LAN9252Config, spi_config: spi::Config) {
    // let lan9252 = Ethercat::new(spi, sck, mosi, miso, cs, spi_config);
    let mut spi = spi::Spi::new(
        ethconf.peri,
        ethconf.sck,
        ethconf.mosi,
        ethconf.miso,
        NoDma,
        NoDma,
        spi_config,
    );
    let mut cs = Output::new(ethconf.cs, Level::High, Speed::Low);
}
