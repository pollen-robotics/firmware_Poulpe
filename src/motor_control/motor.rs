use embassy_stm32::spi::{Config, Instance, MisoPin, MosiPin, SckPin};
use embassy_stm32::{dma::NoDma, spi::Spi};
use embassy_stm32::{peripherals as p, Peripheral};

fn axis_1() {
    let cs_foc = unsafe { p::PE3::steal() };
    let cs_driver = unsafe { p::PC15::steal() };
    let sck = unsafe { p::PE12::steal() };
    let miso = unsafe { p::PE5::steal() };
    let mosi = unsafe { p::PE6::steal() };
    let peri: p::SPI4 = unsafe { p::SPI4::steal() };
    let rxdma = NoDma;
    let txdma = NoDma;
    let foc_enable = unsafe { p::PE0::steal() };
    let foc_status = unsafe { p::PC13::steal() };
    let driver_fault = unsafe { p::PC14::steal() };

    // let m = Motor::new(peri, sck, mosi, miso);

    Motor1::new(peri, sck, mosi, miso);
}

fn axis_2() {
    let cs_foc = unsafe { p::PD7::steal() };
    let cs_driver = unsafe { p::PD6::steal() };
    let sck = unsafe { p::PB3::steal() };
    let miso = unsafe { p::PB4::steal() };
    let mosi = unsafe { p::PB5::steal() };
    let peri = unsafe { p::SPI6::steal() };
    let rxdma = NoDma;
    let txdma = NoDma;
    let foc_enable = unsafe { p::PD5::steal() };
    let foc_status = unsafe { p::PD4::steal() };
    let driver_fault = unsafe { p::PD3::steal() };

    Motor2::new(peri, sck, mosi, miso);
}

type Motor1 = Motor<'static, p::SPI4>;
type Motor2 = Motor<'static, p::SPI6>;

pub struct Motor<'d, T: Instance> {
    spi: Spi<'d, T, NoDma, NoDma>,
}

impl<'d, T: Instance> Motor<'d, T> {
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
    ) -> Self {
        let config = Config::default();

        let txdma = NoDma;
        let rxdma = NoDma;

        let spi = Spi::new(peri, sck, mosi, miso, txdma, rxdma, config);

        Self { spi }
    }

    pub fn salut(&mut self) {
        let mut transfer_data: [u8; 2] = [0x00, 0x00];
        let _result = self.spi.blocking_transfer_in_place(&mut transfer_data);
    }
}
