use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Output, Pin};
use embassy_stm32::spi::{Instance, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal_1::spi::SpiDevice;

use crate::utils::errors::DriverError;

pub trait Driver {
    fn configure(&mut self) -> Result<(), DriverError>;
    fn check_status(&mut self, is_enabled: bool) -> Result<(), DriverError>;
}

pub struct DriverTMC6200<'d, T, EnablePin, StatusPin>
where
    T: Instance,
    EnablePin: Pin,
    StatusPin: Pin,
{
    spi: SpiDeviceWithConfig<
        'd,
        NoopRawMutex,
        Spi<'static, T, NoDma, NoDma>,
        Output<'static, EnablePin>,
    >,
    pub(crate) status_pin: Input<'static, StatusPin>,
}

impl<'d, T, EnablePin, StatusPin> DriverTMC6200<'d, T, EnablePin, StatusPin>
where
    T: Instance,
    EnablePin: Pin,
    StatusPin: Pin,
{
    pub fn new(
        spi: SpiDeviceWithConfig<
            'd,
            NoopRawMutex,
            Spi<'static, T, NoDma, NoDma>,
            Output<'static, EnablePin>,
        >,
        status_pin: StatusPin,
    ) -> Self {
        let mut status_pin = Input::new(status_pin, embassy_stm32::gpio::Pull::None);
        Self { spi, status_pin }
    }

    pub fn checked_write(&mut self, reg: u8, data_w: u32) -> Result<(), embassy_stm32::spi::Error> {
        self.write_register(reg, data_w)?;
        let data_r = self.read_register(reg)?;
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

    fn write_register(&mut self, reg: u8, data_w: u32) -> Result<u32, embassy_stm32::spi::Error> {
        let data_m = data_w;
        self.transmit_raw_data(true, reg, &data_m)
    }

    fn read_register(&mut self, reg: u8) -> Result<u32, embassy_stm32::spi::Error> {
        let data_m = 0x00000000u32;
        self.transmit_raw_data(false, reg, &data_m)
    }

    fn transmit_raw_data(
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

impl<'d, T, EnablePin, StatusPin> Driver for DriverTMC6200<'d, T, EnablePin, StatusPin>
where
    T: Instance,
    EnablePin: Pin,
    StatusPin: Pin,
{
    fn configure(&mut self) -> Result<(), DriverError> {
        let mut ret_err = false;

        // /!\ Please note that the TMC6200 must be in Single-line mode (aka 6-PMW)
        match self.checked_write(0x00u8, 0x00000000u32) {
            Ok(_) => {
                debug!("TMC6200 setting 6-PWM set");
            }
            Err(e) => {
                ret_err = true;
                error!(
                    "TMC6200 setting 6-PWM failed: {:?} => check SPI and power connection",
                    e
                );
            }
        };
        // BOB configuration - this was the config for beta hardware (migth be changed)
        // DRVSRENGTH=2
        // BBMCLKS=2
        match self.checked_write(0x0au8, 0x00000000u32) {
            Ok(_) => {
                debug!("TMC6200 setting DRVSRENGTH set");
            }
            Err(e) => {
                ret_err = true;
                error!(
                    "TMC6200 setting DRVSRENGTH failed: {:?} => check SPI and power connection",
                    e
                );
            }
        };
        if ret_err {
            return Err(DriverError::ConfigError);
        } else {
            return Ok(());
        }
    }

    fn check_status(&mut self, _is_enabled: bool) -> Result<(), DriverError> {
        // if not gamma, then return Ok
        #[cfg(any(feature = "gamma", feature = "pvt"))]
        {
            return Ok(());
        }
        // high on error
        if self.status_pin.is_low() {
            return Ok(());
        } else {
            return Err(DriverError::FaultState);
        }
    }
}

pub struct DriverDRV8316<'d, T, EnablePin, StatusPin>
where
    T: Instance,
    EnablePin: Pin,
    StatusPin: Pin,
{
    spi: SpiDeviceWithConfig<
        'd,
        NoopRawMutex,
        Spi<'static, T, NoDma, NoDma>,
        Output<'static, EnablePin>,
    >,
    pub(crate) status_pin: Input<'static, StatusPin>,
}

impl<'d, T, EnablePin, StatusPin> DriverDRV8316<'d, T, EnablePin, StatusPin>
where
    T: Instance,
    EnablePin: Pin,
    StatusPin: Pin,
{
    pub fn new(
        spi: SpiDeviceWithConfig<
            'd,
            NoopRawMutex,
            Spi<'static, T, NoDma, NoDma>,
            Output<'static, EnablePin>,
        >,
        status_pin: StatusPin,
    ) -> Self {
        let mut status_pin = Input::new(status_pin, embassy_stm32::gpio::Pull::None);
        Self { spi, status_pin }
    }

    pub fn checked_write(&mut self, reg: u8, data_w: u8) -> Result<(), embassy_stm32::spi::Error> {
        self.write_register(reg, data_w)?;
        let data_r = self.read_register(reg)?;
        if data_w == data_r[1] {
            Ok(())
        } else {
            info!(
                "!!! DRV8316: Error checked write addr {:#x} {:#x}_r / {:#x}_w !!!",
                reg, data_r[1], data_w
            );
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    fn write_register(
        &mut self,
        reg: u8,
        data_w: u8,
    ) -> Result<[u8; 2], embassy_stm32::spi::Error> {
        let data_m = data_w;
        self.transmit_raw_data(true, reg, data_m)
    }

    fn read_register(&mut self, reg: u8) -> Result<[u8; 2], embassy_stm32::spi::Error> {
        let data_m = 0x00u8;
        self.transmit_raw_data(false, reg, data_m)
    }

    fn transmit_raw_data(
        &mut self,
        write_bit: bool,
        addr: u8,
        data: u8,
    ) -> Result<[u8; 2], embassy_stm32::spi::Error> {
        // Building array
        let mut msb_data: u8 = addr << 1;
        let mut data = data;
        if write_bit == false {
            //if read
            msb_data |= 0b10000000;
            data = 0x00;
        }

        let mut transfer_data = ((msb_data as u16) << 8) | data as u16;
        // check parity
        if transfer_data.count_ones() % 2 != 0 {
            transfer_data |= 0x0100;
        }
        let mut transfer_data = transfer_data.to_be_bytes();

        // Sending data
        self.spi
            .transfer_in_place(&mut transfer_data)
            .map_err(|e| {
                error!("!!! DRV8316: Error SPI {:?}!!!", e);
                embassy_stm32::spi::Error::Framing
            })?;

        let mut read_data = [0x00u8; 2];
        read_data[0] = transfer_data[0];
        read_data[1] = transfer_data[1];

        Ok(read_data)
    }
}

impl<'d, T, EnablePin, StatusPin> Driver for DriverDRV8316<'d, T, EnablePin, StatusPin>
where
    T: Instance,
    EnablePin: Pin,
    StatusPin: Pin,
{
    fn configure(&mut self) -> Result<(), DriverError> {
        let mut ret_err = false;
        debug!("DRV8316: Configuring the driver");
        // unlock the registers in order to be able to write to them
        // write 0x3 (position 0-2) to register 0x3 (control register 1 pp. 62)
        let mut data: u8 = 0b011;
        match self.checked_write(0x3, data) {
            Ok(_) => {
                debug!("DRV8316: Registers unlocked");
            }
            Err(e) => {
                ret_err = true;
                error!("DRV8316: Could not unlock the registers: {:?}", e);
            }
        };

        // set the slew rate to the faster setting
        // setting it to 200V/us
        // write 0x3 (position 3-4) to register 0x4 (control register 2 pp. 63)
        data = 0x60 as u8 | 0b11 << 3;
        match self.checked_write(0x4, data) {
            Ok(_) => {
                debug!("DRV8316: Slew rate set!");
            }
            Err(e) => {
                ret_err = true;
                error!("DRV8316: Could not set the slew rate: {:?}", e);
            }
        };

        // set the gain
        // set gain to 0.3V/A
        // write 0x1 (position 0-1) to register 0x7 (control register 5 pp. 66)
        data = 0x00 as u8 | 0x1;
        match self.checked_write(0x7, data) {
            Ok(_) => {
                debug!("DRV8316: Gain set");
            }
            Err(e) => {
                ret_err = true;
                error!("DRV8316: Could not set the gain rate: {:?}", e);
            }
        };

        if ret_err {
            return Err(DriverError::ConfigError);
        } else {
            return Ok(());
        }
    }

    fn check_status(&mut self, is_enabled: bool) -> Result<(), DriverError> {
        // low on error (DRV8316)
        if self.status_pin.is_high() {
            return Ok(());
        } else {
            // only error if driver is enabled
            if is_enabled {
                return Err(DriverError::FaultState);
            } else {
                // if not enabled, then it is not an error
                return Ok(());
            }
        }
    }
}
