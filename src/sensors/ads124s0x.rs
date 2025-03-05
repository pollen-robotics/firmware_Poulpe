use crate::utils::errors::{IOError, Result};

use core::cell::RefCell;
use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::can::bxcan::filter;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Speed};
use embassy_stm32::gpio::{Output, Pin};
use embassy_stm32::spi::{Instance, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embedded_hal_1::spi::SpiDevice;

#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xCommand {
    // SPI Control Commands
    NOP = 0x00,
    WAKEUP = 0x02,
    POWERDOWN = 0x04,
    RESET = 0x06,
    START = 0x08,
    STOP = 0x0A,
    // SPI Calibration Commands
    SYOCAL = 0x16,
    SYGCAL = 0x17,
    SFOCAL = 0x19,
    // SPI Data Read Command
    RDATA = 0x12,
    // SPI Register Read and Write Commands
    RREG = 0x20,
    WREG = 0x40,
}
#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xRegister {
    ID = 0x00,
    STATUS = 0x01,
    INPMUX = 0x02,
    PGA = 0x03,
    DATARATE = 0x04,
    REF = 0x05,
    IDACMAG = 0x06,
    IDACMUX = 0x07,
    VBIAS = 0x08,
    SYS = 0x09,
    OFCAL0 = 0x0A,
    OFCAL1 = 0x0B,
    OFCAL2 = 0x0C,
    FSCAL0 = 0x0D,
    FSCAL1 = 0x0E,
    FSCAL2 = 0x0F,
    GPIODAT = 0x10,
    GPIOCON = 0x11,
}

#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xMuxChannel {
    AIN0 = 0b0000, // default
    AIN1 = 0b0001,
    AIN2 = 0b0010,
    AIN3 = 0b0011,
    AIN4 = 0b0100,
    AIN5 = 0b0101,
    AIN6 = 0b0110,
    AIN7 = 0b0111,
    AIN8 = 0b1000,
    AIN9 = 0b1001,
    AIN10 = 0b1010,
    AIN11 = 0b1011,
    AINCOM = 0b1100,
}

#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xGain {
    Gain1 = 0b0000, // default
    Gain2 = 0b0001,
    Gain4 = 0b0010,
    Gain8 = 0b0011,
    Gain16 = 0b0100,
    Gain32 = 0b0101,
    Gain64 = 0b0110,
    Gain128 = 0b0111,
}

#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xMode {
    Continuous = 0b0, // default
    SingleShot = 0b1,
}

#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xFilter {
    Sinc3 = 0b0,
    LowLatency = 0b1, // default
}

#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xDataRate {
    DR2SPS = 0b0000, // default
    DR5SPS = 0b0001,
    DR10SPS = 0b0010,
    DR16SPS = 0b0011,
    DR20SPS = 0b0100,
    DR50SPS = 0b0101,
    DR60SPS = 0b0110,
    DR100SPS = 0b0111,
    DR200SPS = 0b1000,
    DR400SPS = 0b1001,
    DR800SPS = 0b1010,
    DR1000SPS = 0b1011,
    DR2000SPS = 0b1100,
    DR4000SPS = 0b1101,
}

#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xRef {
    Ref0 = 0b00, // default
    Ref1 = 0b01,
    Ref2V5 = 0b10
}

#[derive(Debug, Format, Copy, Clone)]
#[repr(u8)]
pub enum Ads124s0xInternalRefConf {
    IntRefOff = 0b00, // default^
    IntRefPD = 0b01, // On but Off when ADC is power-down
    IntRefOn = 0b10, // Always On
}


pub struct ADS124S0x<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    pub spi:
        SpiDeviceWithConfig<'d, NoopRawMutex, Spi<'static, T, NoDma, NoDma>, Output<'static, Cs>>,
}


impl<'d, T, Cs> ADS124S0x<'d, T, Cs>
where
    T: Instance,
    Cs: Pin,
{
    pub fn new(
        spi: SpiDeviceWithConfig<
            'd,
            NoopRawMutex,
            Spi<'static, T, NoDma, NoDma>,
            Output<'static, Cs>,
        >,
    ) -> Self {
        Self { spi }
    }


    // Send command
    // Read Single Reg
    // Write Single Reg
    // Read Data

    pub fn setup(&mut self) -> Result<()> {
        self.reset()
    }

    fn send_command(&mut self, reg: Ads124s0xCommand) -> Result<()> {
        let mut data_reg = [(reg as u8)];
        match self.spi.transfer_in_place(&mut data_reg) {
            Ok(_) => {},
            Err(e) => {
                error!("Error: error sending command ADS124S0x (reg {=u8:#x})", reg as u8);
                return Err(IOError::CommunicationError);
            }
        };
        Ok(())
    }

    pub fn nop(&mut self) -> Result<()> {
        self.send_command(Ads124s0xCommand::NOP)
    }

    pub fn wakeup(&mut self) -> Result<()> {
        self.send_command(Ads124s0xCommand::WAKEUP)
    }

    pub fn powerdown(&mut self) -> Result<()> {
        self.send_command(Ads124s0xCommand::POWERDOWN)
    }

    pub fn reset(&mut self) -> Result<()> {
        self.send_command(Ads124s0xCommand::RESET)
    }

    pub fn start(&mut self) -> Result<()> {
        self.send_command(Ads124s0xCommand::START)
    }

    pub fn stop(&mut self) -> Result<()> {
        self.send_command(Ads124s0xCommand::STOP)
    }

    fn read_single_reg(&mut self, reg: Ads124s0xRegister) -> Result<u8> {
        let mut data_reg = [(reg as u8) | 0b00100000, 0];
        match self.spi.transfer_in_place(&mut data_reg) {
            Ok(_) => {}
            Err(e) => {
                error!("Error: error reading ADS124S0x (reg {=u8:#x})", reg as u8);
                return Err(IOError::CommunicationError);
            }
        }
        Ok(data_reg[1])
    }

    pub fn read_reg_id(&mut self) -> Result<u8> {
        self.read_single_reg(Ads124s0xRegister::ID)
    }

    pub fn read_reg_status(&mut self) -> Result<u8> {
        self.read_single_reg(Ads124s0xRegister::STATUS)
    }

    pub fn read_status_ok(&mut self) -> Result<bool> {
        self.read_reg_status().map(|status| (status & (1 << 6)) == 0)
    }
    
    pub fn read_reg_mux(&mut self) -> Result<u8> {
        self.read_single_reg(Ads124s0xRegister::INPMUX)
    }

    pub fn read_reg_pga(&mut self) -> Result<u8> {
        self.read_single_reg(Ads124s0xRegister::PGA)
    }

    pub fn read_reg_datarate(&mut self) -> Result<u8> {
        self.read_single_reg(Ads124s0xRegister::DATARATE)
    }

    pub fn read_reg_reference(&mut self) -> Result<u8> {
        self.read_single_reg(Ads124s0xRegister::REF)
    }

    pub fn read_reg_sys(&mut self) -> Result<u8> {
        self.read_single_reg(Ads124s0xRegister::SYS)
    }

    pub fn read_adc_data(&mut self) -> Result<u32> {
        let mut data_adc = [(Ads124s0xCommand::RDATA as u8), 0x00, 0x00, 0x00];
        match self.spi.transfer_in_place(&mut data_adc) {
            Ok(_) => {}
            Err(e) => {
                error!("Error: error reading ADC data");
                return Err(IOError::CommunicationError);
            }
        }
        let mut adc_value = 0 as u32
            | ((data_adc[1] as u32) << 16)
            | ((data_adc[2] as u32) << 8)
            |  (data_adc[3] as u32);
        Ok((adc_value))
    }


    fn write_single_reg(&mut self, addr: Ads124s0xRegister, data: u8) -> Result<()> {
        let mut msg_ads = [(addr as u8) | Ads124s0xCommand::WREG as u8, 0x01, data]; // add, nb, data
        match self.spi.transfer_in_place(&mut msg_ads) {
            Ok(_) => {}
            Err(_) => {
                error!("Error: error writing ADS124S0x (reg {=u8:#x})", addr as u8);
                return Err(IOError::CommunicationError);
            }
        }
        let val = self.read_single_reg(addr)?;
        if val != data {
            error!("Error: error verifying ADS124S0x configuration");
            return Err(IOError::InitError);
        }
        Ok(())
    }

    pub fn clear_power_on_reset(&mut self) -> Result<()> {
        let status = self.read_reg_status()?;
        let msg_force = status & 0b01111111;
        self.write_single_reg(Ads124s0xRegister::STATUS, msg_force)?;
        Ok(())
    }

    pub fn set_mux_channels(&mut self, muxp: Ads124s0xMuxChannel, muxn: Ads124s0xMuxChannel) -> Result<()> {
        let mut mux_reg = (muxp as u8) << 4;
        mux_reg += muxn as u8;
        self.write_single_reg(Ads124s0xRegister::INPMUX, mux_reg);
        Ok(())
    }

    pub fn set_pga_gain(&mut self, gain: Ads124s0xGain) -> Result<()> {
        let mut pga = self.read_reg_pga().unwrap();
        pga &= 0b1111_1000; // clear
        pga |= gain as u8; // set
        self.write_single_reg(Ads124s0xRegister::PGA, pga);
        Ok(())
    }

    pub fn enable_pga(&mut self) -> Result<()> {
        let mut pga = self.read_reg_pga().unwrap();
        pga &= 0b1110_0111; // clear
        pga |= 0b0000_1000; // set (01)
        self.write_single_reg(Ads124s0xRegister::PGA, pga);
        Ok(())
    }

    pub fn disable_pga(&mut self) -> Result<()> {
        let mut pga = self.read_reg_pga().unwrap();
        pga &= 0b1110_0111; // clear (00)
        self.write_single_reg(Ads124s0xRegister::PGA, pga);
        Ok(())
    }

    pub fn set_convertion_mode(&mut self, mode: Ads124s0xMode) -> Result<()> {
        let mut reg_dr = self.read_reg_datarate().unwrap();
        match mode {
            Ads124s0xMode::Continuous => reg_dr &= 0b0001_0000, // clear b5
            Ads124s0xMode::SingleShot => reg_dr |= 0b0001_0000, // set b5
        }
        self.write_single_reg(Ads124s0xRegister::DATARATE, reg_dr);
        Ok(())
    }

    pub fn set_filter_type(&mut self, filter: Ads124s0xFilter) -> Result<()> {
        let mut reg_dr = self.read_reg_datarate().unwrap();
        match filter {
            Ads124s0xFilter::Sinc3 => reg_dr &= 0b0000_1000, // clear b4
            Ads124s0xFilter::LowLatency => reg_dr |= 0b0000_1000, // set b4
        }
        self.write_single_reg(Ads124s0xRegister::DATARATE, reg_dr);
        Ok(())
    }

    pub fn set_datarate(&mut self, dr: Ads124s0xDataRate) -> Result<()> {
        let mut reg_dr = self.read_reg_datarate().unwrap();
        reg_dr &= 0b0000_1111; // clear
        reg_dr |= (dr as u8); // set (01)
        self.write_single_reg(Ads124s0xRegister::DATARATE, reg_dr);
        Ok(())
    }

    pub fn select_reference(&mut self, ref_v: Ads124s0xRef) -> Result<()> {
        let mut reg_ref = self.read_reg_reference().unwrap();
        match ref_v {
            Ads124s0xRef::Ref0 => reg_ref &= 0b0000_0011,
            Ads124s0xRef::Ref1 => { // 01
                reg_ref &= 0b0000_0010;
                reg_ref |= 0b0000_0001;
            }
            Ads124s0xRef::Ref2V5 => { // 10
                reg_ref &= 0b0000_0001;
                reg_ref |= 0b0000_0010;
            }
        }
        self.write_single_reg(Ads124s0xRegister::DATARATE, reg_ref);
        Ok(())
    }

    pub fn config_ref(&mut self, ref_conf: Ads124s0xInternalRefConf) -> Result<()> {
        let mut reg_ref = self.read_reg_reference().unwrap();
        match ref_conf {
            Ads124s0xInternalRefConf::IntRefOff => reg_ref &= 0b0000_0011,
            Ads124s0xInternalRefConf::IntRefPD => { // 01
                reg_ref &= 0b0000_0010;
                reg_ref |= 0b0000_0001;
            }
            Ads124s0xInternalRefConf::IntRefOn => { // 10
                reg_ref &= 0b0000_0001;
                reg_ref |= 0b0000_0010;
            }
        }
        self.write_single_reg(Ads124s0xRegister::DATARATE, reg_ref);
        Ok(())
    }



    /*pub enum  {
    pub enum Ads124s0xInternalRefConf {*/
}
