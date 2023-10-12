#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::peripherals as p;
use embassy_stm32::dma::NoDma;
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::*; // TODO : to be remove (delays for testing purposes only)

use crate::tmc4671;

pub enum Tmc4671Registers {
    CHIPINFO_DATA = 0x00,
    CHIPINFO_ADDR = 0x01,
    ADC_RAW_ADDR = 0x03,
    dsADC_MCFG_B_MCFG_A = 0x04,
    dsADC_MCLK_A = 0x05,
    dsADC_MCLK_B = 0x06,
    dsADC_MDEC_B_MDEC_A = 0x07,
    ADC_I1_SCALE_OFFSET = 0x08,
    ADC_I0_SCALE_OFFSET = 0x09,
    ADC_I_SELECT = 0x0A,
    ADC_I1_I0_EXT = 0x0B,
    DS_ANALOG_INPUT_STAGE_CFG = 0x0C,
    AENC_0_SCALE_OFFSET = 0x0D,
    AENC_1_SCALE_OFFSET = 0x0E,
    AENC_2_SCALE_OFFSET = 0x0F,
    AENC_SELECT = 0x11,
    PWM_POLARITIES = 0x17,
    PWM_MAXCNT = 0x18,
    PWM_BBM_H_BBM_L = 0x19,
    PWM_SV_CHOP = 0x1A,
    MOTOR_TYPE_N_POLE_PAIRS = 0x1B,
    PHI_E_EXT = 0x1C,
    OPENLOOP_MODE = 0x1F,
    OPENLOOP_ACCELERATION = 0x20,
    OPENLOOP_VELOCITY_TARGET = 0x21,
    OPENLOOP_VELOCITY_ACTUAL = 0x22,
    OPENLOOP_PHI = 0x23,
    UQ_UD_EXT = 0x24,
    ABN_DECODER_MODE = 0x25,
    ABN_DECODER_PPR = 0x26,
    ABN_DECODER_COUNT = 0x27,
    ABN_DECODER_COUNT_N = 0x28,
    ABN_DECODER_PHI_E_PHI_M_OFFSET = 0x29,
    ABN_2_DECODER_MODE = 0x2C,
    ABN_2_DECODER_PPR = 0x2D,
    ABN_2_DECODER_COUNT = 0x2E,
    ABN_2_DECODER_COUNT_N = 0x2F,
    ABN_2_DECODER_PHI_M_OFFSET = 0x30,
    HALL_MODE = 0x33,
    HALL_POSITION_060_000 = 0x34,
    HALL_POSITION_180_120 = 0x35,
    HALL_POSITION_300_240 = 0x36,
    HALL_PHI_E_PHI_M_OFFSET = 0x37,
    HALL_DPHI_MAX = 0x38,
    AENC_DECODER_MODE = 0x3B,
    AENC_DECODER_N_THRESHOLD = 0x3C,
    AENC_DECODER_PHI_A_OFFSET = 0x3E,
    AENC_DECODER_PPR = 0x40,
    AENC_DECODER_COUNT_N = 0x42,
    AENC_DECODER_PHI_E_PHI_M_OFFSET = 0x45,
    CONFIG_DATA = 0x4D,
    CONFIG_ADDR = 0x4E,
    VELOCITY_SELECTION = 0x50,
    POSITION_SELECTION = 0x51,
    PHI_E_SELECTION = 0x52,
    PID_FLUX_P_FLUX_I = 0x54,
    PID_TORQUE_P_TORQUE_I = 0x56,
    PID_VELOCITY_P_VELOCITY_I = 0x58,
    PID_POSITION_P_POSITION_I = 0x5A,
    PIDOUT_UQ_UD_LIMITS = 0x5D,
    PID_TORQUE_FLUX_LIMITS = 0x5E,
    PID_VELOCITY_LIMIT = 0x60,
    PID_POSITION_LIMIT_LOW = 0x61,
    PID_POSITION_LIMIT_HIGH = 0x62,
    MODE_RAMP_MODE_MOTION = 0x63,
    PID_TORQUE_FLUX_TARGET = 0x64,
    PID_TORQUE_FLUX_OFFSET = 0x65,
    PID_VELOCITY_TARGET = 0x66,
    PID_VELOCITY_OFFSET = 0x67,
    PID_POSITION_TARGET = 0x68,
    PID_VELOCITY_ACTUAL = 0x6A,
    PID_POSITION_ACTUAL = 0x6B,
    PID_ERROR_ADDR = 0x6D,
    INTERIM_DATA = 0x6E,
    INTERIM_ADDR = 0x6F,
    WATCHDOG_CFG = 0x74,
    ADC_VM_LIMITS = 0x75,
    STEP_WIDTH = 0x78,
    UART_BPS = 0x79,
    UART_ADDRS = 0x7A,
    GPIO_dsADCI_CONFIG = 0x7B,
    STATUS_FLAGS = 0x7C,
    STATUS_MASK = 0x7D
    }

pub enum MotionMode { // From register MODE_RAMP_MODE_MOTION
    Stopped = 0,
    Torque = 1,
    Velocity = 2,
    Position = 3,
    PrbsFlux = 4,
    PrbsTorque = 5,
    PrbsVelocity = 6,
    PrbsPosition = 7,
    RqUdExt = 8,
    Reserved = 9,
    AgpiATorque = 10,
    AgpiAVelocity = 11,
    AgpiAPosition = 12,
    PmwITorque = 13,
    PmwIVelocity = 14,
    PmwIPosition = 15
}

pub struct Ventouse {
    spi: Spi<'static, p::SPI4, NoDma, NoDma>,
    cs_foc:    Output<'static, p::PE3>,
    cs_driver: Output<'static, p::PC15>
}

impl Ventouse {
    pub fn new(
        cs_foc_p: p::PE3,
        cs_driver_p: p::PC15,
        sck_p: p::PE12,
        miso_p: p::PE5,
        mosi_p: p::PE6,
        spi: p::SPI4,
        dma_rx: NoDma,
        dma_tx: NoDma,
    ) -> Self {
        let cs_foc    = Output::new(cs_foc_p,    Level::High, Speed::Medium);
        let cs_driver = Output::new(cs_driver_p, Level::High, Speed::Medium);
        let mut cfg = Config::default();
        cfg.mode = embassy_stm32::spi::MODE_3;
        let spi = Spi::new(spi, sck_p, mosi_p, miso_p, dma_tx, dma_rx, cfg);

        Self { cs_foc, cs_driver, spi }
    }

    pub fn tmc4671_set_mode(&mut self, mode: MotionMode) {
        let mut data = 0x00000000u32;
        self.tmc4671_transmit_raw_data(false, Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, &mut data).unwrap();
        data &= 0xFFFFFF00u32;
        data |= mode as u32;
        self.tmc4671_write_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data);
    }

    pub fn tmc4671_set_target_velocity(&mut self, velocity_target: i32) {
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_TARGET as u8, velocity_target as u32);
    }

    pub fn tmc4671_get_i32(&mut self, reg: u8) -> Result<i32, embassy_stm32::spi::Error> {
        if let Ok(res) = self.tmc4671_read_register(reg) {
            let mut val = 0i32;
            val +=  res[0] as i32 ;
            val += (res[1] as i32) <<  8;
            val += (res[2] as i32) << 16;
            val += (res[3] as i32) << 24;
            return Ok(val);
        } else {
            return Err(embassy_stm32::spi::Error::Framing);
        }
    }

    pub fn tmc4671_get_target_velocity(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_VELOCITY_TARGET as u8)
    }

    pub fn tmc4671_get_actual_velocity(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_VELOCITY_ACTUAL as u8)
    }

    pub fn tmc4671_init(&mut self)// -> Result<>
    {
        self.tmc4671_write_register(Tmc4671Registers::CHIPINFO_ADDR as u8, 0x00000001).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ADC_RAW_ADDR as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::dsADC_MCFG_B_MCFG_A as u8, 0x00100010).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::dsADC_MCLK_A as u8, 0x20000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::dsADC_MCLK_B as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::dsADC_MDEC_B_MDEC_A as u8, 0x014E014E).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ADC_I1_SCALE_OFFSET as u8, 0xFF0083CF).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ADC_I0_SCALE_OFFSET as u8, 0xFF00822E).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ADC_I_SELECT as u8, 0x24000100).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ADC_I1_I0_EXT as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::DS_ANALOG_INPUT_STAGE_CFG as u8, 0x00044400).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_0_SCALE_OFFSET as u8, 0x01000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_1_SCALE_OFFSET as u8, 0x01000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_2_SCALE_OFFSET as u8, 0x01000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_SELECT as u8, 0x03020100).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PWM_POLARITIES as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PWM_MAXCNT as u8, 0x00000F9F).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PWM_BBM_H_BBM_L as u8, 0x00001919).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PWM_SV_CHOP as u8, 0x00000007).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::MOTOR_TYPE_N_POLE_PAIRS as u8, 0x00030007).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PHI_E_EXT as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::OPENLOOP_MODE as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::OPENLOOP_ACCELERATION as u8, 0x0000003C).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::OPENLOOP_VELOCITY_TARGET as u8, 0xFFFFFFFB).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::OPENLOOP_VELOCITY_ACTUAL as u8, 0xFFFFFFFB).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::OPENLOOP_PHI as u8, 0x00003BDC).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::UQ_UD_EXT as u8, 0x00000483).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_DECODER_MODE as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_DECODER_PPR as u8, 0x00001000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_DECODER_COUNT as u8, 0x000006D4).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_DECODER_COUNT_N as u8, 0x000006D4).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_DECODER_PHI_E_PHI_M_OFFSET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_2_DECODER_MODE as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_2_DECODER_PPR as u8, 0x00010000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_2_DECODER_COUNT as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_2_DECODER_COUNT_N as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ABN_2_DECODER_PHI_M_OFFSET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::HALL_MODE as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::HALL_POSITION_060_000 as u8, 0x2AAA0000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::HALL_POSITION_180_120 as u8, 0x80005555).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::HALL_POSITION_300_240 as u8, 0xD555AAAA).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::HALL_PHI_E_PHI_M_OFFSET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::HALL_DPHI_MAX as u8, 0x00002AAA).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_DECODER_MODE as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_DECODER_N_THRESHOLD as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_DECODER_PHI_A_OFFSET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_DECODER_PPR as u8, 0x00000001).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_DECODER_COUNT_N as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::AENC_DECODER_PHI_E_PHI_M_OFFSET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::CONFIG_DATA as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::CONFIG_ADDR as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::VELOCITY_SELECTION as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::POSITION_SELECTION as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PHI_E_SELECTION as u8, 0x00000003).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_FLUX_P_FLUX_I as u8, 0x01DC0000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_TORQUE_P_TORQUE_I as u8, 0x01DC0000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_P_VELOCITY_I as u8, 0x032006D6).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_POSITION_P_POSITION_I as u8, 0x00FA0000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PIDOUT_UQ_UD_LIMITS as u8, 0x00005A81).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8, 0x00007D00).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_LIMIT as u8, 0x003D0900).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_POSITION_LIMIT_LOW as u8, 0x80000001).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_POSITION_LIMIT_HIGH as u8, 0x7FFFFFFF).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, 0x00000001).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_TORQUE_FLUX_OFFSET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_TARGET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_OFFSET as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_POSITION_TARGET as u8, 0xFFFFFF90).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_POSITION_ACTUAL as u8, 0xFFFFFF90).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::PID_ERROR_ADDR as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::INTERIM_DATA as u8, 0xFF92003C).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::INTERIM_ADDR as u8, 0x00000011).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::WATCHDOG_CFG as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::ADC_VM_LIMITS as u8, 0xFFFFFFFF).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::STEP_WIDTH as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::UART_BPS as u8, 0x00009600).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::UART_ADDRS as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::GPIO_dsADCI_CONFIG as u8, 0x00000000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::STATUS_FLAGS as u8, 0xF0780000).unwrap();
        self.tmc4671_write_register(Tmc4671Registers::STATUS_MASK as u8, 0x00000000).unwrap();
    }

    pub fn tmc4671_write_register(
        &mut self,
        reg: u8,
        data_w: u32
    ) -> Result<[u8; 4], embassy_stm32::spi::Error> {
        let mut data_m = data_w;
        let res = self.tmc4671_transmit_raw_data(true, reg, &mut data_m);
        let _ = Timer::after(Duration::from_millis(1)); ///////////////////////////////////// Warning!!! Testing only
        return res;
    }

    pub fn tmc4671_read_register(
        &mut self,
        reg: u8,
    ) -> Result<[u8; 4], embassy_stm32::spi::Error> {
        let mut data_m = 0x00000000u32;
        return self.tmc4671_transmit_raw_data(false, reg, &mut data_m);
    }

    pub fn tmc4671_transmit_raw_data(
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
        &mut self.cs_foc.set_low();
        let _result = &mut self.spi.blocking_transfer_in_place(&mut transfer_data); // Todo: the error is not treated.
        &mut self.cs_foc.set_high();
    
        let mut read_data = [0x00u8; 4];
        read_data[0] = transfer_data[1];
        read_data[1] = transfer_data[2];
        read_data[2] = transfer_data[3];
        read_data[3] = transfer_data[4];
    
        Ok(read_data)
    }

    pub fn tmc6200_transmit_raw_data(
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
        &mut self.cs_driver.set_low();
        let _result = &mut self.spi.blocking_transfer_in_place(&mut transfer_data); // Todo: the error is not treated.
        &mut self.cs_driver.set_high();
    
        let mut read_data = [0x00u8; 4];
        read_data[0] = transfer_data[1];
        read_data[1] = transfer_data[2];
        read_data[2] = transfer_data[3];
        read_data[3] = transfer_data[4];
    
        Ok(read_data)
    }

}
