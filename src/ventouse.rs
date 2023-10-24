#![no_std]
#![no_main]

use crate::config::MotorConfig;

use defmt::*;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals as p;
use embassy_stm32::spi::{Config, Spi};
use embassy_time::*;
use {defmt_rtt as _, panic_probe as _}; // TODO : to be remove (delays for testing purposes only)
                                        // Motor type & PWM configuration
                                        // const MOTOR_TYPE_N_POLE_PAIRS: u32 = 0x00030007; // BLDC, 7 pole-pairs EC45
const MOTOR_TYPE_N_POLE_PAIRS: u32 = 0x00030004; // BLDC, 4 pole-pairs ECXtorque

const PWM_POLARITIES: u32 = 0x00000000;
const PWM_MAXCNT: u32 = 0x00000F9F; // PWM-freq
const PWM_BBM_H_BBM_L: u32 = 0x00001919; // Break-Before-Make
const PWM_SV_CHOP: u32 = 0x00000107;
// ADC configuration
const ADC_I_SELECT: u32 = 0x24000100;
const dsADC_MCFG_B_MCFG_A: u32 = 0x00100010;
const dsADC_MCLK_A: u32 = 0x20000000;
const dsADC_MCLK_B: u32 = 0x00000000;
const dsADC_MDEC_B_MDEC_A: u32 = 0x014E014E;

const ADC_I0_SCALE_OFFSET: u32 = 0x002B822E; // gain is 43 and offset is centered on 2^32 (-> millis Amps)
const ADC_I1_SCALE_OFFSET: u32 = 0x002B83CF; // gain is 43 and offset is centered on 2^32
                                             // const ADC_I0_SCALE_OFFSET: u32 = 0x010083D4;
                                             // const ADC_I1_SCALE_OFFSET: u32 = 0x01008355;

// ABN encoder settings
const ABN_DECODER_MODE: u32 = 0x00000000;
const ABN_DECODER_PPR: u32 = 0x00001000;
const ABN_DECODER_PHI_E_PHI_M_OFFSET: u32 = 0x00000000;
// Limits
const PID_TORQUE_FLUX_LIMITS: u32 = 0x00001000; // 4000
                                                // PI settings
                                                //EC45
                                                // const PID_FLUX_P_FLUX_I: u32 = 0x03200000;
                                                // const PID_TORQUE_P_TORQUE_I: u32 = 0x03200000;
                                                // const PID_VELOCITY_P_VELOCITY_I: u32 = 0x01F401C2;
                                                // const PID_POSITION_P_POSITION_I: u32 = 0x00500000;

// const PID_TORQUE_FLUX_LIMITS: u32 = 0x00001000; // 4000
//                                                 // PI settings
//ECX22
// const PID_FLUX_P_FLUX_I: u32 = 0x03200080;
// const PID_TORQUE_P_TORQUE_I: u32 = 0x03200000;
// const PID_VELOCITY_P_VELOCITY_I: u32 = 0x01000080;
// const PID_POSITION_P_POSITION_I: u32 = 0x00400010;

// Motor alignment
const OPENLOOP_ACCELERATION: u32 = 0x0000003c; // Wizard default
const UQ_UD_EXT: u32 = 0x000007D0; // Openloop "torque_target"

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
    PID_TORQUE_FLUX_ACTUAL = 0x69,
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
    STATUS_MASK = 0x7D,
}

pub enum MotionMode {
    // From register MODE_RAMP_MODE_MOTION
    Stopped = 0,
    Torque = 1,
    Velocity = 2,
    Position = 3,
    PrbsFlux = 4,
    PrbsTorque = 5,
    PrbsVelocity = 6,
    PrbsPosition = 7,
    UqUdExt = 8,
    Reserved = 9,
    AgpiATorque = 10,
    AgpiAVelocity = 11,
    AgpiAPosition = 12,
    PmwITorque = 13,
    PmwIVelocity = 14,
    PmwIPosition = 15,
}

impl MotionMode {
    fn motion_from_u8(value: u8) -> Option<MotionMode> {
        match value {
            0 => Some(MotionMode::Stopped),
            1 => Some(MotionMode::Torque),
            2 => Some(MotionMode::Velocity),
            3 => Some(MotionMode::Position),
            4 => Some(MotionMode::PrbsFlux),
            5 => Some(MotionMode::PrbsTorque),
            6 => Some(MotionMode::PrbsVelocity),
            7 => Some(MotionMode::PrbsPosition),
            8 => Some(MotionMode::UqUdExt),
            9 => Some(MotionMode::Reserved),
            10 => Some(MotionMode::AgpiATorque),
            11 => Some(MotionMode::AgpiAVelocity),
            12 => Some(MotionMode::AgpiAPosition),
            13 => Some(MotionMode::PmwITorque),
            14 => Some(MotionMode::PmwIVelocity),
            15 => Some(MotionMode::PmwIPosition),
            _ => None, // Handle invalid values, if needed
        }
    }
    fn motion_to_u8(value: MotionMode) -> Option<u8> {
        match value {
            MotionMode::Stopped => Some(0),
            MotionMode::Torque => Some(1),
            MotionMode::Velocity => Some(2),
            MotionMode::Position => Some(3),
            MotionMode::PrbsFlux => Some(4),
            MotionMode::PrbsTorque => Some(5),
            MotionMode::PrbsVelocity => Some(6),
            MotionMode::PrbsPosition => Some(7),
            MotionMode::UqUdExt => Some(8),
            MotionMode::Reserved => Some(9),
            MotionMode::AgpiATorque => Some(10),
            MotionMode::AgpiAVelocity => Some(11),
            MotionMode::AgpiAPosition => Some(12),
            MotionMode::PmwITorque => Some(13),
            MotionMode::PmwIVelocity => Some(14),
            MotionMode::PmwIPosition => Some(15),
            _ => None, // Handle invalid values, if needed
        }
    }
}

pub struct Ventouse {
    // Now that is for J5 (middle FCC) - Motor "B"
    spi: Spi<'static, p::SPI4, NoDma, NoDma>,
    cs_foc: Output<'static, p::PE3>,
    cs_driver: Output<'static, p::PC15>,
    foc_enable: Output<'static, p::PE0>,
    foc_status: Input<'static, p::PC13>,
    driver_fault: Input<'static, p::PC14>,
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
        foc_enable_p: p::PE0,
        foc_status_p: p::PC13,
        driver_fault_p: p::PC14,
    ) -> Self {
        // SPI
        let cs_foc = Output::new(cs_foc_p, Level::High, Speed::Medium);
        let cs_driver = Output::new(cs_driver_p, Level::High, Speed::Medium);
        let mut cfg = Config::default();
        cfg.mode = embassy_stm32::spi::MODE_3;
        let spi = Spi::new(spi, sck_p, mosi_p, miso_p, dma_tx, dma_rx, cfg);
        // IOs
        let mut foc_enable = Output::new(foc_enable_p, Level::Low, Speed::Low);
        foc_enable.set_low();
        let foc_status = Input::new(foc_status_p, Pull::None);
        let driver_fault = Input::new(driver_fault_p, Pull::None);

        Self {
            cs_foc,
            cs_driver,
            spi,
            foc_enable,
            foc_status,
            driver_fault,
        }
    }

    pub fn tmc4671_enable(&mut self) {
        self.foc_enable.set_high();
    }

    pub fn tmc4671_disable(&mut self) {
        self.foc_enable.set_low();
    }

    pub fn tmc4671_set_mode(&mut self, mode: MotionMode) {
        let mut data = 0x00000000u32;
        // read current state first
        self.tmc4671_transmit_raw_data(false, Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)
            .unwrap();
        data &= 0xFFFFFF00u32;
        data |= mode as u32;
        self.tmc4671_write_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data);
    }

    pub fn tmc4671_get_mode(&mut self) -> Result<u8, embassy_stm32::spi::Error> {
        // TODO: it should return a MotionMode but display of enum with defmt is unstable
        if let Ok(read) = self.tmc4671_read_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8)
        {
            return Ok((read & 0x000000FFu32) as u8);
            /*let data = match MotionMode::motion_from_u8(res as u8) {
                Some(d) => d,
                None => return Err(embassy_stm32::spi::Error::Framing)
            };
            return Ok(data);*/
        } else {
            return Err(embassy_stm32::spi::Error::Framing);
        }
    }

    pub fn tmc4671_get_u32(&mut self, reg: u8) -> Result<u32, embassy_stm32::spi::Error> {
        if let Ok(res) = self.tmc4671_read_register(reg) {
            return Ok(res);
        } else {
            return Err(embassy_stm32::spi::Error::Framing);
        }
    }

    pub fn tmc4671_get_i32(&mut self, reg: u8) -> Result<i32, embassy_stm32::spi::Error> {
        if let Ok(res) = self.tmc4671_read_register(reg) {
            return Ok(res as i32);
        } else {
            return Err(embassy_stm32::spi::Error::Framing);
        }
    }

    fn tmc4671_get_lower_i16(&mut self, reg: u8) -> Result<i16, embassy_stm32::spi::Error> {
        if let Ok(res) = self.tmc4671_read_register(reg) {
            return Ok((res & 0x0000FFFFu32) as i16);
        } else {
            return Err(embassy_stm32::spi::Error::Framing);
        }
    }

    fn tmc4671_get_upper_i16(&mut self, reg: u8) -> Result<i16, embassy_stm32::spi::Error> {
        match self.tmc4671_read_register(reg) {
            Ok(res) => Ok(((res & 0xFFFF0000u32) >> 16) as i16),
            Err(e) => Err(e),
        }
    }

    pub fn tmc4671_get_torque_actual(&mut self) -> Result<i16, embassy_stm32::spi::Error> {
        self.tmc4671_get_upper_i16(Tmc4671Registers::PID_TORQUE_FLUX_ACTUAL as u8)
    }

    pub fn tmc4671_get_flux_actual(&mut self) -> Result<i16, embassy_stm32::spi::Error> {
        self.tmc4671_get_lower_i16(Tmc4671Registers::PID_TORQUE_FLUX_ACTUAL as u8)
    }

    pub fn tmc4671_get_torque_target(&mut self) -> Result<i16, embassy_stm32::spi::Error> {
        self.tmc4671_get_upper_i16(Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8)
    }

    pub fn tmc4671_get_flux_target(&mut self) -> Result<i16, embassy_stm32::spi::Error> {
        self.tmc4671_get_lower_i16(Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8)
    }

    pub fn tmc4671_set_torque_target(&mut self, torque_target: i16) {
        // read current state first -> bits 15-0 is flux_target and should be kept.
        let mut torque_and_flux = self
            .tmc4671_read_register(Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8)
            .unwrap();
        torque_and_flux &= 0x0000FFFFu32; // clear actual torque
        torque_and_flux |= (torque_target as u32) << 16;
        self.tmc4671_write_register(
            Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8,
            torque_and_flux,
        );
    }

    pub fn tmc4671_set_flux_target(&mut self, flux_target: i16) {
        // read current state first -> bits 31-16 is torque_target and should be kept.
        let mut torque_and_flux = self
            .tmc4671_read_register(Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8)
            .unwrap();
        torque_and_flux &= 0xFFFF0000u32; // clear actual flux
        torque_and_flux |= flux_target as u32;
        self.tmc4671_write_register(
            Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8,
            torque_and_flux,
        );
    }

    pub fn tmc4671_set_target_velocity(&mut self, velocity_target: i32) {
        self.tmc4671_write_register(
            Tmc4671Registers::PID_VELOCITY_TARGET as u8,
            velocity_target as u32,
        );
    }

    pub fn tmc4671_get_target_velocity(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_VELOCITY_TARGET as u8)
    }

    pub fn tmc4671_get_actual_velocity(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_VELOCITY_ACTUAL as u8)
    }

    pub fn tmc4671_get_actual_position(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_POSITION_ACTUAL as u8)
    }

    pub fn tmc4671_get_encoder_count(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::ABN_DECODER_COUNT as u8)
    }

    pub fn tmc4671_get_encoder_ppr(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::ABN_DECODER_PPR as u8)
    }

    pub fn tmc4671_set_encoder_ppr(&mut self, ppr: i32) {
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_TARGET as u8, ppr as u32);
    }

    pub async fn tmc4671_init_registers(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        // // /!\ Please note that the TMC6200 must be in Single-line mode (aka 6-PMW)
        // self.tmc6200_checked_write(0x00u8, 0x00000000u32);

        // Motor type & PWM configuration
        self.tmc4671_checked_write(
            Tmc4671Registers::MOTOR_TYPE_N_POLE_PAIRS as u8,
            MOTOR_TYPE_N_POLE_PAIRS,
        )?;
        self.tmc4671_checked_write(Tmc4671Registers::PWM_POLARITIES as u8, PWM_POLARITIES)?;
        self.tmc4671_checked_write(Tmc4671Registers::PWM_MAXCNT as u8, PWM_MAXCNT)?;
        self.tmc4671_checked_write(Tmc4671Registers::PWM_BBM_H_BBM_L as u8, PWM_BBM_H_BBM_L)?;
        self.tmc4671_checked_write(Tmc4671Registers::PWM_SV_CHOP as u8, PWM_SV_CHOP)?;

        // ADC configuration
        self.tmc4671_checked_write(Tmc4671Registers::ADC_I_SELECT as u8, ADC_I_SELECT)?;
        self.tmc4671_checked_write(
            Tmc4671Registers::dsADC_MCFG_B_MCFG_A as u8,
            dsADC_MCFG_B_MCFG_A,
        )?;
        self.tmc4671_checked_write(Tmc4671Registers::dsADC_MCLK_A as u8, dsADC_MCLK_A)?;
        self.tmc4671_checked_write(Tmc4671Registers::dsADC_MCLK_B as u8, dsADC_MCLK_B)?;
        self.tmc4671_checked_write(
            Tmc4671Registers::dsADC_MDEC_B_MDEC_A as u8,
            dsADC_MDEC_B_MDEC_A,
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::ADC_I1_SCALE_OFFSET as u8,
            ADC_I0_SCALE_OFFSET,
        )?; // gain = 43
        self.tmc4671_checked_write(
            Tmc4671Registers::ADC_I0_SCALE_OFFSET as u8,
            ADC_I1_SCALE_OFFSET,
        )?; // gain = 43

        // ABN encoder settings
        self.tmc4671_checked_write(Tmc4671Registers::ABN_DECODER_MODE as u8, ABN_DECODER_MODE)?;
        self.tmc4671_checked_write(Tmc4671Registers::ABN_DECODER_PPR as u8, ABN_DECODER_PPR)?;
        self.tmc4671_checked_write(
            Tmc4671Registers::ABN_DECODER_PHI_E_PHI_M_OFFSET as u8,
            ABN_DECODER_PHI_E_PHI_M_OFFSET,
        )?;

        // Limits
        //        self.tmc4671_checked_write(Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8, 0x00007D00)?; // 32000
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8,
            PID_TORQUE_FLUX_LIMITS,
        )?; // ~4000

        // PI settings
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_FLUX_P_FLUX_I as u8,
            MotorConfig::PID_FLUX_P_FLUX_I as u32,
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_TORQUE_P_TORQUE_I as u8,
            MotorConfig::PID_TORQUE_P_TORQUE_I as u32,
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_VELOCITY_P_VELOCITY_I as u8,
            MotorConfig::PID_VELOCITY_P_VELOCITY_I as u32,
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_POSITION_P_POSITION_I as u8,
            MotorConfig::PID_POSITION_P_POSITION_I as u32,
        )?;

        Ok(())
    }

    pub async fn tmc4671_align_motor(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        // /!\ Please note that the TMC6200 must be in Single-line mode (aka 6-PMW)
        self.tmc6200_checked_write(0x00u8, 0x00000000u32);

        // Set openloop mode
        self.tmc4671_checked_write(Tmc4671Registers::OPENLOOP_MODE as u8, 0x00000000)?; // Positive Openloop phi e (OPENLOOP_MODE)
        self.tmc4671_checked_write(
            Tmc4671Registers::OPENLOOP_ACCELERATION as u8,
            OPENLOOP_ACCELERATION,
        )?; // Default acceleration
        self.tmc4671_checked_write(Tmc4671Registers::OPENLOOP_VELOCITY_TARGET as u8, 0x00000000)?; // Motor is stopped
        self.tmc4671_checked_write(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, 0x00000008)?; // Open loop mode
        self.tmc4671_checked_write(
            Tmc4671Registers::ABN_DECODER_PHI_E_PHI_M_OFFSET as u8,
            0x00000000,
        )?;
        self.tmc4671_checked_write(Tmc4671Registers::PHI_E_SELECTION as u8, 0x00000002)?; // Phi_e_openloop
        self.tmc4671_checked_write(Tmc4671Registers::UQ_UD_EXT as u8, UQ_UD_EXT)?; // 2000, ud_ext only
        self.tmc4671_enable(); // Start moving
        let _ = Timer::after(Duration::from_millis(1000)).await;
        // Now the motor is aligned with a phase.

        // Clear abn_decoder_count
        self.tmc4671_checked_write(Tmc4671Registers::ABN_DECODER_COUNT as u8, 0x00000000)?;

        // Feedback selection
        self.tmc4671_checked_write(Tmc4671Registers::PHI_E_SELECTION as u8, 0x00000003)?; // Phi_e_ABN
        self.tmc4671_checked_write(Tmc4671Registers::VELOCITY_SELECTION as u8, 0x00000000)?; // PHI_E_SELECTION

        //put max value
        // self.tmc4671_checked_write(Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8, 0x00007D00)?; // ~4000

        // Move!
        self.tmc4671_set_mode(MotionMode::Velocity);

        // Rotate right
        info!("Rotate right...");
        self.tmc4671_set_target_velocity(1000);
        let _ = Timer::after(Duration::from_millis(3000)).await;

        // Rotate left
        info!("Rotate left...");
        self.tmc4671_set_target_velocity(-1000);
        let _ = Timer::after(Duration::from_millis(3000)).await;

        // Stop
        info!("Stop...");
        self.tmc4671_set_target_velocity(0);
        self.tmc4671_set_mode(MotionMode::Stopped);

        Ok(())
    }

    pub fn tmc4671_checked_write(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<(), embassy_stm32::spi::Error> {
        self.tmc4671_write_register(reg, data_w)?;
        let data_r = self.tmc4671_read_register(reg)?;
        if data_r == data_w {
            Ok(())
        } else {
            info!("!!! Error INIT {:#x}_r / {:#x}_w !!!", data_r, data_w);
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    pub fn tmc4671_write_register(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        let mut data_m = data_w;
        let res = self.tmc4671_transmit_raw_data(true, reg, data_m);
        return res;
    }

    pub fn tmc4671_read_register(&mut self, reg: u8) -> Result<u32, embassy_stm32::spi::Error> {
        let mut data_m = 0x00000000u32;
        return self.tmc4671_transmit_raw_data(false, reg, data_m);
    }

    pub fn tmc4671_transmit_raw_data(
        &mut self,
        write_bit: bool,
        addr: u8,
        data: u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        // Building the array
        let mut msb_data = addr;
        let mut data_u8_array = data.to_le_bytes();
        if write_bit == true {
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
        &mut self.cs_foc.set_low();
        let _result = &mut self.spi.blocking_transfer_in_place(&mut transfer_data);
        &mut self.cs_foc.set_high();

        let mut read_data = transfer_data[4] as u32;
        read_data += (transfer_data[3] as u32) << 8;
        read_data += (transfer_data[2] as u32) << 16;
        read_data += (transfer_data[1] as u32) << 24;

        Ok(read_data)
    }

    pub fn tmc6200_checked_write(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<(), embassy_stm32::spi::Error> {
        self.tmc6200_write_register(reg, data_w)?;
        let data_r = self.tmc6200_read_register(reg)?;
        if data_r == data_w {
            Ok(())
        } else {
            info!("!!! Error INIT {:#x}_r / {:#x}_w !!!", data_r, data_w);
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    pub fn tmc6200_write_register(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        let mut data_m = data_w;
        let res = self.tmc6200_transmit_raw_data(true, reg, &mut data_m);
        return res;
    }

    pub fn tmc6200_read_register(&mut self, reg: u8) -> Result<u32, embassy_stm32::spi::Error> {
        let mut data_m = 0x00000000u32;
        return self.tmc6200_transmit_raw_data(false, reg, &mut data_m);
    }

    pub fn tmc6200_transmit_raw_data(
        &mut self,
        write_bit: bool,
        addr: u8,
        data: &mut u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        // Building array
        let mut msb_data = addr;
        if write_bit == true {
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
        &mut self.cs_driver.set_low();
        let _result = &mut self.spi.blocking_transfer_in_place(&mut transfer_data); // Todo: the error is not treated.
        &mut self.cs_driver.set_high();

        let mut read_data = transfer_data[4] as u32;
        read_data += (transfer_data[3] as u32) << 8;
        read_data += (transfer_data[2] as u32) << 16;
        read_data += (transfer_data[1] as u32) << 24;

        Ok(read_data)
    }
}
