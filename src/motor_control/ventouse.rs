// use super::axis::Axis;
use crate::config;

use defmt::*;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::spi::{Config, Instance, MisoPin, MosiPin, SckPin, Spi};
use embassy_time::*;

use super::motors_io::IOError;
use super::{Pid, RawMotorsIO};

const MOTOR_TYPE_N_POLE_PAIRS: u32 = 0x00030004; // BLDC, 4 pole-pairs ECXtorque

// PWM configuration
const PWM_POLARITIES: u32 = 0x00000000;
const PWM_MAXCNT: u32 = 0x00000F9F; // PWM-freq
const PWM_BBM_H_BBM_L: u32 = 0x00001919; // Break-Before-Make
const PWM_SV_CHOP: u32 = 0x00000107;

// ADC configuration
const ADC_I_SELECT: u32 = 0x24000100;
const DS_ADC_MCFG_B_MCFG_A: u32 = 0x00100010;
const DS_ADC_MCLK_A: u32 = 0x20000000;
const DS_ADC_MCLK_B: u32 = 0x00000000;
const DS_ADC_MDEC_B_MDEC_A: u32 = 0x014E014E;
const ADC_I0_SCALE_OFFSET: u32 = 0x002B822E; // gain is 43 and offset is centered on 2^32 (-> millis Amps)
const ADC_I1_SCALE_OFFSET: u32 = 0x002B83CF; // gain is 43 and offset is centered on 2^32 (-> millis Amps)

// ABN encoder settings
const ABN_DECODER_MODE: u32 = 0x00000000;
const ABN_DECODER_PPR: u32 = 0x00001000;
const ABN_DECODER_PHI_E_PHI_M_OFFSET: u32 = 0x00000000;

// Limits
const PID_TORQUE_FLUX_LIMITS: u32 = 0x00001000; // 4000

// Motor alignment
const OPENLOOP_ACCELERATION: u32 = 0x0000003c; // Wizard default
const UQ_UD_EXT: u32 = 0x000007D0; // Openloop "torque_target"

#[allow(non_camel_case_types)]
#[allow(dead_code)]
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

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum MotionMode {
    // From register MODE_RAMP_MODE_MOTION
    Stopped,
    Torque,
    Velocity,
    Position,
    PrbsFlux,
    PrbsTorque,
    PrbsVelocity,
    PrbsPosition,
    UqUdExt,
    Reserved,
    AgpiATorque,
    AgpiAVelocity,
    AgpiAPosition,
    PmwITorque,
    PmwIVelocity,
    PmwIPosition,
}

impl MotionMode {
    fn from_u8(val: u8) -> Option<MotionMode> {
        match val {
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
            _ => None,
        }
    }
}

pub struct Ventouse<'d, T, CsFoc, CsDrv, FocEnb, FocStat, DrvFlt>
where
    T: Instance,
    CsFoc: Pin,
    CsDrv: Pin,
    FocEnb: Pin,
    FocStat: Pin,
    DrvFlt: Pin,
{
    // Now that is for J5 (middle FCC) - Motor "B"
    spi: Spi<'d, T, NoDma, NoDma>,

    cs_foc: Output<'d, CsFoc>,
    cs_driver: Output<'d, CsDrv>,
    foc_enable: Output<'d, FocEnb>,
    #[allow(dead_code)]
    foc_status: Input<'d, FocStat>,
    #[allow(dead_code)]
    driver_fault: Input<'d, DrvFlt>,

    brushless_motor_config: config::BrushlessMotor,

    ppr: Option<f32>,
}

pub struct VentouseConfig<
    T: Instance,
    CsFoc: Pin,
    CsDrv: Pin,
    Sck: SckPin<T>,
    Mosi: MosiPin<T>,
    Miso: MisoPin<T>,
    FocEnb: Pin,
    FocStat: Pin,
    DrvFlt: Pin,
> {
    pub cs_foc: CsFoc,
    pub cs_driver: CsDrv,
    pub peri: T,
    pub sck: Sck,
    pub mosi: Mosi,
    pub miso: Miso,
    pub foc_enable: FocEnb,
    pub foc_status: FocStat,
    pub driver_fault: DrvFlt,
}

#[allow(dead_code)]
impl<'d, T, CsFoc, CsDrv, FocEnb, FocStat, DrvFlt>
    Ventouse<'d, T, CsFoc, CsDrv, FocEnb, FocStat, DrvFlt>
where
    T: Instance,
    CsFoc: Pin,
    CsDrv: Pin,
    FocEnb: Pin,
    FocStat: Pin,
    DrvFlt: Pin,
{
    pub fn new(
        ventouse_config: VentouseConfig<
            T,
            CsFoc,
            CsDrv,
            impl SckPin<T>,
            impl MosiPin<T>,
            impl MisoPin<T>,
            FocEnb,
            FocStat,
            DrvFlt,
        >,
        brushless_motor_config: config::BrushlessMotor,
    ) -> Self {
        let mut spi_config = Config::default();
        spi_config.mode = embassy_stm32::spi::MODE_3;
        let spi = Spi::new(
            ventouse_config.peri,
            ventouse_config.sck,
            ventouse_config.mosi,
            ventouse_config.miso,
            NoDma,
            NoDma,
            spi_config,
        );

        // IOs
        let cs_foc = Output::new(ventouse_config.cs_foc, Level::High, Speed::Medium);
        let cs_driver = Output::new(ventouse_config.cs_driver, Level::High, Speed::Medium);

        let mut foc_enable = Output::new(ventouse_config.foc_enable, Level::Low, Speed::Low);
        foc_enable.set_low();
        let foc_status = Input::new(ventouse_config.foc_status, Pull::None);
        let driver_fault = Input::new(ventouse_config.driver_fault, Pull::None);

        Self {
            cs_foc,
            cs_driver,
            spi,
            foc_enable,
            foc_status,
            driver_fault,
            brushless_motor_config,
            ppr: None,
        }
    }

    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        self.tmc4671_init_registers().await?;
        info!("TMC4671 init done");

        self.ppr = Some(self.tmc4671_get_encoder_ppr()? as f32);

        self.tmc4671_align_motor().await?;
        info!("Motor align done");
        self.tmc4671_set_mode(MotionMode::Position)?;
        info!("Motor set to position mode done");

        Ok(())
    }

    pub fn tmc4671_enable(&mut self) {
        self.foc_enable.set_high();
    }

    pub fn tmc4671_disable(&mut self) {
        self.foc_enable.set_low();
    }

    pub fn tmc4671_set_mode(&mut self, mode: MotionMode) -> Result<u32, embassy_stm32::spi::Error> {
        let mut data = 0x00000000u32;
        // read current state first
        self.tmc4671_transmit_raw_data(false, Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)?;
        data &= 0xFFFFFF00u32;
        data |= mode as u32;
        self.tmc4671_write_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)
    }

    pub fn tmc4671_get_mode(&mut self) -> Result<MotionMode, embassy_stm32::spi::Error> {
        if let Ok(read) = self.tmc4671_read_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8)
        {
            Ok(MotionMode::from_u8((read & 0x000000FFu32) as u8).unwrap())
        } else {
            Err(embassy_stm32::spi::Error::Framing)
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

    pub fn tmc4671_set_torque_target(
        &mut self,
        torque_target: i16,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        // read current state first -> bits 15-0 is flux_target and should be kept.
        let mut torque_and_flux =
            self.tmc4671_read_register(Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8)?;
        torque_and_flux &= 0x0000FFFFu32; // clear actual torque
        torque_and_flux |= (torque_target as u32) << 16;
        self.tmc4671_write_register(
            Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8,
            torque_and_flux,
        )
    }

    pub fn tmc4671_set_flux_target(
        &mut self,
        flux_target: i16,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        // read current state first -> bits 31-16 is torque_target and should be kept.
        let mut torque_and_flux =
            self.tmc4671_read_register(Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8)?;
        torque_and_flux &= 0xFFFF0000u32; // clear actual flux
        torque_and_flux |= flux_target as u32;
        self.tmc4671_write_register(
            Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8,
            torque_and_flux,
        )
    }

    pub fn tmc4671_set_target_velocity(
        &mut self,
        velocity_target: i32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(
            Tmc4671Registers::PID_VELOCITY_TARGET as u8,
            velocity_target as u32,
        )
    }

    pub fn tmc4671_get_target_velocity(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_VELOCITY_TARGET as u8)
    }

    pub fn tmc4671_get_actual_velocity(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_VELOCITY_ACTUAL as u8)
    }

    pub fn tmc4671_set_target_position(
        &mut self,
        position_target: i32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(
            Tmc4671Registers::PID_POSITION_TARGET as u8,
            position_target as u32,
        )
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

    pub fn tmc4671_set_encoder_ppr(&mut self, ppr: i32) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_TARGET as u8, ppr as u32)
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
            DS_ADC_MCFG_B_MCFG_A,
        )?;
        self.tmc4671_checked_write(Tmc4671Registers::dsADC_MCLK_A as u8, DS_ADC_MCLK_A)?;
        self.tmc4671_checked_write(Tmc4671Registers::dsADC_MCLK_B as u8, DS_ADC_MCLK_B)?;
        self.tmc4671_checked_write(
            Tmc4671Registers::dsADC_MDEC_B_MDEC_A as u8,
            DS_ADC_MDEC_B_MDEC_A,
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
            self.brushless_motor_config.pid_flux_p_flux_i(),
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_TORQUE_P_TORQUE_I as u8,
            self.brushless_motor_config.pid_torque_p_torque_i(),
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_VELOCITY_P_VELOCITY_I as u8,
            self.brushless_motor_config.pid_velocity_p_velocity_i(),
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_POSITION_P_POSITION_I as u8,
            self.brushless_motor_config.pid_position_p_position_i(),
        )?;

        Ok(())
    }

    pub async fn tmc4671_align_motor(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        // /!\ Please note that the TMC6200 must be in Single-line mode (aka 6-PMW)
        self.tmc6200_checked_write(0x00u8, 0x00000000u32)?;

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
        self.tmc4671_set_mode(MotionMode::Velocity)?;

        // Rotate right
        info!("Rotate right...");
        self.tmc4671_set_target_velocity(1000)?;
        let _ = Timer::after(Duration::from_millis(1000)).await;

        // Rotate left
        info!("Rotate left...");
        self.tmc4671_set_target_velocity(-1000)?;
        let _ = Timer::after(Duration::from_millis(1000)).await;

        // Stop
        info!("Stop...");
        self.tmc4671_set_target_velocity(0)?;
        self.tmc4671_set_mode(MotionMode::Stopped)?;

        Ok(())
    }

    fn tmc4671_checked_write(
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

    fn tmc4671_write_register(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        let data_m = data_w;
        self.tmc4671_transmit_raw_data(true, reg, data_m)
    }

    fn tmc4671_read_register(&mut self, reg: u8) -> Result<u32, embassy_stm32::spi::Error> {
        let data_m = 0x00000000u32;
        self.tmc4671_transmit_raw_data(false, reg, data_m)
    }

    fn tmc4671_transmit_raw_data(
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
        self.cs_foc.set_low();
        let result = self.spi.blocking_transfer_in_place(&mut transfer_data);
        self.cs_foc.set_high();

        result?;

        let mut read_data = transfer_data[4] as u32;
        read_data += (transfer_data[3] as u32) << 8;
        read_data += (transfer_data[2] as u32) << 16;
        read_data += (transfer_data[1] as u32) << 24;

        Ok(read_data)
    }

    fn tmc6200_checked_write(
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
        self.cs_driver.set_low();
        let result = self.spi.blocking_transfer_in_place(&mut transfer_data);
        self.cs_driver.set_high();

        result?;

        let mut read_data = transfer_data[4] as u32;
        read_data += (transfer_data[3] as u32) << 8;
        read_data += (transfer_data[2] as u32) << 16;
        read_data += (transfer_data[1] as u32) << 24;

        Ok(read_data)
    }

    fn tmc4671_get_u32(&mut self, reg: u8) -> Result<u32, embassy_stm32::spi::Error> {
        if let Ok(res) = self.tmc4671_read_register(reg) {
            Ok(res)
        } else {
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    fn tmc4671_get_i32(&mut self, reg: u8) -> Result<i32, embassy_stm32::spi::Error> {
        if let Ok(res) = self.tmc4671_read_register(reg) {
            Ok(res as i32)
        } else {
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    fn tmc4671_get_lower_i16(&mut self, reg: u8) -> Result<i16, embassy_stm32::spi::Error> {
        if let Ok(res) = self.tmc4671_read_register(reg) {
            Ok((res & 0x0000FFFFu32) as i16)
        } else {
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    fn tmc4671_get_upper_i16(&mut self, reg: u8) -> Result<i16, embassy_stm32::spi::Error> {
        match self.tmc4671_read_register(reg) {
            Ok(res) => Ok(((res & 0xFFFF0000u32) >> 16) as i16),
            Err(e) => Err(e),
        }
    }
}

impl<'d, T, CsFoc, CsDrv, FocEnb, FocStat, DrvFlt> RawMotorsIO<1>
    for Ventouse<'d, T, CsFoc, CsDrv, FocEnb, FocStat, DrvFlt>
where
    T: Instance,
    CsFoc: Pin,
    CsDrv: Pin,
    FocEnb: Pin,
    FocStat: Pin,
    DrvFlt: Pin,
{
    /// Check if the motors are ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; 1], IOError> {
        Ok([self.foc_enable.is_set_high()])
    }
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: [bool; 1]) -> Result<(), IOError> {
        if on[0] {
            self.tmc4671_enable();
        } else {
            self.tmc4671_disable();
        }
        Ok(())
    }

    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f32; 1], IOError> {
        let encoder = self
            .tmc4671_get_actual_position()
            .map_err(IOError::SpiError)?;
        let rads = conversion::encoder_to_rads(encoder, self.ppr.unwrap());

        Ok([rads])
    }
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f32; 1], IOError> {
        // TODO:
        Ok([0.0])
    }
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f32; 1], IOError> {
        // TODO:
        Ok([0.0])
    }

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; 1], IOError> {
        // TODO:
        Ok([0.0])
    }
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f32; 1]) -> Result<(), IOError> {
        self.tmc4671_set_target_position(conversion::rads_to_encoder(
            position[0],
            self.ppr.unwrap(),
        ))
        .map(|_| ())
        .map_err(IOError::SpiError)
    }

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f32; 1], IOError> {
        // TODO:
        Ok([0.0])
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, _velocity: [f32; 1]) -> Result<(), IOError> {
        // TODO:
        Ok(())
    }

    /// Get the torque limit of the motors (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f32; 1], IOError> {
        // TODO:
        Ok([0.0])
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_limit(&mut self, _torque: [f32; 1]) -> Result<(), IOError> {
        // TODO:
        Ok(())
    }

    /// Get the current PID gains of the motors
    fn get_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        // TODO:
        Ok([Pid {
            p: 0.0,
            i: 0.0,
            d: 0.0,
        }])
    }
    /// Set the current PID gains of the motors
    fn set_pid_gains(&mut self, _pid: [Pid; 1]) -> Result<(), IOError> {
        // TODO:
        Ok(())
    }
}

mod conversion {
    pub fn encoder_to_rads(enc: i32, ppr: f32) -> f32 {
        enc as f32 / ppr
    }
    pub fn rads_to_encoder(rads: f32, ppr: f32) -> i32 {
        (rads * ppr) as i32
    }
}

pub enum VentouseKind {
    A(config::VentouseA),
    B(config::VentouseB),
}

impl VentouseKind {
    pub async fn init(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        match self {
            VentouseKind::A(v) => v.init().await,
            VentouseKind::B(v) => v.init().await,
        }
    }
}

// TODO: make this generic (how?)
impl RawMotorsIO<1> for VentouseKind {
    /// Check if the motors are ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; 1], IOError> {
        match self {
            VentouseKind::A(v) => v.is_torque_on(),
            VentouseKind::B(v) => v.is_torque_on(),
        }
    }
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: [bool; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(v) => v.set_torque(on),
            VentouseKind::B(v) => v.set_torque(on),
        }
    }

    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(v) => v.get_current_position(),
            VentouseKind::B(v) => v.get_current_position(),
        }
    }
    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(v) => v.get_current_velocity(),
            VentouseKind::B(v) => v.get_current_velocity(),
        }
    }
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(v) => v.get_current_torque(),
            VentouseKind::B(v) => v.get_current_torque(),
        }
    }

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(v) => v.get_target_position(),
            VentouseKind::B(v) => v.get_target_position(),
        }
    }
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(v) => v.set_target_position(position),
            VentouseKind::B(v) => v.set_target_position(position),
        }
    }

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(v) => v.get_velocity_limit(),
            VentouseKind::B(v) => v.get_velocity_limit(),
        }
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(v) => v.set_velocity_limit(velocity),
            VentouseKind::B(v) => v.set_velocity_limit(velocity),
        }
    }

    /// Get the torque limit of the motors (in Nm)
    fn get_torque_limit(&mut self) -> Result<[f32; 1], IOError> {
        match self {
            VentouseKind::A(v) => v.get_torque_limit(),
            VentouseKind::B(v) => v.get_torque_limit(),
        }
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_limit(&mut self, torque: [f32; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(v) => v.set_torque_limit(torque),
            VentouseKind::B(v) => v.set_torque_limit(torque),
        }
    }

    /// Get the current PID gains of the motors
    fn get_pid_gains(&mut self) -> Result<[Pid; 1], IOError> {
        match self {
            VentouseKind::A(v) => v.get_pid_gains(),
            VentouseKind::B(v) => v.get_pid_gains(),
        }
    }
    /// Set the current PID gains of the motors
    fn set_pid_gains(&mut self, pid: [Pid; 1]) -> Result<(), IOError> {
        match self {
            VentouseKind::A(v) => v.set_pid_gains(pid),
            VentouseKind::B(v) => v.set_pid_gains(pid),
        }
    }
}
