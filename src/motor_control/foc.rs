use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::spi::{Instance, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal_1::spi::SpiDevice;

use libm;

use crate::utils::errors::IOError;

use crate::config;

use super::ventouse::conversion;

// PWM configuration
const PWM_POLARITIES: u32 = 0x00000000;

// drv8316 which are in the gamma elec for orbita3d use low-side
// current sensing and require a bit more time to measure the current
// so we need to keep the pwm frequency a bit lower (at 25kHz)
#[cfg(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
))]
const PWM_MAXCNT: u32 = 0x00000F9F; // PWM-freq 3999 = > 25KHz
                                    // for other boards we can use a higher frequency as they use inline current sensing
#[cfg(not(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
)))]
const PWM_MAXCNT: u32 = 0x000007CF; // PWM-freq 1999 = > 50KHz slightly less noisy
                                    // const PWM_MAXCNT: u32 = 0x000003E7; // PWM-freq 999 = > 100KHz

const PWM_BBM_H_BBM_L: u32 = 0x00001919; // Break-Before-Make

// for drv8316 which are in the gamma elec for orbita3d use low-side
// better current measurement with space vector than with sine wave
#[cfg(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
))]
const PWM_SV_CHOP: u32 = 0x00000107; //Space vector On + PWM centered
                                     // for other boards we can use the default value
#[cfg(not(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
)))]
const PWM_SV_CHOP: u32 = 0x00000007; //Space vector On + PWM centered

// ADC configuration
#[cfg(feature = "beta")] // current U and V inversion for BOBs in beta elec
const ADC_I_SELECT: u32 = 0x18000100;
#[cfg(any(any(feature = "gamma", feature = "pvt"), feature = "pvt"))] // no inversions for gamma elec
const ADC_I_SELECT: u32 = 0x24000100;

const DS_ADC_MCFG_B_MCFG_A: u32 = 0x00100010;

// for drv8316 which are in the gamma elec for orbita3d use low-side
// better current sensing for lower ADC frequency (20Mhz)
// datasheet Table 9: Delta Sigma MCLK Configurations
#[cfg(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
))]
const DS_ADC_MCLK: u32 = 0x19000000;
// for other boards we can use the default value (25Mhz)
#[cfg(not(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
)))]
const DS_ADC_MCLK: u32 = 0x20000000;

const DS_ADC_MCLK_A: u32 = DS_ADC_MCLK;
const DS_ADC_MCLK_B: u32 = DS_ADC_MCLK;

// drv8316 which are in the gamma elec for orbita3d use low-side
// current sensing and require an exact synchronisation between the ADC and the
// PWM frequency
// The MDEC_A and MDEC_B should be set to 665 to be in sync with 25KHz pwm
// dataheet https://www.analog.com/media/en/technical-documentation/data-sheets/TMC4671_datasheet_v1.06.pdf (page 27, table 10.)
#[cfg(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
))]
const DS_ADC_MDEC_B_MDEC_A: u32 = 0x02990299;
// for other boards we can use the default value
#[cfg(not(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
)))]
const DS_ADC_MDEC_B_MDEC_A: u32 = 0x014E014E;

// full resolution of the ADC is 2^16 - 1
// bidirectional current measurement is used
// center is around 2^15 - 1
pub const ADC_RESOLUTION: f32 = 65535.0; // 16 bit
pub const ADC_OFFSET: f32 = 32767.0; // initial offset is half of the resolution

// ABN encoder settings
const ABN_DECODER_MODE: u32 = 0x00000000;
// const ABN_DECODER_PPR: u32 = 0x00001000;
const ABN_DECODER_PHI_E_PHI_M_OFFSET: u32 = 0x00000000;

// in TMC4671 one electrical revolution
// is always represented with 16 bits
pub const PPR_PER_ELECTRICAL_REVOLUTION: f32 = 65535.0; // 16 bit

// Limits
// const PID_TORQUE_FLUX_LIMITS: u32 = 0x00001000; // 4096
// const PID_TORQUE_FLUX_LIMITS: u32 = 0x00005a81; //Max: 58.9A
// const PID_TORQUE_FLUX_LIMITS: u32 = 0x00000b82; //2.9A

// const PID_TORQUE_FLUX_LIMITS: u32 = 0x00000800; // 2048

// const PID_VELOCITY_LIMIT: u32 = 0x0000_FFFF;
const PID_VELOCITY_LIMIT: u32 = 0x0000_7D00; //32000 //tuned ok at 500Hz
                                             // const PID_VELOCITY_LIMIT: u32 = 0x0000_0FA0; //4000

// Motor alignment
pub const OPENLOOP_ACCELERATION: u32 = 0x0000003c; // Wizard default
                                                   // pub const UQ_UD_EXT: u32 = 0x000007D0; // Openloop "torque_target" 2000

// for drv8316 which are in the gamma elec for orbita3d use low-side
// better motor encoder align if higher voltage is used (5000 out of 32000)
#[cfg(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
))]
pub const UQ_UD_EXT: u32 = 0x1388;
// for other boards we can use the default value (4000 out of 32000)
#[cfg(not(all(
    any(any(feature = "gamma", feature = "pvt"), feature = "pvt"),
    feature = "orbita3d"
)))]
pub const UQ_UD_EXT: u32 = 0x000001000;

// max is 32000
pub const UQ_UD_LIMIT: u32 = 31000;

#[allow(non_camel_case_types)]
#[allow(dead_code)]
pub enum Tmc4671Registers {
    CHIPINFO_DATA = 0x00,
    CHIPINFO_ADDR = 0x01,
    ADC_RAW_DATA = 0x02,
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
#[derive(Debug, Clone, Copy, Format)]
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
    pub fn from_u8(val: u8) -> Option<MotionMode> {
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

    pub fn to_u8(&self) -> u8 {
        match self {
            MotionMode::Stopped => 0,
            MotionMode::Torque => 1,
            MotionMode::Velocity => 2,
            MotionMode::Position => 3,
            MotionMode::PrbsFlux => 4,
            MotionMode::PrbsTorque => 5,
            MotionMode::PrbsVelocity => 6,
            MotionMode::PrbsPosition => 7,
            MotionMode::UqUdExt => 8,
            MotionMode::Reserved => 9,
            MotionMode::AgpiATorque => 10,
            MotionMode::AgpiAVelocity => 11,
            MotionMode::AgpiAPosition => 12,
            MotionMode::PmwITorque => 13,
            MotionMode::PmwIVelocity => 14,
            MotionMode::PmwIPosition => 15,
        }
    }
}

pub struct Foc<'d, T, P, EnablePin>
where
    T: Instance,
    P: Pin,
    EnablePin: Pin,
{
    spi: SpiDeviceWithConfig<'d, NoopRawMutex, Spi<'static, T, NoDma, NoDma>, Output<'static, P>>,

    pub(crate) enable: Output<'static, EnablePin>,
    //     #[allow(dead_code)]
    //     foc_status: Input<'d, FocStat>,
    pub brushless_motor_config: config::BrushlessMotor,
    pub current_sensing_config: config::CurrentSensing,
    pub ppr: f32,
    pub adc_resolution: f32,
    pub adc_vm_offset: f32,
    pub adc_temp_offset: f32,
}

impl<'d, T, P, EnablePin> Foc<'d, T, P, EnablePin>
where
    T: Instance,
    P: Pin,
    EnablePin: Pin,
{
    pub fn new(
        spi: SpiDeviceWithConfig<
            'd,
            NoopRawMutex,
            Spi<'static, T, NoDma, NoDma>,
            Output<'static, P>,
        >,
        enable: EnablePin,
        brushless_motor_config: config::BrushlessMotor,
        current_sensing_config: config::CurrentSensing,
    ) -> Self {
        let mut enable = Output::new(enable, Level::Low, Speed::Low);
        enable.set_low();

        Self {
            spi,
            enable,
            brushless_motor_config,
            current_sensing_config,
            ppr: PPR_PER_ELECTRICAL_REVOLUTION,
            adc_resolution: ADC_RESOLUTION,
            adc_vm_offset: ADC_OFFSET,
            adc_temp_offset: ADC_OFFSET,
        }
    }

    pub fn tmc4671_enable(&mut self) {
        self.enable.set_high();
    }

    pub fn tmc4671_disable(&mut self) {
        self.enable.set_low();
    }

    pub fn tmc4671_set_mode(&mut self, mode: MotionMode) -> Result<u32, embassy_stm32::spi::Error> {
        let mut data = 0x00000000u32;
        // read current state first
        self.tmc4671_transmit_raw_data(false, Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)?;
        data &= 0xFFFFFF00u32;
        data |= mode as u32;
        self.tmc4671_write_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)
    }

    pub fn tmc4671_set_pid_down(&mut self, down: u8) -> Result<u32, embassy_stm32::spi::Error> {
        let mut data = 0x00000000u32;
        // read current state first
        self.tmc4671_transmit_raw_data(false, Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)?;
        data &= 0x80FFFFFFu32;
        data |= down as u32;
        self.tmc4671_write_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)
    }
    pub fn tmc4671_set_pid_type(&mut self, pidtype: u8) -> Result<u32, embassy_stm32::spi::Error> {
        let mut data = 0x00000000u32;
        // read current state first
        self.tmc4671_transmit_raw_data(false, Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)?;
        data &= 0x7FFFFFFFu32;
        data |= pidtype as u32;
        self.tmc4671_write_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8, data)
    }

    pub fn tmc4671_get_adc_raw(&mut self) -> Result<(u16, u16), embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::ADC_RAW_ADDR as u8, 0x0)?;
        match self.tmc4671_get_u32(Tmc4671Registers::ADC_RAW_DATA as u8) {
            Ok(raw) => Ok(((raw & 0xFFFF) as u16, (raw >> 16) as u16)),
            Err(e) => Err(e),
        }
    }

    pub fn tmc4671_calibrate_adc_offsets(
        &mut self,
    ) -> Result<(u32, u32), embassy_stm32::spi::Error> {
        // read the adc raw values for finding the current offset (1000 times)
        let mut adc_offset: [u32; 2] = [0; 2];
        for _ in 0..1000 {
            self.tmc4671_get_adc_raw().map(|adc| {
                adc_offset[0] += adc.0 as u32;
                adc_offset[1] += adc.1 as u32;
            })?;
        }
        // divide by 1000 to get the average
        adc_offset[0] /= 1000;
        adc_offset[1] /= 1000;

        // set the new offset values
        self.current_sensing_config
            .set_adc_offsets(adc_offset[0], adc_offset[1]);

        self.tmc4671_checked_write(
            Tmc4671Registers::ADC_I0_SCALE_OFFSET as u8,
            0x01000000 | adc_offset[0],
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::ADC_I1_SCALE_OFFSET as u8,
            0x01000000 | adc_offset[1],
        )?;

        Ok((adc_offset[0] as u32, adc_offset[1] as u32))
    }

    #[cfg(feature = "beta")]
    fn adc_to_temperature(&self, adc_raw: u32) -> Result<f32, IOError> {
        // datasheet BOB: https://www.mouser.fr/datasheet/2/281/r44e-522712.pdf  (NCP18XH103F03RB)
        // ADC is operated in a single ended mode it measures the voltage from 0 to 2.5V
        // the 10k NTC is supplied with 3.3V and pulled down to ground with two 4.7k resistor
        // the voltage is measured at the center of the two resistors
        // the temperature reading is very bad on TMC4761 for low temperatures especially
        // but for higher temperatures it is quite accurate (above 60°C) - good for security
        // - empirically tested
        let volt = (adc_raw as f32 - self.adc_temp_offset as f32) / 65535.0 * 5.0;
        let r_div: f32 = 4700.0;
        let beta: f32 = 3455.0;
        let room_temp_inv: f32 = 1.0 / 298.15; //[K]
        let r_t: f32 = r_div * (3.3 / volt - 2.0); // estimated resistance of the NTC
        let r_25: f32 = 10000.0;
        let t: f32 = 1.0 / (((libm::log((r_t / r_25) as f64) as f32) / beta) + room_temp_inv);

        match t {
            t if t.is_nan() => Err(IOError::InvalidData),
            _ => {
                let mut t_celsius = (t as f32) - 273.15;
                // a seemingly constant linear error
                // empirically tested correction
                t_celsius = (t_celsius - 1.75297) / 1.0987;
                Ok(t_celsius) // final conversion to Celsius
            }
        }
    }
    #[cfg(any(any(feature = "gamma", feature = "pvt"), feature = "pvt"))]
    fn adc_to_temperature(&self, adc_raw: u32) -> Result<f32, IOError> {
        // NTCS0603E3103FMT 10k
        // https://www.vishay.com/docs/29056/ntcs0603e3t.pdf
        let volt = (adc_raw as f32 - self.adc_temp_offset as f32) / 65535.0 * 5.0;
        let r_div: f32 = 4700.0;
        let beta: f32 = 3610.0;
        let room_temp_inv: f32 = 1.0 / 298.15; //[K]
        let r_t: f32 = r_div * ((3.3 / volt) - 1.0); // estimated resistance of the NTC
        let r_25: f32 = 10000.0;
        let t: f32 = 1.0 / (((libm::log((r_t / r_25) as f64) as f32) / beta) + room_temp_inv);

        match t {
            t if t.is_nan() => Err(IOError::InvalidData),
            _ => {
                let mut t_celsius = (t as f32) - 273.15;
                // a seemingly constant linear error
                // empirically tested correction
                // https://www.notion.so/pollen-robotics/Overtemperature-protection-91d2e7de1c5e4745a34cd67209dca090?pvs=4
                t_celsius = (t_celsius - 1.75297) / 1.0987;
                Ok(t_celsius) // final conversion to Celsius
            }
        }
    }

    pub fn tmc4671_get_board_temperature(&mut self) -> Result<(f32), embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::ADC_RAW_ADDR as u8, 0x2)?;

        match self.tmc4671_get_u32(Tmc4671Registers::ADC_RAW_DATA as u8) {
            Ok(raw) => {
                match self.adc_to_temperature((raw & 0xffff) as u32) {
                    Ok(temp) => Ok(temp),
                    Err(e) => Err(embassy_stm32::spi::Error::Framing), // send the math error as a framing error
                }
            }
            Err(e) => Err(e),
        }
    }

    pub fn tmc4671_get_bus_voltage(&mut self) -> Result<(f32), embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::ADC_RAW_ADDR as u8, 0x1)?;
        match self.tmc4671_get_u32(Tmc4671Registers::ADC_RAW_DATA as u8) {
            Ok(raw) => {
                let adc_raw = (raw & 0xffff) as f32; // extract the raw value
                let voltage = (adc_raw - self.adc_vm_offset) / 32768.0 * 2.5; // scale to 0-2.5V
                #[cfg(feature = "beta")]
                let voltage = voltage / (1.0 / (47.0 + 1.0)); // 47k/1k voltage divider (BOB)
                #[cfg(any(any(feature = "gamma", feature = "pvt"), feature = "pvt"))]
                let voltage = voltage / (4.7 / (75.0 + 4.7)); // 75k/4.7k voltage divider (gamma elec ventouse 2d/3d)

                // a seemingly constant linear error
                // empirically tested correction
                // https://www.notion.so/pollen-robotics/Low-bus-voltage-protection-78fead7508fe4027a00669618095f125?pvs=4

                #[cfg(feature = "beta")]
                let voltage = (voltage - 0.8580) / 1.0285;
                #[cfg(any(any(feature = "gamma", feature = "pvt"), feature = "pvt"))]
                let voltage = (voltage - 6.1204) / 0.8779;
                Ok(voltage)
            }
            Err(e) => Err(e),
        }
    }

    pub fn tmc4671_calibrate_general_purpose_adc_offsets(
        &mut self,
        samples: u32,
    ) -> Result<(), embassy_stm32::spi::Error> {
        // calibrate the adc for temperature sensing
        // set the 0 voltage to the AGPI_B and ADC_VM
        // the center should be at VDD/2 (0x5) as the ADCs measure -2.5 to 2.5V
        self.tmc4671_checked_write(Tmc4671Registers::DS_ANALOG_INPUT_STAGE_CFG as u8, 0x54500)?;
        self.adc_temp_offset = 0.0;
        self.adc_vm_offset = 0.0;
        for i in 0..samples {
            // read AGPI_B for temperature
            self.tmc4671_checked_write(Tmc4671Registers::ADC_RAW_ADDR as u8, 0x2)?;
            match self.tmc4671_read_register(Tmc4671Registers::ADC_RAW_DATA as u8) {
                Ok(raw) => self.adc_temp_offset += (raw & 0xffff) as f32,
                Err(e) => {
                    error!("!!! Error SPI {:?}!!!", e);
                }
            }
            // read ADC_VM for bus voltage
            self.tmc4671_checked_write(Tmc4671Registers::ADC_RAW_ADDR as u8, 0x1)?;
            match self.tmc4671_read_register(Tmc4671Registers::ADC_RAW_DATA as u8) {
                Ok(raw) => self.adc_vm_offset += (raw & 0xffff) as f32,
                Err(e) => {
                    error!("!!! Error SPI {:?}!!!", e);
                }
            }
        }
        self.adc_temp_offset /= samples as f32;
        self.adc_vm_offset /= samples as f32;
        debug!(
            "General purpose ADC offsets calibrated temperature: {}, DC bus voltage {}",
            self.adc_temp_offset, self.adc_vm_offset
        );
        // start measuring with ADC_VM and AGPI_B again
        self.tmc4671_checked_write(Tmc4671Registers::DS_ANALOG_INPUT_STAGE_CFG as u8, 0x44400)?;

        Ok(())
    }

    pub fn tmc4671_get_pid_flux(&mut self) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_get_u32(Tmc4671Registers::PID_FLUX_P_FLUX_I as u8)
    }
    pub fn tmc4671_get_pid_torque(&mut self) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_get_u32(Tmc4671Registers::PID_TORQUE_P_TORQUE_I as u8)
    }
    pub fn tmc4671_get_pid_velocity(&mut self) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_get_u32(Tmc4671Registers::PID_VELOCITY_P_VELOCITY_I as u8)
    }
    pub fn tmc4671_get_pid_position(&mut self) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_get_u32(Tmc4671Registers::PID_POSITION_P_POSITION_I as u8)
    }

    pub fn tmc4671_set_pid_flux(&mut self, pid: u32) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::PID_FLUX_P_FLUX_I as u8, pid)
    }

    pub fn tmc4671_set_pid_torque(&mut self, pid: u32) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::PID_TORQUE_P_TORQUE_I as u8, pid)
    }

    pub fn tmc4671_set_pid_velocity(&mut self, pid: u32) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_P_VELOCITY_I as u8, pid)
    }

    pub fn tmc4671_set_pid_position(&mut self, pid: u32) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::PID_POSITION_P_POSITION_I as u8, pid)
    }

    pub fn tmc4671_get_uq_ud_limit(&mut self) -> Result<i16, embassy_stm32::spi::Error> {
        self.tmc4671_get_lower_i16(Tmc4671Registers::PIDOUT_UQ_UD_LIMITS as u8)
    }
    pub fn tmc4671_set_uq_ud_limit(
        &mut self,
        limit: i16,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(
            Tmc4671Registers::PIDOUT_UQ_UD_LIMITS as u8,
            limit as u32 & 0x7FFF as u32,
        )
    }

    pub fn tmc4671_get_torque_flux_limit(&mut self) -> Result<u16, embassy_stm32::spi::Error> {
        self.tmc4671_get_lower_u16(Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8)
    }
    pub fn tmc4671_set_torque_flux_limit(
        &mut self,
        limit: u16,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(
            Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8,
            limit as u32 & 0x7FFF as u32,
        )
    }

    pub fn tmc4671_get_velocity_limit(&mut self) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_read_register(Tmc4671Registers::PID_VELOCITY_LIMIT as u8)
    }
    pub fn tmc4671_set_velocity_limit(
        &mut self,
        limit: u32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(Tmc4671Registers::PID_VELOCITY_LIMIT as u8, limit)
    }

    pub fn tmc4671_get_mode(&mut self) -> Result<MotionMode, embassy_stm32::spi::Error> {
        match self.tmc4671_read_register(Tmc4671Registers::MODE_RAMP_MODE_MOTION as u8) {
            Ok(read) => {
                Ok(
                    MotionMode::from_u8((read & 0x000000FFu32) as u8)
                        .unwrap_or(MotionMode::Stopped),
                ) //Hu??
            }
            Err(_e) => {
                // error!("!!! Error SPI {:?}!!!", e);
                Err(embassy_stm32::spi::Error::Framing)
            }
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
        //TODO Conversion
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

    pub fn tmc4671_set_target_torque(
        &mut self,
        torque_target: i32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_set_torque_target(torque_target as i16)
    }

    pub fn tmc4671_get_target_torque(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_TORQUE_FLUX_TARGET as u8)
    }

    pub fn tmc4671_get_actual_velocity(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        //TODO conversion
        self.tmc4671_get_i32(Tmc4671Registers::PID_VELOCITY_ACTUAL as u8)
    }

    //Mainly for init
    pub fn tmc4671_set_actual_position(
        &mut self,
        position_actual: i32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(
            Tmc4671Registers::PID_POSITION_ACTUAL as u8,
            position_actual as u32,
        )
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

    pub fn tmc4671_set_velocity_offset(
        &mut self,
        velocity_offset: i32,
    ) -> Result<u32, embassy_stm32::spi::Error> {
        self.tmc4671_write_register(
            Tmc4671Registers::PID_VELOCITY_OFFSET as u8,
            velocity_offset as u32,
        )
    }
    pub fn tmc4671_get_velocity_offset(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_VELOCITY_OFFSET as u8)
    }

    pub fn tmc4671_get_target_position(&mut self) -> Result<i32, embassy_stm32::spi::Error> {
        self.tmc4671_get_i32(Tmc4671Registers::PID_POSITION_TARGET as u8) //TODO should probably check INTERIM_DATA
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
        self.tmc4671_write_register(Tmc4671Registers::ABN_DECODER_PPR as u8, ppr as u32)
    }

    pub async fn tmc4671_init_registers(&mut self) -> Result<(), embassy_stm32::spi::Error> {
        // Motor type & PWM configuration
        self.tmc4671_checked_write(
            Tmc4671Registers::MOTOR_TYPE_N_POLE_PAIRS as u8,
            self.brushless_motor_config.motor_type_n_pole_pairs(),
        )?;
        self.tmc4671_checked_write(Tmc4671Registers::PWM_POLARITIES as u8, PWM_POLARITIES)?;
        self.tmc4671_checked_write(Tmc4671Registers::PWM_MAXCNT as u8, PWM_MAXCNT)?;
        self.tmc4671_checked_write(Tmc4671Registers::PWM_BBM_H_BBM_L as u8, PWM_BBM_H_BBM_L)?;
        self.tmc4671_checked_write(Tmc4671Registers::PWM_SV_CHOP as u8, PWM_SV_CHOP)?;

        //PID
        // self.tmc4671_set_pid_down(2)?;

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
            self.current_sensing_config.adc_i1_scale_offset(),
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::ADC_I0_SCALE_OFFSET as u8,
            self.current_sensing_config.adc_i0_scale_offset(),
        )?;

        // ABN encoder settings
        self.tmc4671_checked_write(Tmc4671Registers::ABN_DECODER_MODE as u8, ABN_DECODER_MODE)?;
        self.tmc4671_checked_write(
            Tmc4671Registers::ABN_DECODER_PPR as u8,
            self.brushless_motor_config.abn_decoder_ppr(),
        )?;
        self.tmc4671_checked_write(
            Tmc4671Registers::ABN_DECODER_PHI_E_PHI_M_OFFSET as u8,
            ABN_DECODER_PHI_E_PHI_M_OFFSET,
        )?;

        // set uq and ud limits
        self.tmc4671_checked_write(Tmc4671Registers::PIDOUT_UQ_UD_LIMITS as u8, UQ_UD_LIMIT)?;

        // Limits
        //        self.tmc4671_checked_write(Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8, 0x00007D00)?; // 32000
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_TORQUE_FLUX_LIMITS as u8,
            self.current_sensing_config.mAmps_to_adc(
                self.brushless_motor_config.torque_flux_limit_max() as f32,
                self.adc_resolution,
            ) as u32,
        )?;

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

        //Limite the vel
        self.tmc4671_checked_write(
            Tmc4671Registers::PID_VELOCITY_LIMIT as u8,
            self.brushless_motor_config
                .angle_mech_to_elec(conversion::rads_to_rpm(
                    self.brushless_motor_config.velocity_limit_max() as f32,
                )) as u32,
        )?;

        // calibrate the adc for temperature and dc bus voltage sensing
        // takes the number of samples for the calibration in the argument
        self.tmc4671_calibrate_general_purpose_adc_offsets(100)?;

        Ok(())
    }

    pub(crate) fn tmc4671_checked_write(
        &mut self,
        reg: u8,
        data_w: u32,
    ) -> Result<(), embassy_stm32::spi::Error> {
        self.tmc4671_write_register(reg, data_w)?;
        let data_r = self.tmc4671_read_register(reg)?;
        if data_r == data_w {
            Ok(())
        } else {
            error!(
                "!!! TMC4671 Error checked write addr: {:#x} {:#x}_r / {:#x}_w !!!",
                reg, data_r, data_w
            );
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

    fn tmc4671_get_lower_u16(&mut self, reg: u8) -> Result<u16, embassy_stm32::spi::Error> {
        if let Ok(res) = self.tmc4671_read_register(reg) {
            Ok((res & 0x0000FFFFu32) as u16)
        } else {
            Err(embassy_stm32::spi::Error::Framing)
        }
    }

    fn tmc4671_get_upper_u16(&mut self, reg: u8) -> Result<u16, embassy_stm32::spi::Error> {
        match self.tmc4671_read_register(reg) {
            Ok(res) => Ok(((res & 0xFFFF0000u32) >> 16) as u16),
            Err(e) => Err(e),
        }
    }
}
