use crate::count_items;
use crate::define_register_map;
use crate::registers::AccessType;

use crate::paste;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
pub static DXL_ID: u8 = 42;

#[cfg(feature = "ecx22")]
pub mod motor {
    pub const PID_FLUX_P_FLUX_I: u32 = 0x03200080;
    pub const PID_TORQUE_P_TORQUE_I: u32 = 0x03200000;
    pub const PID_VELOCITY_P_VELOCITY_I: u32 = 0x01000080;
    pub const PID_POSITION_P_POSITION_I: u32 = 0x00400010;
}

#[cfg(feature = "ec60")]
pub mod motor {
    pub const PID_FLUX_P_FLUX_I: u32 = 0x03200000;
    pub const PID_TORQUE_P_TORQUE_I: u32 = 0x03200000;
    pub const PID_VELOCITY_P_VELOCITY_I: u32 = 0x01F401C2;
    pub const PID_POSITION_P_POSITION_I: u32 = 0x00500000;
}

define_register_map!(
    DXL_REGISTERS,
    DxlRegistersEnum,
    DXL_REGISTERS_BUFFER,
    Mutex < ThreadModeRawMutex,[u8; 256] >,
    1, // Word size for the entire register map
    //register_name, address, size, access
    ModelNumber, 0, 2, AccessType::ReadOnly,
    FirmwareRev, 6, 1,  AccessType::ReadOnly,
    Id, 7, 1, AccessType::ReadWrite,
    SystemCheck, 8, 1, AccessType::WriteOnly,
    VoltageLimit, 10, 4, AccessType::ReadWrite,
    IntensityLimit, 14, 4, AccessType::ReadWrite,
    VelocityPID, 18, 12, AccessType::ReadWrite,
    VelocityPGain, 18, 4, AccessType::ReadWrite,
    VelocityIGain, 22, 4, AccessType::ReadWrite,
    VelocityDGain, 26, 4, AccessType::ReadWrite,
    VelocityRampOut, 30, 4, AccessType::ReadWrite,
    SensorRingPresentPosition, 67, 4, AccessType::ReadOnly,
    SensorCenterPresentPosition, 71, 4, AccessType::ReadOnly,

    MotorAGoalPosition, 75, 4, AccessType::ReadWrite,
    MotorBGoalPosition, 79, 4, AccessType::ReadWrite,
    MotorAPresentPosition, 83, 4, AccessType::ReadWrite,
    MotorBPresentPosition, 87, 4, AccessType::ReadWrite,



);

/*
#[macro_export]
macro_rules! define_uart {
    ($uart_name:ident, $usart:ident, $rx_pin:ident, $tx_pin:ident, $irq:ident, $tx_dma:ident, $rx_dma:ident) => {
        pub fn $uart_name(p: embassy_stm32::Peripherals) -> Uart {
            // use embassy_stm32::config::Config;

            let mut config = Config::default();
            config.baudrate = 1_000_000;
            config.detect_previous_overrun = false;

            Uart::new(
                p.$usart, p.$rx_pin, p.$tx_pin, $irq, p.$tx_dma, p.$rx_dma, config,
            )
        }
    };
}

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

define_uart!(dxl_uart, USART1, PB15, PB14, Irqs, DMA1_CH0, DMA1_CH1);
*/
