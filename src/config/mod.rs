use crate::count_items;
use crate::define_register_map;
use crate::registers::AccessType;

use crate::paste;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

pub static DXL_ID: u8 = 42;

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
);
