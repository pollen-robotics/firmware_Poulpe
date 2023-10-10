use crate::count_items;
use crate::define_register_map;
use crate::registers::AccessType;

use crate::paste;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

define_register_map!(
    dxl_registers,
    DxlRegistersEnum,
    dxl_registers_buffer,
    Mutex < ThreadModeRawMutex,[u8; 256] >,
    1, // Word size for the entire register map
    Reg1, 0, 2, AccessType::ReadWrite,
    Reg2, 4, 4, AccessType::ReadWrite
);
