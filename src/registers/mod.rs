// use defmt::*;
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
// use embassy_sync::mutex::Mutex;

// use crate::paste;
// use paste::paste;

use {defmt_rtt as _, panic_probe as _};
// pub struct Registers {
//     pub buffer: [u8; 512],
// }

// pub static REGISTERS: Mutex<ThreadModeRawMutex, Registers> =
//     Mutex::new(Registers { buffer: [0; 512] });

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccessType {
    ReadOnly,
    WriteOnly,
    ReadWrite,
}

#[derive(Debug, Clone, Copy)]
pub struct Register<T> {
    pub name: T,
    pub address: usize,
    pub byte_size: u8,
    pub access_type: AccessType,
}

#[macro_export]
macro_rules! define_register_map {
    ($map_struct_name:ident, $enum_name:ident, $buffer:ident, $mutex_type:ty, $word_size:expr, $($name:ident, $address:expr, $byte_size:expr, $access_type:expr),+ $(,)?) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum $enum_name {
            $($name),+
        }

        pub static $buffer: $mutex_type = Mutex::new([0u8; 256]);

        pub static $map_struct_name: [$crate::registers::Register<$enum_name>; {count_items!($($name),+)}] = [
	    $($crate::registers::Register { name: $enum_name::$name, address: $address, byte_size: $byte_size, access_type: $access_type }
	    ),+

        ];

        paste! {

	    #[allow(dead_code)]
	    pub fn [<$map_struct_name:lower _get_size>](register: $enum_name) -> usize {
		let reg = &$map_struct_name[register as usize];
		reg.byte_size.into()
	    }

	    #[allow(dead_code)]
	    pub async fn [<$map_struct_name:lower _read_by_name>](register: $enum_name, buffer: &mut [u8]) -> Result<usize, ()> {
		let reg = &$map_struct_name[register as usize];
		if reg.access_type == AccessType::WriteOnly {
		    return Err(());
		}
		let start = reg.address * $word_size;
		let end = start + reg.byte_size as usize * $word_size;
		let buf = $buffer.lock().await;
		if buffer.len() < (end - start) {
		    return Err(()); // or handle buffer too small differently
		}
		buffer[..(end - start)].copy_from_slice(&buf[start..end]);
		Ok(end - start)
	    }

	    #[allow(dead_code)]
            pub async fn [<$map_struct_name:lower _write_by_name>](register: $enum_name, value: &[u8]) -> Result<(), ()> {
                let reg = &$map_struct_name[register as usize];
                if reg.access_type == AccessType::ReadOnly || value.len() != reg.byte_size as usize * $word_size {
                    return Err(());
                }
                let start = reg.address * $word_size;
                let end = start + reg.byte_size as usize * $word_size;
                let mut buffer = $buffer.lock().await;
                buffer[start..end].copy_from_slice(value);
                Ok(())
            }
	    #[allow(dead_code)]
	    pub async fn [<$map_struct_name:lower _read_by_address>]( //TODO Check that all the segment is READ
                address: usize, word_count: usize, buffer: &mut [u8]
            ) -> Result<(), ()> {
                let end_address = address + word_count * $word_size;
                for register in &$map_struct_name {
                    let start = register.address * $word_size;
                    let end = start + word_count as usize * $word_size;
                    if address >= start && address < end && end_address <= end {
                        let buf = $buffer.lock().await;
                        buffer[0..word_count as usize * $word_size].copy_from_slice(&buf[start..end_address]);
                        return Ok(());
                    }
                }
                Err(()) // Address not found or out of bounds
            }
	    #[allow(dead_code)]
            pub async fn [<$map_struct_name:lower _write_by_address>]( //TODO Check that all the segment is WRITE
                address: usize, word_count: usize, value: &[u8]
            ) -> Result<(), ()> {
                let end_address = address + word_count * $word_size;
                for register in &$map_struct_name {
                    let start = register.address * $word_size;
                    let end = start + register.byte_size as usize * $word_size;
                    if address >= start && address < end && end_address <= end {
                        let mut buf = $buffer.lock().await;
                        buf[start..end_address].copy_from_slice(value);
                        return Ok(());
                    }
                }
                Err(()) // Address not found or out of bounds
            }


        }
    };
}
#[macro_export]
macro_rules! count_items {
    ($($item:ident),+ $(,)?) => { 0 $(+ { stringify!($item); 1 })+ };
}
