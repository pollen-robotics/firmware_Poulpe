#![no_main]
#![no_std]
#![feature(generic_const_exprs)]

#[cfg(not(test))]
use crate::motor_control::Pid;
#[cfg(test)]
use firmware_poulpe::motor_control::Pid;

pub fn bytes_to_bool<const N: usize>(data: &[u8]) -> [bool; N] {
    assert!(data.len() == N);
    let mut result = [false; N];
    for i in 0..N {
        result[i] = data[i] != 0;
    }
    result
}

pub fn bool_to_bytes<const N: usize>(data: [bool; N]) -> [u8; N] {
    let mut result = [0; N];

    for i in 0..N {
        result[i] = data[i] as u8;
    }

    result
}

pub fn bytes_to_float<const N: usize>(data: &[u8]) -> [f32; N] {
    assert!(data.len() == N * 4);
    let mut result = [0.0; N];
    for i in 0..N {
        result[i] = f32::from_le_bytes(data[i * 4..(i + 1) * 4].try_into().unwrap());
    }
    result
}

pub fn float_to_bytes<const N: usize>(data: [f32; N]) -> [u8; 4 * N] {
    let mut result = [0; 4 * N];

    for i in 0..N {
        result[i * 4..(i + 1) * 4].copy_from_slice(&data[i].to_le_bytes());
    }

    result
}


pub fn u32_to_bytes<const N: usize>(data: [u32; N]) -> [u8; 4 * N] {
    let mut result = [0; 4 * N];

    for i in 0..N {
        result[i * 4..(i + 1) * 4].copy_from_slice(&data[i].to_le_bytes());
    }

    result
}


pub fn bytes_to_u32<const N: usize>(data: &[u8]) -> [u32; N] {
    assert!(data.len() == N * 4);
    let mut result = [0; N];
    for i in 0..N {
        result[i] = u32::from_le_bytes(data[i * 4..(i + 1) * 4].try_into().unwrap());
    }
    result
}



pub fn u16_to_bytes<const N: usize>(data: [u16; N]) -> [u8; 2 * N] {
    let mut result = [0; 2 * N];

    for i in 0..N {
        result[i * 2..(i + 1) * 2].copy_from_slice(&data[i].to_le_bytes());
    }

    result
}


pub fn bytes_to_u16<const N: usize>(data: &[u8]) -> [u16; N] {
    assert!(data.len() == N * 2);
    let mut result = [0; N];
    for i in 0..N {
        result[i] = u16::from_le_bytes(data[i * 2..(i + 1) * 2].try_into().unwrap());
    }
    result
}


pub fn i16_to_bytes<const N: usize>(data: [i16; N]) -> [u8; 2 * N] {
    let mut result = [0; 2 * N];

    for i in 0..N {
        result[i * 2..(i + 1) * 2].copy_from_slice(&data[i].to_le_bytes());
    }

    result
}


pub fn bytes_to_i16<const N: usize>(data: &[u8]) -> [i16; N] {
    assert!(data.len() == N * 2);
    let mut result = [0; N];
    for i in 0..N {
        result[i] = i16::from_le_bytes(data[i * 2..(i + 1) * 2].try_into().unwrap());
    }
    result
}


pub fn pid_to_bytes<const N:usize>(pid: [Pid;N]) -> [u8; 4*N] {
    let mut result = [0; 4 * N];
    for i in 0..N {
	let rawpid=(pid[i].p as u32) << 16 | (pid[i].i as u32) ;
	result[i * 4..(i + 1) * 4].copy_from_slice(&rawpid.to_le_bytes());
    }
    result

}

pub fn bytes_to_pid<const N:usize>(data: &[u8]) -> [Pid;N] {
    assert!(data.len() == N * 4); //FIXME: remove assert!!
    let mut result = [Pid{p:0,i:0}; N];

    for i in 0..N {

	let rawpid = u32::from_le_bytes(data[i * 4..(i + 1) * 4].try_into().unwrap());
	let pid=Pid {
		p: ((rawpid >> 16) as i16) & 0x7FFF,
		i: (rawpid as i16) & 0x7FFF,
	};

        result[i] = pid;
    }
    result

}


// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::{assert, debug};

    use super::*;

    // test conversion from bytes to bool
    #[test]
    fn test_bytes_to_bool() {
        let data = [0, 1, 0, 1, 0, 1, 0, 1];
        let result = bytes_to_bool::<8>(&data);
        assert!(result == [false, true, false, true, false, true, false, true]);
    }

    // test conversion from bool to bytes
    #[test]
    fn test_bool_to_bytes() {
        let data = [false, true, false, true, false, true, false, true];
        let result = bool_to_bytes::<8>(data);
        assert!(result == [0, 1, 0, 1, 0, 1, 0, 1]);
    }

    // test conversion from float to bytes
    #[test]
    fn test_float_to_bytes() {
        let data = [0.5, 0.7];
        let result = float_to_bytes::<2>(data);
        assert!(data == bytes_to_float::<2>(&result));
    }

    // test conversion from u32 to bytes
    #[test]
    fn test_u32_to_bytes() {
        let data = [100, 589];
        let result = u32_to_bytes::<2>(data);
        assert!(data == bytes_to_u32::<2>(&result));
    }
    
    // test conversion from pid to bytes
    #[test]
    fn test_pid_to_bytes() {
        let pid = Pid{p: 100, i: 200};
        let result = pid_to_bytes::<1>([pid]);
        let pid2 = bytes_to_pid::<1>(&result)[0];
        assert!(pid.p == pid2.p && pid.i == pid2.i );
    }

}