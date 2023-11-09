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
