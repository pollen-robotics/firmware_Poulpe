#![no_main]
#![no_std]


use firmware_poulpe::dynamixel::conversion::{bytes_to_bool, bool_to_bytes, bytes_to_float, float_to_bytes, u32_to_bytes, bytes_to_u32, bytes_to_pid, pid_to_bytes};
use firmware_poulpe::motor_control;
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::{bind_interrupts};

use firmware_poulpe::motor_control::ventouse::conversion;


fn is_close(a: f32, b: f32, tol: f32) -> bool {
    if a>b {
        return (a-b) < tol
    }else{
        return (b-a) < tol
    }
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
        let pid = motor_control::Pid{p: 100, i: 200};
        let result = pid_to_bytes::<1>([pid]);
        let pid2 = bytes_to_pid::<1>(&result)[0];
        assert!(pid.p == pid2.p && pid.i == pid2.i );
    }

    // test conversion from rads to encoder values
    #[test]
    fn test_rads_to_encoder() {
        let rads = 3.0;
        let ppr = 4096.0;
        let enc = conversion::rad_to_encoder(rads, ppr);
        let rads2 = conversion::encoder_to_rad(enc, ppr);
        assert!(is_close(rads,rads2,0.01));
    }

    // rest conversion from rpm to rads
    #[test]
    fn test_rpm_to_rads() {
        let rpm = 100.0;
        let rads = conversion::rpm_to_rads(rpm);
        let rpm2 = conversion::rads_to_rpm(rads);
        assert!(is_close(rpm,rpm2,0.001));
    }

}