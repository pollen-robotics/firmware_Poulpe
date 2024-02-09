#![no_main]
#![no_std]


use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::{bind_interrupts};

use firmware_poulpe::motor_control;
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