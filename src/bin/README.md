# Simple programs testing/using the library

This directory contains simple programs that test or use the library. They are used to test the library and to provide examples of how to use the library and test the hardware.

## Testing Programs

Testing programs are used to test the library and the hardware, they avoid having to lanuch the full project to test a specific part of the library or the hardware.

Orbita3d sensor testing       
- `test_Hall.rs`  - testing I2C hall sensors
- `test_DonutSensors.rs` - Testing the SPI as5027d sensors

Orbita2d sensor testing 
- `test_Center.rs`  - Testing the SPI as5027d sensor
- `test_Ring.rs`    - Testing the SPI Aksim2 sensor

Temperature sensor testing
- `test_Temperature.rs` - ouput the temperature of the NTC sensors of the motor

Testing the motor control on one motor - connected to the `B` port
- `test_MotorControl.rs`  - Do a sine wave with the motor on the `B` port


## Bench Programs

Bench programs are used to do standarized tests on the library and the hardware. They are used to test and verify the performance of the library and the hardware.

Verify that the motor has not too much vibration when it is not moving.
Testing the motor in torque mode with 500mA target. 
- `bench_MotorTest.rs`