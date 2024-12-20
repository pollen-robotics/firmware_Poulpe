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

Bench programs are inteded to be run for hardware testing in the benchtop setup to verify the hardware is working correctly. As well as to configure the hardware.

### Write axis zeros to FLASH memory (configure the actuator)
Writing the axis zeros to FLASH memory, the code will read the axis sensors, transform the values to the axis values and write them to the flash memory.
- `bench_Orbita2dWriteZeros.rs` - Write the zeros to the flash memory of the orbita2d board
- `bench_Orbita3dWriteZeros.rs` - Write the zeros to the flash memory of the orbita3d board

**IMPORTANT** The zeros are written to the flash memory, so they are persistent. Make sure to run the program only once, or the zeros will be overwritten. Make sure to position the orbita in its zero position before running the program.

The code will write the axis zeros as well as the DXL_ID to the flash, it will write the id 42 by default, if you want to change the id you can do so by adding the `DXL_ID` environment variable the call of the program.

For example, to write the zeros to the flash memory of the orbita3d (pvt version) board with the `DXL_ID` 56:
```
DEFMT_LOG=info DXL_ID=56 cargo run --release --features orbita3d_pvt --bin bench_Orbita3dWriteZeros
```

### Test the orbita2d on assembly (verify that there is not too tight)

This code will run both motors of the orbita2d board in torque mode with 400mA target, which should be enough to move the assembly. The code will run the motors will change direction each 5 seconds. The goal of this code is to be a visual check for the assembly personeel to verify that the assembly is not too tight. If the motors stop moving, it means that the assembly is too tight.

- `bench_Orbita2dAssemblyTest.rs` 


### Test the motor in torque mode (verify the motor is working correctly)

Verify that the motor has not too much vibration when it is not moving.
Testing the motor in torque mode with 500mA target. 
- `bench_MotorTest.rs`


## Running the programs

```bash
cargo run --release --features <orbita version> --bin <program_name>
```

Make sure to select the correct orbita version for the program you are running using the `--features` flag. See the list of orbita versions in the `Cargo.toml` file or in the docs [see the main README](../../README.md).

For example
```sh
cargo run --release --features orbita3d_pvt --bin test_Hall
```