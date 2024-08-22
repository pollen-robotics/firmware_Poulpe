# Configuration crate of the poule project

This crate contains the configuration of the poule project. It is used to store the configuration of the different crates and to provide a common interface to access the configuration parameters.

It has three main modules:
- `BrushlessMotor`: Contains the configuration of the BLDC motor control crate
- `CurrentSense` : Contains the configuration of the current sense crate
- `Flash` : Contains the configuration of the flash memory crate

## BrushlessMotor
Brushless motor control configuration module. It contains the following configuration parameters:
```rust
// number of pole pairs
n_pole_pairs: u32,

// PID gains of the motor controllers
// using only P and I gains
pid_flux: Pid,
pid_torque: Pid,
pid_velocity: Pid,
pid_position: Pid,
torque_flux_limit_max: u32,  // milliAmps
velocity_limit_max: u32,   // rad/s at the output of the gearbox/axis/motor - depends on the features enabled
// The encoder PPR value - register ABN_DECODER_PPR
abn_decoder_ppr: u32,
// ratio of motor's gearbox
gearbox_ratio: f32,
// additional reduction ration of the axis
axis_ratio: f32
```
these parameters are used to configure the motor controller for given motor. The `BrushlessMotor` struct has three different constructors, one for each supported motor type:
- `EC60` - constructor `ec60()`
- `EC45` - constructor `ec45()`
- `ECX22` - constructor `ecx22()`
Each of them sets the default values for the given motor type and is enabled by passing the feature flag to the `cargo` command.

Additionally the `BrushlessMotor` struct enables transforming the agle recaived form the TMC4671 to the motor's position and velocity at different points in the system:
- The position of the gearbox output - `gearbox_output`
- The position of the axis output - `axis_output`
- The position of motor's output shaft - if neither `gearbox_output` nor `axis_output`

Deciding if the poulpe will control the motor ouptut at the shaft, axis, or gearbox output is done by setting the appropriate feature in the `Cargo.toml` file. 

## CurrentSense
Current sense configuration module. It contains the following configuration parameters:
```rust
// current sensing parameters
// Shunt resistor value
resistance_shunt: f32, // [Ohms]
// gain of the amplifier
amp_gain: f32,    // [V/V]
amp_voltage: f32, // [V]

// adc offset and scale values - register ADC_I0_SCALE_OFFSET and ADC_I1_SCALE_OFFSET
adc_i0_scale_offset: u32,
adc_i1_scale_offset: u32,
```

The `CurrentSense` struct is used to configure the current sense module. It is used to calculate the actual current value from the ADC readings. 


## Flash memory configuration

Flash memory is configured, writen and read using the `Flashmanager` struct.
While the `FlashData` contains the data that is stored in the flash memory. The `FlashData` struct has the following fields:
```rust
board_id: u32, // dynamixel id of the board
sensor_offset: [f32; N_AXIS], // sensor offset values (2 values for orbit2d and 3 for orbit3d)
```
This structure can be easiliy extended in the future to store more data in the flash memory.

For the moment all the data is stored to a fixed location:
```rust
// the address of the 5th sector of the flash memory
// it can be any other sector that is not used by the program
const ADDR: u32 = 5*128*1024; // This is the offset into bank 1
```


## Global configuration

In addition to the configuration structures, it has several safety tresholds that are used to determine the safety status of the board. The safety tresholds are as follows:
```rust
// maximal temperature limits for the motor and the boards
// high temeperature state - only warning
pub const HIGH_TEMP: f32 = 65.0;
// maximal motor temperature - error state if above
pub const MAX_MOTOR_TEMP: f32 = 75.0;
// maximal board temperature - error state if above
pub const MAX_BOARD_TEMP: f32 = 100.0;

// minimal bus voltage - error state if below
pub const MIN_BUS_VOLTAGE: f32 = 10.0;
```
