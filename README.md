# Poulpe board firmware using Embassy-rs

<a href="https://github.com/pollen-robotics/elec_Poulpe"><img align="right" src="./docs/Poulpe_3d.png" width="120px"></a>
A complete firmware stack for the [Poulpe](https://github.com/pollen-robotics/elec_Poulpe) board using the Rust programming language and the [Embassy-rs](https://github.com/embassy-rs/embassy) framework. The firmware is designed to work with the Orbita2d and Orbita3d actuator setups. 

## Table of contents

- [Installation](#installation)
    - [Build](#build)
    - [Run/Flush](#runflush)
- [Software architecture](#software-architecture)
    - [Firmware configuration](#firmware-configuration)
    - [Dynamixel ID and sensor zeros configuration](#dynamixel-id-and-sensor-zeros-configuration)
- [Orbita2d architecture](#orbita2d-architecture)
- [Orbita3d architecture](#orbita3d-architecture)
- [Safety features](#safety-features)
    - [Board state](#board-state)
- [Future work and improvements](#future-work-and-improvements)




## Installation

- `rustup default nightly`
- `rustup update`
- `rustup target add thumbv7em-none-eabihf`
- `cargo install probe-rs --features cli`
- Setup the st-link v2 device permisions: [more info in probe docs](https://probe.rs/docs/getting-started/probe-setup/)

### Build

- `cargo build --release`

<b>Note</b>: The first build will take a long time because it will download the dependencies and compile them.

### Run/Flush
1) Make sure that the stlink is connected to the board and to the computer
2) Make sure that you have properly configured the features for the board in the `Cargo.toml` file
3) Run the command to flush the board:
`cargo run --release`
This command will build the firmware and flash it to the board, and then it will start the firmware. The firmware will start the real-time tasks and will be ready to receive the dynamixel commands at the default dynamixel ID `42`.

<details>
<summary><b>Debugging output</b></summary>
Optionally you can add the <code>DEFMT_LOG</code> environment variable to see the logs<br>
<pre>
<code>DEFMT_LOG=debug cargo run --release</code>
</pre>
It can also be set to <code>trace</code> or <code>info</code>. For the release version, the logs should be disbled, set the <code>DEFMT_LOG</code> to <code>off</code><br>
<pre>
<code>DEFMT_LOG=off cargo run --release</code>
</pre>
</details>

<details>
<summary><b>Changing the dynamixel ID</b></summary>
The dynamixel ID can be changed by setting the <code>DXL_ID</code> environment variable<br>
<pre>
<code>DXL_ID=50 cargo run --release</code>
</pre>
</details>


## Software architecture

The software is divided into four main rust modules:
- `dynamixel` - Implementation of the dynamixel protocol - [Read more](src/dynamixel/README.md)
    - `registers` - Dynamixel registers used to communicate with the host computer
    - `task` - Real-time task executing the dynamixel communication
- `motor_control` - Implementation of the low-level motor control - [Read more](src/motor_control/README.md)
   - `foc` - Communication/configuration of the TMC4671 controller
   - `sensors` - Communication with the RLS and AS5047 sensors
   - `analog` - ADC reading of the motor temperature
   - `task` - Real-time task executing the motor control, sensor reading and communication
- `config` - Configuration mofule for the motor control task - [Read more](src/config/README.md)
    - `motor` - Motor configuration
    - `current_sense` - Current sense configuration
    - `flash` - Flash memory management and configuration
- `shared_memory` - Shared memory between the motor controla and dynamixel communication tasks

#### Real-time tasks
The firmware runs two real-time tasks:
1) `message_handler` - responsible for the communication with the host computer using the serial communication and Dynamixel protocol
2) `control_loop` - responsible for the motor control and sensor reading  
    - inialization of the motor control and sensor reading
    - communication with the low-level TMC4671 actuators using SPI
    - reading the motor position sensors RLS and AS5047 using SPI
    - reading the motor temperature using ADC
    - ensuring the safety of the motor by monitoring the motor temperature, current and voltage

The tasks share data through the `SHARED_MEMORY` module.


### Firmware configuration
The same firmware can be configured to work with two different actuator setups: orbita2d and orbita3d. The orbita2d setup is a 2dof actuator with two motors and the orbita3d setup is a 3dof actuator with three motors. 

The firmware is configured using the `Cargo.toml` file. The configuration is done using the `features` field. In order for the feature to be used in the firmware its name has to be added to the `defualt` array. The following features are available:

<b>Actuator configuration features</b>
These features are used to configure the actuator setup 
- `orbita2d` - Orbita2d actuator setup - [github page](https://github.com/pollen-robotics/orbita2d_control)
- `orbita3d` - Orbita3d actuator setup - [github page](https://github.com/pollen-robotics/orbita3d_control)
- `ec60` - EC60 flat maxon motor used - [datasheet](https://www.maxongroup.net.au/maxon/view/product/motor/ecmotor/ecflat/ecflat60/645604)
- `ec45` - EC45 flat maxon motor used - [datasheet](https://www.maxongroup.fr/medias/sys_master/root/8882563907614/EN-21-300.pdf)
- `ecx22` - ECX22 maxon motor used - [datasheet](https://www.maxongroup.com/maxon/view/product/motor/ecmotor/ECX/ECX22/ECXI22M4ZF46C4IL1Y501A)

<b>Advanced control features</b>
These features are used to configure the advanced control features in order to improve the motor control performance
- `cmd_filter` - Filter received position commands to reduce the jerk of the motor
- `velocity_feedforward` - Enable the use of the velocity feedforward to improve the velocity tracking performance 
    - has to be used in conjunction with the appropriate dynamixel message
    - using this feature will not change the default behavior of the firmware

<b>Actuator output features</b>
These features are used to decide which output angle of the motor to control
- `gearbox_output` - Control the motor angle after the gearbox
- `axis_output` - Control the motor angle after the gearbox and axis reduction

<b>Safety features</b>
Used to configure the safety features of the board
- `no_temperature_sensor` - The board does not have a temperature sensor, avoid reading it and using it for safety
- `ignore_errors` - Ignore the safety errors and continue the operation

<b>Flash memory features</b>
Used to enable/disable the usage of the flash memory
- `use_flash` - Enable the usage of the flash memory
- `write_flash` - Write the configuration to the flash memory (if not used the configuration will be read from flash - if available)

### Dynamixel ID and sensor zeros configuration

There are two ways to configure the firmware:
- By writing and reading the configuration to/from the flash memory **(default)**
- By using the command line arguments

#### Flash memory configuration (default)
To enable using flash memory make sure to include the `use_flash` feature in the `Cargo.toml` (it is included by default).

- If the `use_flash` feature is enabled the configuration will be read from the flash memory on the boot. If the configuration is not found in the flash memory the default configuration  will be used. (`DXL_ID=42` and `HARDWARE_ZEROS=[0, 0, 0]` for orbita3d and `HARDWARE_ZEROS=[0, 0]` for orbita2d)

- To write the configuration to the flash memory use the `write_flash` feature and set the command line arguments `DXL_ID` and `ZEROS` to the desired values. The configuration will be written to the flash memory and will be read from it on the next boot. 

```bash
DXL_ID=50 ZEROS=0.12,0.34,0.56 cargo run --release --features write_flash
```

- Once the configuration is written to the flash memory the `write_flash` feature can be removed as well as the command line arguments. The firmaware will automatically read the configuration from the flash memory on the next boot. 

```bash
cargo run --release # the configuration will be read from the flash memory
```

- To reset the configuration in the flash memory use the `write_flash` feature and dont set any command line arguemnts. The configuration will be reset to the default values.

```bash
cargo run --release --features write_flash
```

So here is an example suggested workflow:
1) Set the desired configuration using the command line arguments and the `write_flash` feature
```bash
DXL_ID=50 ZEROS=0.12,0.34,0.56 cargo run --release --features write_flash
```
2) Remove the `write_flash` feature and the command line arguments for any other upload of the firmware in the future
```bash
cargo run --release
```



#### Configuration using command line arguments

The same two command line arguments `ZEROS` and `DXL_ID` can be used to configure the firmware without using the flash memory (no `use_flash` speciifed). 
- `DXL_ID` - Dynamixel ID used by the firmware, default is `42`
- `ZEROS` - default is `[0, 0, 0]` (orbita3d) and `[0, 0]` (orbita2d)
    - The motor positions associated with the actuator's zero absolute position

This way the configuration can be set without using the flash memory, however it will only be valid for this upload of the firmware. If you want to keep the configuration for the next version of the firmware you need to provide the same command line arguments.

```bash
DXL_ID=50 ZEROS=0.12,0.34,0.56 cargo run --release
```


### Orbita2d architecture
<img src="docs/orbita2d.png" alt="Orbita 2D" />

Orbita2d is a robotic actuator with two motors that use differential drive to run two axis. The motors used are maxon flat motors of either `EC60` or `EC45` series. The motor control board used in the setup is the [TMC4671 + TMC6100 BOB](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/tmc4671-tmc6100-bob.html) board. These boards implment the FOC control of the motor and the communication with the motor sensors (incremental encoders). The motor control board is connected to the Poulpe board using SPI. The motor control board is also connected to the motor temperature sensor and read's it using the internal ADC. The two output axis of the orbita2d are equiped with absolute encoders that are read using SPI. 



### Orbita3d architecture
<img src="docs/orbita3d.png" alt="Orbita 3D" />

Orbita3d is a robotic actuator with three motors that use a parallel mechanical structure drive to run three axis. The motors used are maxon motors of the `ECX22` series. As well as for orbita2s, the motor control board used in the setup is the [TMC4671 + TMC6100 BOB](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/tmc4671-tmc6100-bob.html) board which are connected to the Poulpe board using SPI. The motor control board is also connected to the motor temperature sensor and read's it using the internal ADC. The three output axis of the motors (after the gearbox) are equiped with absolute encoders that are read using SPI. Additionally, the orbita3d setup has an absolute position sensor based on an array of hall sensors, that are read using the I2C communication protocol. Given the position sensors after the gearbox and the hall sensor array, the orbita3d is capable of detecting the absolute position of the end effector, if one is provided using the `ZEROS` parameter.


## Safety features

The firmware has implemented several safety features to ensure the safety of the motor and the user. The safety features are implemented in the `motor_control` module and are executed in the `control_loop` real-time task. The safety features are:
<b>Safe startup and checks</b>
- Motor enabled only if initialized properly
- Motor enabled only if the BOB is configured properly

<b>Real-time safety monitoring</b>
- Motor temperature monitoring
- BOB temperature monitoring
- Over-current protection (not implemented yet)
- Undervoltage protection 

The poulpe will stop disable the motor if any of the safety features are triggered. 

> Currently the motor can not be enabled again after the safety feature is triggered. The user has to reset the board in order to enable the motor again.

### Board state
The state of the poulpe board containg its safety features can be read using the dynamixel protocol, and is managed in firmware with the `BoardState` structure. Possible states are:

<b>Normal operation states</b>
- `Ok = 0` - Board is working properly
- `HighTemperatureState = 100` - The motor/board temperature is high 
    - but not too high - warning state
    - it returns to `Ok` if the temperature is back to normal
    - temperature threshold is set in the `config` module (`config::HIGH_TEMP` - default `65°C`)

<b>Initialisation error states</b>
The actuators can not be enabled if the board is in one of these states
- `InitError = 1` - Board failed to initialise properly
- `SensorError = 2` - The absolute position sensor failed to initialise properly
- `IndexError = 3` - The absolute position sensor failed to find the index 
    - orbita3d only
- `ZeroingError = 4` - The zeroing of the absolute position sensor failed 
    - orbita3d only
> these errors are not recoverable

<b>Real-time safety violation states</b>
The actuators will stop their operation gracefully if the board is in one of these states
- `OverTemperatureError = 5` - The motor/board temperature is too high
    - temperature threshold is set in the `config` module 
    - boards: `config::MAX_BOARD_TEMP` - default `100°C`
    - motors: `config::MAX_MOTOR_TEMP` - default `75°C`
- `OverCurrentError = 6` - The motor current is too high (**not implemented yet**)
- `BusVoltageError = 7` - The bus voltage is too low
    - voltage threshold is set to `10V` 
- `Unknown = 255` - Unknown error
> these errors are not recoverable

If any of thre real-time safety violation states are triggered the actuators will stop their operation gracefully. They will try reduce the velocity limit to the 10% of the max value and reduce the torque to 0 gradually over 5 seconds. 
Once the zero torque is reached the actuators will be disabled. The user has to reset the board in order to enable the actuators again.

## Future work and improvements

- Safety 
    - Make more accurate motor temperature reading
    - Add over-current protection
- Testing  - [initial developement](https://github.com/pollen-robotics/firmware_Poulpe/tree/feat_embedded_tests)
    - Add unit tests
    - Add integration tests
    - Add hardware tests

