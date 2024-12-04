# Poulpe board firmware using Embassy-rs

<a href="https://github.com/pollen-robotics/elec_Poulpe"><img src="./docs/carte_Poulpe_2d.png" width="120px"></a><img src="./docs/carte_Poulpe_3d.png" width="120px"></a><img src="./docs/Poulpe_3d.png" width="120px"></a>


A complete firmware stack for the **Poulpe** boards in combination with **Venouse** boards, using the Rust programming language and the [Embassy-rs](https://github.com/embassy-rs/embassy) framework. The firmware is designed to work with the Orbita2d and Orbita3d actuator setups. 

- [Poulpe](https://github.com/pollen-robotics/elec_Poulpe) + [Sponge](https://github.com/pollen-robotics/elec_Sponge) + [TMC4671+TMC6100 BOB](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/tmc4671-tmc6100-bob.html)
- [Poulpe 2d](https://github.com/pollen-robotics/elec_Poulpe_2d)  + [Ventouse 2d](https://github.com/pollen-robotics/elec_Ventouse_2d)
- [Poulpe 3d ](https://github.com/pollen-robotics/elec_Poulpe_3d) + [Ventouse 3d](https://github.com/pollen-robotics/elec_Ventouse_3d)


## Table of contents

- [Installation](#installation)
- [Build](#build)
- [Run/Flush](#runflush)
- [Firmware architecture](#firmware-architecture)
    - [Firmware real-time tasks](#firmware-real-time-tasks)
    - [Orbita2d architecture](#orbita2d-architecture)
    - [Orbita3d architecture](#orbita3d-architecture)
- [Firmware configuration](#firmware-configuration)
- [Safety features](#safety-features)
- [Firmware state machine](#firmware-state-machine)





## Installation

- `rustup default nightly`
- `rustup update`
- `rustup target add thumbv7em-none-eabihf`
- `cargo install probe-rs --features cli`
- Setup the st-link v2 device permisions: [more info in probe docs](https://probe.rs/docs/getting-started/probe-setup/)

### Build

```sh
cargo build --release --features # hardware version ex. orbita3d_beta
```

Version | orbita2d | orbita3d | Communication | Hardware
----| ----| ----| ---- | ----
BETA | `orbita2d_beta` | `orbita3d_beta` |  dynamixel | [Poulpe](https://github.com/pollen-robotics/elec_Poulpe) + [Sponge](https://github.com/pollen-robotics/elec_Sponge) + [TMC4671+TMC6100 BOB](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/tmc4671-tmc6100-bob.html)
DVT | `orbita2d_gamma` | `orbita3d_gamma` | EtherCAT | [Poulpe 2d](https://github.com/pollen-robotics/elec_Poulpe_2d)  + [Ventouse 2d](https://github.com/pollen-robotics/elec_Ventouse_2d) or [Poulpe 3d ](https://github.com/pollen-robotics/elec_Poulpe_3d) + [Ventouse 3d](https://github.com/pollen-robotics/elec_Ventouse_3d)
PVT | `orbita2d_pvt` | `orbita3d_pvt` | EtherCAT | [Poulpe 2d](https://github.com/pollen-robotics/elec_Poulpe_2d)  + [Ventouse 2d](https://github.com/pollen-robotics/elec_Ventouse_2d) or [Poulpe 3d ](https://github.com/pollen-robotics/elec_Poulpe_3d) + [Ventouse 3d](https://github.com/pollen-robotics/elec_Ventouse_3d)

<b>Note</b>: The first build will take a long time because it will download the dependencies and compile them.

### Run/Flush
1) Make sure that the stlink is connected to the board and to the computer
2) Make sure that you selected the proper version of your hardware as indicated in the table above
3) Run the command to flush the board:
ex. `cargo run --release --features orbita2d_beta`
This command will build the firmware and flash it to the board, and then it will start the firmware.

<details>
<summary><b>Debugging output</b></summary>
Optionally you can add the <code>DEFMT_LOG</code> environment variable to see the logs<br>
<pre>
<code>DEFMT_LOG=debug cargo run --release --features orbita2d_pvt</code>
</pre>
It can also be set to <code>trace</code> or <code>info</code>. For the release version, the logs should be disbled, set the <code>DEFMT_LOG</code> to <code>off</code><br>
<pre>
<code>DEFMT_LOG=off cargo run --release --features orbita2d_pvt</code>
</pre>
</details>


## Firmware architecture

The software is divided into a few main rust modules:
- `sensors` - module implementing the communication with the sensors - [read more](src/sensors/README.md)
- `motor_control` - module implementing the motor control - [read more](src/motor_control/README.md)
- `config` - module unifying the configuration of the firmware - [read more](src/config/README.md)
- `ethercat` - module implementing the EtherCAT communication - [read more](src/ethercat/README.md)
- `dynamixel` - module implementing the Dynamixel communication - [read more](src/dynamixel/README.md)
- `state_machine` - module implementing the state machine of the board and the safety features - [read more](src/state_machine/README.md)
- `utils` - module implementing the utility functions - [read more](src/utils/README.md)
- `bin` - module implementing a set of test and benchmark programs that can be run on the board - [read more](src/bin/README.md)

The main firmware is implemented in the `main.rs` file. 

### Firmware real-time tasks

<img src="docs/firmware_tasks.png" alt="Firmware architecture" width="400" />

The firmware is composed into two real-time tasks that communicate through the `shared_memory` module. The two tasks are:

1) `control_loop` - responsible for the motor control and sensor reading  
    - inialization of the motor control and sensor reading
    - communication with the low-level TMC4671 actuators using SPI
    - reading the motor position sensors
    - reading the motor and board temperatures 
    - ensuring the safety of the motor by monitoring the temperatures, voltages low-level errors etc.

2) `message_handler` - responsible for the communication with the host computer either
    - using the serial communication and Dynamixel protocol - `dynamixel`
    - using EtherCAT protocol - `ethercat`
    - choosing the communication protocol is done using the `features` (either `ethercat` or `dynamixel`)


### Orbita2d architecture

<details>
<summary><b>Orbita2d beta</b></summary>
<img src="docs/orbita2d.png" alt="Orbita 2D" />
</details>

<details>
<summary><b>Orbita2d DVT</b></summary>
<img src="docs/orbita2d_dvt.png" alt="Orbita 2D" />
</details>
<details open>
<summary><b>Orbita2d PVT</b></summary>
<img src="docs/orbita2d_pvt.png" alt="Orbita 2D" />
</details>

Orbita2d is a robotic actuator with two motors that use differential drive to run two axis. The motors used are maxon flat motors of Maxon `EC45` series. 
There are three differnet versions of the Orbita2d setup: beta, DVT and PVT. The main differences between the versions are the motor control board and the communication protocol used.

version | control board | driver control board | motor | communication | temperature sensing | axis sensor communication
----| ----| ----| ---- | ---- | ---- | ----
beta | poulpe | TMC4671 + TMC6100 BOB | EC45 flat | dynamixel | motor B | SPI
DVT | poulpe2d | ventouse2d | EC45 flat | EtherCAT | motor B | SPI
PVT | poulpe2d | ventouse2d | EC45 flat | EtherCAT | both motors | Differential I2C <br> LTC4332




### Orbita3d architecture

<details>
<summary><b>Orbita2d beta</b></summary>
<img src="docs/orbita3d.png" alt="Orbita 2D" />
</details>

<details>
<summary><b>Orbita2d DVT</b></summary>
<img src="docs/orbita3d_dvt.png" alt="Orbita 2D" />
</details>
<details open>
<summary><b>Orbita2d PVT</b></summary>
<img src="docs/orbita3d_pvt.png" alt="Orbita 2D" />
</details>

Orbita3d is a robotic actuator with three motors that use a parallel mechanical structure drive to run three axis. The motors used are maxon motors of the Maxon `ECX22` series. 
There are three differnet versions of the Orbita3d setup: beta, DVT and PVT.

version | control board | driver control board | motor | communication | temperature sensing | axis sensor communication 
----| ----| ----| ---- | ---- | ---- | ----
beta | poulpe | TMC4671 + TMC6100 BOB | ECX22 M | dynamixel | motor TOP | SPI, I2C
DVT | poulpe3d | ventouse3d | ECX22 M | EtherCAT | motor TOP | SPI, I2C
PVT | poulpe3d | ventouse3d | ECX22 L | EtherCAT | all three motors | Differential link<br> LTC4332(SPI)<br>LTC4331(I2C)


## Firmware configuration
The same firmware can be configured to work with many different  Orbita2d and Orbita3d hardware setups from which the most important are the beta, DVT and PVT versions. The configutation can be done using the `Cargo.toml` file and the command line arguments `--featatures`.


### Main features

Version | orbita2d | orbita3d | Communication | Hardware
----| ----| ----| ---- | ----
BETA | `orbita2d_beta` | `orbita3d_beta` |  dynamixel | [Poulpe](https://github.com/pollen-robotics/elec_Poulpe) + [Sponge](https://github.com/pollen-robotics/elec_Sponge) + [TMC4671+TMC6100 BOB](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/tmc4671-tmc6100-bob.html)
DVT | `orbita2d_gamma` | `orbita3d_gamma` | EtherCAT | [Poulpe 2d](https://github.com/pollen-robotics/elec_Poulpe_2d)  + [Ventouse 2d](https://github.com/pollen-robotics/elec_Ventouse_2d) or [Poulpe 3d ](https://github.com/pollen-robotics/elec_Poulpe_3d) + [Ventouse 3d](https://github.com/pollen-robotics/elec_Ventouse_3d)
PVT | `orbita2d_pvt` | `orbita3d_pvt` | EtherCAT | [Poulpe 2d](https://github.com/pollen-robotics/elec_Poulpe_2d)  + [Ventouse 2d](https://github.com/pollen-robotics/elec_Ventouse_2d) or [Poulpe 3d ](https://github.com/pollen-robotics/elec_Poulpe_3d) + [Ventouse 3d](https://github.com/pollen-robotics/elec_Ventouse_3d)

### All features

Orbita2d and Orbita3d setups can be configured using the following features (one of them has to be set):
- `orbita2d` - Orbita2d actuator setup  - [github page](https://github.com/pollen-robotics/orbita2d_control)
- `orbita3d` - Orbita3d actuator setup - [github page](https://github.com/pollen-robotics/orbita3d_control)

<b> Electronics version </b>
The electronics version determins the motor control board used and the communication protocols to the sensors
- `beta` - beta electronics version
- `gamma` - DVT electronics version
- `pvt` - PVT electronics version
> One them has to be set

<b> Motor version </b>
The motor version determins the gearing ratio and motor parameters used in the fimrware for the motor control
- `ec60` - EC60 flat maxon motor used - [datasheet](https://www.maxongroup.net.au/maxon/view/product/motor/ecmotor/ecflat/ecflat60/645604)
- `ec45` - EC45 flat maxon motor used - [datasheet](https://www.maxongroup.fr/medias/sys_master/root/8882563907614/EN-21-300.pdf)
- `ecx22` - ECX22 maxon motor used - [datasheet](https://www.maxongroup.com/maxon/view/product/motor/ecmotor/ECX/ECX22/ECXI22M4ZF46C4IL1Y501A)
> One them has to be set


<b> Communication configuration features </b>

There are two supported communication protocoles and each can be enabled using its dedicated feature
- `ethercat` - EtherCAT communication
- `dynamixel` - dynamixel communication 
> One them has to be set

They cannot be used both at the same time. At least one of them has to be used in order to be able to talk to the poulpe board.

<b>Advanced control features</b>
These features are used to configure the advanced control features in order to improve the motor control performance
- `cmd_filter` - Filter received position commands to reduce the jerk of the motor
- `velocity_feedforward` - Enable the use of the velocity feedforward to improve the velocity tracking performance 
    - has to be used in conjunction with the appropriate dynamixel message
    - using this feature will not change the default behavior of the firmware
- `allow_mode_change` - Allow the mode change of the motor control. Default mode is the position mode, and if this feature is enabled the motor can be switched to the velocity mode or to torque mode


<b>Actuator output features</b>
These features are used to decide which output angle of the motor to control
- `gearbox_output` - Control the motor angle after the gearbox
- `axis_output` - Control the motor angle after the gearbox and axis reduction

<b>Safety features</b>
Used to configure the safety features of the board
- `no_temperature_sensor` - The board does not have a temperature sensor, avoid reading it and using it for safety
- `ignore_errors` - Ignore the safety errors and continue the operation
- `allow_quickstop` - Allow the quickstop state of the actuator, a software emergency stop

<b>Flash memory features</b>
Used to enable/disable the usage of the flash memory
- `use_flash` - Enable the usage of the flash memory
- `write_flash` - Write the configuration to the flash memory (if not used the configuration will be read from flash - if available)

<b>Debugging features</b>
Used to enable the debugging features
- `debug_execution_time` - Measure and display the execution time of the real-time tasks

### Axis absolute zeros configuration

In order to use the absolute zero position of the actuators the absolute zero values need to be writen to the flash memory of the poulpe boards. These values are written to the memory once and are used on the boot of the board. The absolute zeros can be set using the `ZEROS` command line argument. The values are written to the flash memory using the `write_flash` feature.


- If the `use_flash` feature is enabled the configuration will be read from the flash memory on the boot. If the configuration is not found in the flash memory the default configuration  will be used. (`HARDWARE_ZEROS=[0, 0, 0]` for orbita3d and `HARDWARE_ZEROS=[0, 0]` for orbita2d)

- To write the configuration to the flash memory use the `write_flash` feature and set the command line arguments `ZEROS` to the desired values. The configuration will be written to the flash memory and will be read from it on the next boot. 

```bash
ZEROS=0.12,0.34,0.56 cargo run --release --features "orbita3d_xvz,write_flash"
```

- Once the configuration is written to the flash memory the `write_flash` feature can be removed as well as the command line arguments. The firmaware will automatically read the configuration from the flash memory on the next boot. 

```bash
cargo run --release --features orbita3d_xvz # the configuration will be read from the flash memory
```

- To reset the configuration in the flash memory use the `write_flash` feature and dont set any command line arguemnts. The configuration will be reset to the default values.

```bash
cargo run --release --features "orbitaNd_xvz,write_flash"
```

So here is an example suggested workflow:
1) Set the desired configuration using the command line arguments and the `write_flash` feature
```bash
ZEROS=0.12,0.34,0.56 cargo run --release --features "orbita3d_beta,write_flash"
```
2) Remove the `write_flash` feature and the command line arguments for any other upload of the firmware in the future
```bash
cargo run --release --features orbita3d_beta
```


## Safety features

The firmware has implemented several safety features to ensure the safety of the motor and the user. The safety features are implemented in the `motor_control` module and are executed in the `control_loop` real-time task. The safety features are:
<b>Safe startup and checks</b>
- Check that the low-level drivers are working properly
- Check that the absolute sensors are working properly
- Check that the motor moves freely and is not blocked

Only if all the checks are passed the board will pass the initialization and will be ready to be switched on.

<b>Real-time safety monitoring</b>
- Motor temperature monitoring (high temperature warning at 65°C, high temperature error at 75°C)
- Board temperature monitoring (high temperature warning at 65°C, high temperature error at 75°C)
- Low-level driver failure monitoring
- Absolute sensor error monitoring
- Bus voltage monitoring (error under 10V)
- Low-level driver and sensor communication monitoring (error if the communication is lost for more thant 1s) 
- Over-temperature protection (not implemented yet)

The poulpe will stop disable the motors if any of the safety checks failed are triggered. 


## Frimware state machine

The state machine of the poulpe board is implemented in the `state_machine` module. The state machine is responsible for the initialization of the board and the safety features. The state machine follows the CiA 402 standard for the motor control. 

<img src="docs/state_machine.png" alt="State machine" width="500" />

The state machine has the following states:
- `NotReadyToSwitchOn` - The board is performing the initialization
- `SwitchOnDisabled` - The init is done successfully and the board is disabled
- `ReadyToSwitchOn` - The board is ready to be switched on 
- `SwitchedOn` - The board is switched on
- `OperationEnabled` - The board is switched on and the actuators are enabled
- `QuickStopActive` - The board is responding to the quick stop command (emergency stop)
    - once the response is done the firmware goes to the `SwitchOnDisabled` state
- `FaultReactionActive` - The board is in the fault reaction state (one of the safety checks failed)
    - once the response is done the firmware goes to the `Fault` state
- `Fault` - The board is in the fault state (not recoverable error)

There is an additional warning flag that can be active if the board or motor temperatures are high, but still under the maximum allowed value. The warning flag is active in the `SwitchedOn` and `OperationEnabled` states.

Read more in the [state machine module](src/state_machine/README.md)


## LED blinking patterns

The blinking of the LED on the board is used to indicate the state of the board. There are two colors of the LED - green and red. The LED can be solid or blinking. The LED is blinking with a period of 500ms. The pattern of blinking is as follows:


 state            | green         | red
 -----------------|---------------|------
 init             | blinks        | blinks
 preop            | solid         | off
 preop  + warning | solid         | blinks
 op               | solid         | off
 op  + warning    | solid         | blinks
 fault            | off           | solid
 fault_reaction   | off           | blinks
 quick_stop_reaction   | solid           | solid
## LED blinking patterns

The blinking of the LED on the board is used to indicate the state of the board. There are two colors of the LED - green and red. The LED can be solid or blinking. The LED is blinking with a period of 500ms. The pattern of blinking is as follows:


 state           | CiA402 state | green         | red
 ----------------|--------------|---------------|---------
 init            | `NotReadyToSwitchOn`  | blinks        | blinks
 preop           |`SwitchOnDisabled`,`ReadyToSwitchOn`,`SwitchedOn` | solid         | off
 preop  + warning |`SwitchOnDisabled`,`ReadyToSwitchOn`,`SwitchedOn`| solid         | blinks
 op               |`OperationEnabled`| solid         | off
 op  + warning    |`OperationEnabled`| solid         | blinks
 fault            |`Fault`| off           | solid
 fault_reaction   |`FaultReactionActive`| off           | blinks
 quick_stop_reaction   |`QuickStopActive`| solid           | solid

## Future work and improvements

- Safety 
    - Make more accurate motor temperature reading
    - Add over-current protection
- Testing  - [initial developement](https://github.com/pollen-robotics/firmware_Poulpe/tree/feat_embedded_tests)
    - Add unit tests
    - Add integration tests
    - Add hardware tests

