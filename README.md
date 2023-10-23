# Poulpe board firmware using Embassy-rs

## Installation

- `rustup default nightly`
- `rustup update`
- `rustup target add thumbv7em-none-eabihf`
- `cargo install probe-rs --features cli`
<<<<<<< HEAD
- `cargo install probe-run`


||||||| f4e04bb

=======
- https://probe.rs/docs/getting-started/probe-setup/
>>>>>>> test_dynamixel

## Build

- `cargo build`


## Architecture

### Tasks (TODO)

- ComDynamixel: uart dynamixel compatible communication
- TMC4671: spi communication with TMC4671
- RLS: spi communication with RLS
- AS5048A: spi communication with AS5048A
- Control: control loop
- Ethercat: ethercat communication

### Shared data

- DynamixelRegisters: Dynamixel registers
- TMC4671Registers: TMC4671 registers
- RLSRegisters: RLS registers
- AS5048ARegisters: AS5048A registers
- ControlRegisters: Control registers
- EthercatRegisters: Ethercat communication registers
