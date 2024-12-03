# State machine module


This module contains the state machine implementation. The state machine implemented is based on the EtherCAT CiA 402 standard. 

## Modules  
- `cia402_state_machine.rs`: Contains the state machine implementation based on the EtherCAT CiA 402 standard
- `cia402_registries.rs`: Contains some registers of related to the CiA 402 standard
- `poulpe_state.rs`: Contains the wrapper of the CiA402 state machine for the Poulpe board 

## CiA 402 state machine
<img src="../../docs/state_machine.png" width="600">

The state machine is implemented as a finite state machine with the following states:
- `NotReadyToSwitchOn` - The initial state of the state machine. The motor is not ready to switch on.
- `SwitchOnDisabled` - The initialisation is done successfully. But the ready command has not been received yet
- `ReadyToSwitchOn` - The ready command has been received. The motor is ready to switch on.
- `SwitchedOn` - The motor is switched on but the voltage is not applied to the motor
- `OperationEnabled` - The motor is switched on and the voltage is applied to the motor
- `QuickStopActive` - The quick stop state has been activated (ex. emergency stop), and the stopping procedure is in progress
- `FaultReactionActive` - The fault state has been triggered and the fault reaction is in progress
- `Fault` - The fault reaction is done and the actual is in the fault state (not recoverable)

There is an additional warning flag that can be active if the board or motor temperatures are high, but still under the maximum allowed value. The warning flag is active in the `SwitchedOn` and `OperationEnabled` states.

## Error flags 

The firmware allows setting multiple error flags, which can be triggered by different events and multiple error flags can be active at the same time. The error flags are divided into two categories: motor and actuator errors.

There is a set of error flags that can be triggered for any motor (`MotorErrorFlag`):
- `ConfigFail` - The configuration of the motor has failed
- `MotorAlignFail` - The motor alignment has failed
- `HighTemperatureWarning` - The motor/board temperature is high but still under the maximum allowed value
- `OverTemperatureMotor` - The motor temperature is over the maximum allowed value
- `OverTemperatureBoard` - The board temperature is over the maximum allowed value
- `OverCurrent`  - The motor current is over the maximum allowed value (not implemented yet)
- `LowBusVoltage` - The bus voltage is lower than the minimum allowed value
- `DriverFault` - The PWM driver has a fault
- `TemperatureSensorMalfunctionWarning` - The temperature sensor has a malfunction, not reading well (not implemented yet)


Then there are more generic errors that are triggered for the actuator (`HomingErrorFlag`):
- `AxisSensorReadFail` - The absolute axis sensor reading has failed
- `MotorMovementCheckFail` - The motor movement check has failed
- `AxisSensorAlignFail` - The absolute axis sensor alignment has failed
- `ZeroingFail` - The zeroing of the motor has failed
- `IndexSearchFail` - The index search has failed (orbita3d)
- `LowLevelCommunicaiton` - The low-level communication has failed (either the pwm drivers or position sensors)


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