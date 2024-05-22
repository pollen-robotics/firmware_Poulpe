# Dynamixel communication crate

This crate provides the communication interface for the Dynamixel protocol. It is based on the `rustypot` library and its primary role is to provide the interface for the communication with the pouple boards. 

In this crate you can find the following modules:
- `packet`: Contains the packet definitions for the Dynamixel protocol
- `conversion`: Contains the utility conversion functions
- `registers`: Contains the list of registers used for the communication
- `task` : Contains the real-time task that is responsible for the communication 

## Implemented messages

Message ID | Message Name | Read/Write | Type | Description
--- | --- | --- | --- | -- 
`0`|`ModelNumber` | R | - |The model number of the device (not implemented)
`6`|`FirmwareVersion`| R | - |The firmware version of the device (not implemented) 
`7`|`Id`| R | u8 |The ID of the device (not implemented) 
`10`|`VelocityLimit`| R/W | `N_AXIS` x `f32`  |The velocity limit of the motors
`12`|`VelocityLimitMax`| R/W | `N_AXIS` x `f32` |The maximum velocity limit of the motors
`14`|`TorqueFluxLimit`| R/W |  `N_AXIS` x `f32` |The torque limit of the motors 0-1 range
`16`|`TorqueFluxLimitMax`| R/W | `N_AXIS` x `f32` |The maximum torque limit of the motors in milli amps
`18`|`UqUdLimit`| R/W | `N_AXIS` x `u32` |The voltage limit of the motors in 15 bit duty cycle
`20`|`FluxPID`| R/W | `N_AXIS` x [ 2 x `f32` ] |The PI controller parameters
`24`|`TorquePID`| R/W |  `N_AXIS` x [ 2 x `f32` ] |The PI controller parameters
`28`|`VelocityPID`| R/W |  `N_AXIS` x [ 2 x `f32` ] |The PI controller parameters
`32`|`PositionPID`|R/W |  `N_AXIS` x [ 2 x `f32` ] |The PI controller parameters
`40`|`TorqueEnable`|R/W |  `N_AXIS` x boolean |Enable/disable the torque
`51`|`CurrentVelocity`|R | `N_AXIS` x `f32` |Read the current velocity
`50`|`CurrentPosition`|R | `N_AXIS` x `f32` |Read the current position
`54`|`FeedforwardVelocity`|R/W | `N_AXIS` x `f32` |Read/write the feedforward velocity
`52`|`CurrentTorque`| R | `N_AXIS` x `f32` |Read the current torque
`60`|`TargetPosition`|R/W | `N_AXIS` x `f32` |ead/write the target position
`62`|`TargetPositionWithVelocityFF`| R/W | `N_AXIS` x [2 x `f32`] |Write the target position with velocity feedforward (returns the actual position)
`64`|`TargetPositionEstimateVelocityFF`|R/W | `N_AXIS` x `f32` |Write the target position, feedfoward is estimated in the firmware (returns the actual position)
`70`|`Temperature`| R | `N_AXIS` + 1 x `f32`  |Read the temperature of the motor and the boards
`72`|`BusVoltage`| R | `N_AXIS` x `f32` |Read the bus voltage
`80`|`BoardState`|R/W | `u8` |Read/write the board state
`90`|`AxisSensor`| R  | `N_AXIS` x `f32` | Read the actuator ssensor values
`99`|`IndexSensor`| R | `N_AXIS` x `u8` | Read the index hall sensors 
`100`|`FullState`| R | `N_AXIS` x [3 x `f32`] | Read the full state (actual position, velocity, torque) at once

Where `N_AXIS` is `2` for the orbita 2d and `3` for the orbita 3d.