use defmt::{debug, error, trace};
use embassy_stm32::gpio::AnyPin;
use embassy_time::{Timer, Duration};

use crate::{
    config,
    dynamixel::{conversion, DynamixelRegister, InstructionPacketKind, StatusPacket},
    SHARED_MEMORY,
};

#[embassy_executor::task]
pub async fn messsage_handler(usart: config::DynamixelUart, dir_pin: AnyPin) {
    let id = config::DXL_ID;
    let mut dxl = super::DynamixelUsartIO::new(usart, dir_pin, id);

    let dxl_error = 0;

    loop {
        debug!("Waiting for packet...");
        match dxl.read().await {
            Ok(packet) => {
                debug!("Got packet: {:?}", packet);

                match packet {
                    InstructionPacketKind::Ping(_) => {
                        let sp = StatusPacket::ack(id, dxl_error);
                        trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                        if let Some(e) = dxl.write(&sp).await.err() {
                            error!("Error: {:?}", e);
                        }
                    }
                    InstructionPacketKind::ReadData(read_data_packet) => {
                        let reg = DynamixelRegister::with_address(read_data_packet.address);
                        if reg.is_none() {
                            error!("Invalid register address: {}", read_data_packet.address);
                            continue;
                        }
                        let reg = reg.unwrap();

                        match reg {
                            DynamixelRegister::TorqueEnable => {
                                // TODO: abstract those in any way possible
                                let value = { SHARED_MEMORY.lock().await.get_torque_on() };
                                let value = conversion::bool_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }
                            DynamixelRegister::CurrentPosition => {
                                let value = { SHARED_MEMORY.lock().await.get_current_position() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::CurrentVelocity => {
                                let value = { SHARED_MEMORY.lock().await.get_current_velocity() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::CurrentTorque => {
                                let value = { SHARED_MEMORY.lock().await.get_current_torque() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::TargetPosition => {
                                let value = { SHARED_MEMORY.lock().await.get_target_position() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }


                            DynamixelRegister::AxisSensor => {
                                let value = { SHARED_MEMORY.lock().await.get_axis_sensor() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }


                            DynamixelRegister::FullState => {
				// let target = { SHARED_MEMORY.lock().await.get_target_position() };
                                // let pos = { SHARED_MEMORY.lock().await.get_current_position() };
				// let vel = { SHARED_MEMORY.lock().await.get_current_velocity() };
				// let torque = { SHARED_MEMORY.lock().await.get_current_torque() };
                                // // let sensor = { SHARED_MEMORY.lock().await.get_axis_sensor() };
				// let torque_on = { SHARED_MEMORY.lock().await.get_torque_on() };

				// //concatenate all the values
                                // let target  = conversion::float_to_bytes(target);
				// let pos = conversion::float_to_bytes(pos);
				// let vel = conversion::float_to_bytes(vel);
				// let torque = conversion::float_to_bytes(torque);
				// // let sensor = conversion::float_to_bytes(sensor);
				// let torque_on = conversion::bool_to_bytes(torque_on);

				// //I cannot find a better way to do this
				// let mut value = [0; (4*4+1)*config::N_AXIS];
				// value[0..4*config::N_AXIS].copy_from_slice(&target[0..4*config::N_AXIS]);
				// value[4*config::N_AXIS..2*4*config::N_AXIS].copy_from_slice(&pos[0..4*config::N_AXIS]);
				// value[2*4*config::N_AXIS..3*4*config::N_AXIS].copy_from_slice(&vel[0..4*config::N_AXIS]);
				// value[3*4*config::N_AXIS..4*4*config::N_AXIS].copy_from_slice(&torque[0..4*config::N_AXIS]);
				// value[4*4*config::N_AXIS..(4*4+1)*config::N_AXIS].copy_from_slice(&torque_on[0..config::N_AXIS]);


				let value={ SHARED_MEMORY.lock().await.get_full_state() };
                                let value  = conversion::float_to_bytes(value);


                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }


			    _ => {}


                        }
                    }
                    InstructionPacketKind::WriteData(write_data_packet) => {
                        let reg = DynamixelRegister::with_address(write_data_packet.address);
                        if reg.is_none() {
                            error!("Invalid register address: {}", write_data_packet.address);
                            continue;
                        }
                        let reg = reg.unwrap();

                        match reg {
                            // TODO: Can we match only on Write registers?
                            DynamixelRegister::TorqueEnable => {
                                let torque_on: [bool; config::N_AXIS] =
                                    conversion::bytes_to_bool(write_data_packet.data);
                                {
                                    SHARED_MEMORY.lock().await.set_torque_on(torque_on);
                                }
				let sp = StatusPacket::ack(id, dxl_error);
				debug!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
				if let Some(e) = dxl.write(&sp).await.err() {
				    error!("Error: {:?}", e);
				}
                            }
                            DynamixelRegister::TargetPosition => {

                                let target: [f32; config::N_AXIS] =
                                    conversion::bytes_to_float(write_data_packet.data);
                                {
                                    SHARED_MEMORY.lock().await.set_target_position(target);
                                }


				//return the full state
				let value={ SHARED_MEMORY.lock().await.get_full_state() };
                                let value  = conversion::float_to_bytes(value);


                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }



                            }
                            _ => {}
                        }

                        // let sp = StatusPacket::ack(id, dxl_error);
                        // debug!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                        // if let Some(e) = dxl.write(&sp).await.err() {
                        //     error!("Error: {:?}", e);
                        // }
                    }
                }
            }
            Err(e) => {
                error!("Error: {:?}", e);
            }
        }

        // Timer::after(Duration::from_micros(1)).await;

    }
}
