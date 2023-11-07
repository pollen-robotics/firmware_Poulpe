use defmt::{debug, error};
use embassy_stm32::gpio::AnyPin;

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
                        debug!("Sending status packet: {:?}", sp);
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
                                debug!("Sending status packet: {:?}", sp);
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }
                            DynamixelRegister::CurrentPosition => {
                                let value = { SHARED_MEMORY.lock().await.get_current_position() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                debug!("Sending status packet: {:?}", sp);
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }
                            DynamixelRegister::TargetPosition => {
                                let value = { SHARED_MEMORY.lock().await.get_target_position() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                debug!("Sending status packet: {:?}", sp);
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }
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
                            }
                            DynamixelRegister::TargetPosition => {
                                let target: [f32; config::N_AXIS] =
                                    conversion::bytes_to_float(write_data_packet.data);
                                {
                                    SHARED_MEMORY.lock().await.set_target_position(target);
                                }
                            }
                            _ => {}
                        }

                        let sp = StatusPacket::ack(id, dxl_error);
                        debug!("Sending status packet: {:?}", sp);
                        if let Some(e) = dxl.write(&sp).await.err() {
                            error!("Error: {:?}", e);
                        }
                    }
                }
            }
            Err(e) => {
                error!("Error: {:?}", e);
            }
        }
    }
}
