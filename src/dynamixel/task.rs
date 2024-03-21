use defmt::{debug, error, trace};
use embassy_stm32::gpio::AnyPin;
use embassy_time::{Duration, Instant, Timer};

use crate::{
    config,
    dynamixel::{
        self, conversion, packet::ParsingError, DynamixelRegister, InstructionPacketKind,
        StatusPacket,
    },
    motor_control::{BoardStatus, Pid},
    SHARED_MEMORY,
};

#[embassy_executor::task]
pub async fn messsage_handler(usart: config::DynamixelUart, dir_pin: AnyPin) {
    let id = config::DXL_ID;
    let mut dxl = super::DynamixelUsartIO::new(usart, dir_pin, id);

    let mut dxl_error = 0;

    loop {
        debug!("Waiting for packet...");
        match dxl.read().await {
            Ok(packet) => {
                debug!("Got packet: {:?}", packet);
                dxl_error = { SHARED_MEMORY.lock().await.get_error_state() } as u8;
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
                            DynamixelRegister::BoardState => {
                                let value = { SHARED_MEMORY.lock().await.get_error_state() };
                                let sp = StatusPacket::with_value(id, dxl_error, [value as u8]);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

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

                            // reading velocity feedforward
                            DynamixelRegister::FeedforwardVelocity => {
                                let value =
                                    { SHARED_MEMORY.lock().await.get_velocity_feedforward() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                debug!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
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

                            DynamixelRegister::FluxPID => {
                                let value = { SHARED_MEMORY.lock().await.get_flux_pid_gains() };
                                let value = conversion::pid_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::TorquePID => {
                                let value = { SHARED_MEMORY.lock().await.get_torque_pid_gains() };
                                let value = conversion::pid_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::VelocityPID => {
                                let value = { SHARED_MEMORY.lock().await.get_velocity_pid_gains() };
                                let value = conversion::pid_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::PositionPID => {
                                let value = { SHARED_MEMORY.lock().await.get_position_pid_gains() };
                                let value = conversion::pid_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::UqUdLimit => {
                                let value = { SHARED_MEMORY.lock().await.get_uq_ud_limit() };
                                let value = conversion::i16_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::BusVoltage => {
                                let value = { SHARED_MEMORY.lock().await.get_bus_voltage() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::Temperature => {
                                let board_values =
                                    { SHARED_MEMORY.lock().await.get_board_temperature() };
                                let motor_value =
                                    { SHARED_MEMORY.lock().await.get_motor_temperature() };
                                // concatenate the values
                                let mut all_values = [0.0; config::N_AXIS + 1];
                                all_values[0..(config::N_AXIS)].copy_from_slice(&board_values);
                                all_values[config::N_AXIS] = motor_value;
                                let value = conversion::float_to_bytes(all_values);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::TorqueFluxLimit => {
                                let value = { SHARED_MEMORY.lock().await.get_torque_flux_limit() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::TorqueFluxLimitMax => {
                                let value =
                                    { SHARED_MEMORY.lock().await.get_torque_flux_limit_max() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::VelocityLimit => {
                                let value = { SHARED_MEMORY.lock().await.get_velocity_limit() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::VelocityLimitMax => {
                                let value = { SHARED_MEMORY.lock().await.get_velocity_limit_max() };
                                let value = conversion::float_to_bytes(value);
                                let sp = StatusPacket::with_value(id, dxl_error, value);
                                trace!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            #[cfg(feature = "orbita3d")]
                            DynamixelRegister::IndexSensor => {
                                let value = { SHARED_MEMORY.lock().await.get_index_sensor() };
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

                                let value = { SHARED_MEMORY.lock().await.get_full_state() };
                                let value = conversion::float_to_bytes(value);

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
                            DynamixelRegister::BoardState => {
                                let newstate = write_data_packet.data;
                                {
                                    SHARED_MEMORY
                                        .lock()
                                        .await
                                        .set_error_state(BoardStatus::from_u8(newstate[0]));
                                }
                                let sp = StatusPacket::ack(id, dxl_error);
                                debug!("Sending status packet: {:?} {:#x}", sp, sp.to_bytes());
                                if let Some(e) = dxl.write(&sp).await.err() {
                                    error!("Error: {:?}", e);
                                }
                            }

                            DynamixelRegister::TorqueEnable => {
                                match conversion::bytes_to_bool(write_data_packet.data) {
                                    Ok(torque_on) => {
                                        {
                                            SHARED_MEMORY.lock().await.set_torque_on(torque_on);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::FluxPID => {
                                match conversion::bytes_to_pid(write_data_packet.data) {
                                    Ok(gains) => {
                                        {
                                            SHARED_MEMORY.lock().await.set_flux_pid_gains(gains);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::TorquePID => {
                                match conversion::bytes_to_pid(write_data_packet.data) {
                                    Ok(gains) => {
                                        {
                                            SHARED_MEMORY.lock().await.set_torque_pid_gains(gains);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::VelocityPID => {
                                match conversion::bytes_to_pid(write_data_packet.data) {
                                    Ok(gains) => {
                                        {
                                            SHARED_MEMORY
                                                .lock()
                                                .await
                                                .set_velocity_pid_gains(gains);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::PositionPID => {
                                match conversion::bytes_to_pid(write_data_packet.data) {
                                    Ok(gains) => {
                                        {
                                            SHARED_MEMORY
                                                .lock()
                                                .await
                                                .set_position_pid_gains(gains);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::UqUdLimit => {
                                match conversion::bytes_to_i16(write_data_packet.data) {
                                    Ok(limits) => {
                                        {
                                            SHARED_MEMORY.lock().await.set_uq_ud_limit(limits);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::TorqueFluxLimit => {
                                match conversion::bytes_to_float(write_data_packet.data) {
                                    Ok(limits) => {
                                        {
                                            SHARED_MEMORY
                                                .lock()
                                                .await
                                                .set_torque_flux_limit(limits);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::TorqueFluxLimitMax => {
                                match conversion::bytes_to_float(write_data_packet.data) {
                                    Ok(limits) => {
                                        {
                                            SHARED_MEMORY
                                                .lock()
                                                .await
                                                .set_torque_flux_limit_max(limits);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::VelocityLimit => {
                                match conversion::bytes_to_float(write_data_packet.data) {
                                    Ok(limits) => {
                                        {
                                            SHARED_MEMORY.lock().await.set_velocity_limit(limits);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::VelocityLimitMax => {
                                match conversion::bytes_to_float(write_data_packet.data) {
                                    Ok(limits) => {
                                        {
                                            SHARED_MEMORY
                                                .lock()
                                                .await
                                                .set_velocity_limit_max(limits);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::FeedforwardVelocity => {
                                match conversion::bytes_to_float(write_data_packet.data) {
                                    Ok(feedforward) => {
                                        {
                                            SHARED_MEMORY
                                                .lock()
                                                .await
                                                .set_velocity_feedforward(feedforward);
                                        }
                                        let sp = StatusPacket::ack(id, dxl_error);
                                        debug!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            DynamixelRegister::TargetPosition => {
                                match conversion::bytes_to_float(write_data_packet.data) {
                                    Ok(target) => {
                                        {
                                            SHARED_MEMORY.lock().await.set_target_position(target);
                                        }
                                        // set zero feedforward velocity
                                        {
                                            SHARED_MEMORY
                                                .lock()
                                                .await
                                                .set_velocity_feedforward([0.0; config::N_AXIS]);
                                        }

                                        //return the full state
                                        let value =
                                            { SHARED_MEMORY.lock().await.get_current_position() };
                                        // let value = { SHARED_MEMORY.lock().await.get_full_state() };
                                        let value = conversion::float_to_bytes(value);

                                        let sp = StatusPacket::with_value(id, dxl_error, value);
                                        trace!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }
                            // parse the target position and velocity feedforward message
                            DynamixelRegister::TargetPositionWithVelocityFF => {
                                let pos_data = &write_data_packet.data[0..4 * config::N_AXIS];
                                let vel_data =
                                    &write_data_packet.data[4 * config::N_AXIS..8 * config::N_AXIS];
                                // set the position target
                                match conversion::bytes_to_float(pos_data) {
                                    Ok(target) => {
                                        {
                                            SHARED_MEMORY.lock().await.set_target_position(target);
                                        }
                                        // set the velocity feedforward
                                        // only if the position target is set
                                        match conversion::bytes_to_float(vel_data) {
                                            Ok(velocity_feedforward) => {
                                                {
                                                    SHARED_MEMORY
                                                        .lock()
                                                        .await
                                                        .set_velocity_feedforward(
                                                            velocity_feedforward,
                                                        );
                                                }

                                                //return the full state
                                                let value = {
                                                    SHARED_MEMORY
                                                        .lock()
                                                        .await
                                                        .get_current_position()
                                                };
                                                // let value = { SHARED_MEMORY.lock().await.get_full_state() };
                                                let value = conversion::float_to_bytes(value);

                                                let sp =
                                                    StatusPacket::with_value(id, dxl_error, value);
                                                trace!(
                                                    "Sending status packet: {:?} {:#x}",
                                                    sp,
                                                    sp.to_bytes()
                                                );
                                                if let Some(e) = dxl.write(&sp).await.err() {
                                                    error!("Error: {:?}", e);
                                                }
                                            }
                                            Err(e) => {
                                                error!("Error: {:?}", e);
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
                                }
                            }

                            // parse the target position and velocity feedforward message
                            DynamixelRegister::TargetPositionEstimateVelocityFF => {
                                match conversion::bytes_to_float(write_data_packet.data) {
                                    Ok(target) => {
                                        // save the old target position
                                        let old_target =
                                            { SHARED_MEMORY.lock().await.get_target_position() };

                                        // get the timestamp of the last target set
                                        // in order to calculate the velocity feedforward
                                        let timestamp = {
                                            SHARED_MEMORY.lock().await.get_target_set_timestamp()
                                        };
                                        match timestamp {
                                            Some(target_set_timestamp) => {
                                                // calculate the time elapsed from the last target set
                                                let dt = target_set_timestamp.elapsed().as_micros()
                                                    as f32
                                                    / 1_000_000.0;
                                                // calculate the velocity feedforward
                                                let mut velocity_feedforward =
                                                    [0.0; config::N_AXIS];
                                                //vel = (target - old_target)/dt;
                                                velocity_feedforward
                                                    .iter_mut()
                                                    .zip(target.iter().zip(old_target.iter()))
                                                    .for_each(|(v, (t, o))| {
                                                        *v = (*t - *o) / dt as f32;
                                                    });
                                                // write the velocity feedforward to the memory
                                                {
                                                    SHARED_MEMORY
                                                        .lock()
                                                        .await
                                                        .set_velocity_feedforward(
                                                            velocity_feedforward,
                                                        );
                                                }
                                            }
                                            None => {
                                                // if there is no timestamp, set the velocity feedforward to zero
                                                // ex. first time setting the target position
                                                {
                                                    SHARED_MEMORY
                                                        .lock()
                                                        .await
                                                        .set_velocity_feedforward(
                                                            [0.0; config::N_AXIS],
                                                        );
                                                }
                                            }
                                        }
                                        // set the new target position - after the feedforward not to modify the target_set_timestamp
                                        {
                                            SHARED_MEMORY.lock().await.set_target_position(target);
                                        }

                                        //return the full state
                                        let value =
                                            { SHARED_MEMORY.lock().await.get_current_position() };
                                        // let value = { SHARED_MEMORY.lock().await.get_full_state() };
                                        let value = conversion::float_to_bytes(value);
                                        let sp = StatusPacket::with_value(id, dxl_error, value);
                                        trace!(
                                            "Sending status packet: {:?} {:#x}",
                                            sp,
                                            sp.to_bytes()
                                        );
                                        if let Some(e) = dxl.write(&sp).await.err() {
                                            error!("Error: {:?}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Error: {:?}", e);
                                    }
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
            Err(e) => match e {
                dynamixel::usart_io::CommunicationError::DynamixelParsingError(
                    ParsingError::IgnorePacket(id1, id2),
                ) => {
                    trace!("Ignoring packet with id {} (I am {}).", id2, id1);
                }
                _ => {
                    error!("Error: {:?}", e);
                }
            },
        }

        // Timer::after(Duration::from_micros(1)).await;
    }
}
