use core::array;

use embassy_futures::join;

use super::foc::MotionMode;
use super::motors_io::{Pid, RawMotorsIO};

use crate::config::{self, BrushlessMotor};
use crate::config::DonutHall;

use super::ventouse::VentouseKind;
use crate::sensors::{sensors::SensorKind, sensors_io::RawSensorsIO, sensors_io};
use crate::utils::errors::{IOError, Result};
use defmt::{debug, error, info, warn};
use micromath::F32Ext;
use embassy_time::{Timer,Duration};

use crate::state_machine::poulpe_state::HomingErrorFlag;
use crate::SHARED_MEMORY;


const PI: f32 = 3.141592653589793;
const TAU: f32 = 6.283185307179586;

pub struct Actuator<'d, const N: usize> {
    axes: [VentouseKind<'d>; N],
    sensors: [SensorKind<'d>; N],
    #[cfg(feature = "orbita3d")]
    index_sensor: [u8; N],
    inverted: f32, //FIXME: horrible...
    hardware_zeros: [f32; N],
}

impl<'d, const N: usize> Actuator<'d, N> {
    #[cfg(feature = "orbita3d")]
    pub fn new(axes: [VentouseKind<'d>; N], sensors: [SensorKind<'d>; N]) -> Self {
        Self {
            axes,
            sensors,
            index_sensor: [0xff; N],
            inverted: -1.0,
            hardware_zeros: [0.0; N],
        }
    }
    #[cfg(feature = "orbita2d")]
    pub fn new(axes: [VentouseKind<'d>; N], sensors: [SensorKind<'d>; N]) -> Self {
        Self {
            axes,
            sensors,
            inverted: 1.0,
            hardware_zeros: [0.0; N],
        }
    }

    pub async fn init(&mut self) -> [Result<()>; N] {
        join::join_array(self.axes.each_mut().map(|v| v.init())).await
    }

    // check motors
    pub async fn check_motors_1(&mut self) -> [Result<()>; N] {
        join::join_array(self.axes.each_mut().map(|v| v.check_motors_1())).await
    }
    pub async fn check_motors_2(&mut self) -> [Result<()>; N] {
        join::join_array(self.axes.each_mut().map(|v| v.check_motors_2())).await
    }
    // pub fn get_ventouse(&mut self, v: char) -> Option<&mut dyn RawMotorsIO<1>> {
    //     match v {
    //         'A' => self.axes[0].get_ventouse('A'),
    //         'B' => self.axes[1].get_ventouse('B'),
    //         'C' => self.axes[2].get_ventouse('C'),
    //         _ => None,
    //     }
    // }

    // check the state of each
    pub fn check_driver_states(&mut self) -> [Result<()>; N] {
        array::from_fn(|i| match self.axes[i].get_driver_state() {
            Ok(_) => Ok(()),
            Err(_) => Err(IOError::DriverError),
        })
    }

    pub fn check_driver_communication(&mut self) -> [bool; N] {
        array::from_fn(|i| match self.axes[i].get_control_mode() {
            Ok(_) => true,
            Err(_) => false,
        })
    }



    pub fn get_axis(&mut self, idx: usize) -> &mut dyn RawMotorsIO<1> {
        &mut self.axes[idx]
    }

    #[cfg(feature = "orbita3d")]
    pub fn get_index_sensor(&mut self) -> [u8; N] {
        self.index_sensor
    }

    #[cfg(feature = "orbita3d")]
    pub fn set_index_sensor(&mut self, index: [u8; N]) {
        self.index_sensor = index;
    }

    #[cfg(feature = "orbita3d")]
    pub fn compute_offset(
        &mut self,
        hall_idx: [u8; N],
        hardware_zero: [f32; N],
    ) -> Result<([f32; N], [i16; N])> {
        use crate::config::ORBITA3D_HALL_ZERO_IDX;

        let mut zero_hall_offsets: [f32; 3] = [0.0, 0.0, 0.0]; //orbita domain

        zero_hall_offsets[0] = hall_diff(hall_idx[0], ORBITA3D_HALL_ZERO_IDX[0])
            * 22.5_f32.to_radians()
            + 11.25_f32.to_radians();

        zero_hall_offsets[1] = hall_diff(hall_idx[1], ORBITA3D_HALL_ZERO_IDX[1])
            * 22.5_f32.to_radians()
            + 3.75_f32.to_radians();

        zero_hall_offsets[2] = hall_diff(hall_idx[2], ORBITA3D_HALL_ZERO_IDX[2])
            * 22.5_f32.to_radians()
            - 3.75_f32.to_radians();

        let mut found_turn: [i16; N] = [0; N];

        //TODO match and check errors NaN
        let mut current_pos = [0.0; N];
        let axis_read = self.get_axis_sensors(); //gearbox domain
        for (i, val) in axis_read.iter().enumerate() {
            match val {
                Ok(v) => {
                    // check if nan 
                    if v.is_nan() {
                        return Err(IOError::InitError);
                    }
                    current_pos[i] = *v;
                },
                Err(e) => return Err(IOError::InitError),
            }
        }

        let reductions = 1.0 / self.axes[0].get_ratio(); //5.3333

        current_pos[0] /= reductions; //orbita domain
        current_pos[1] /= reductions;
        current_pos[2] /= reductions;
        let mut hardware_zero_orbita = [0.0, 0.0, 0.0];

        // Should be in Orbita domain
        hardware_zero_orbita[0] = hardware_zero[0]; // / reductions;
        hardware_zero_orbita[1] = hardware_zero[1]; // / reductions;
        hardware_zero_orbita[2] = hardware_zero[2]; // / reductions;

        let mut offsets: [f32; N] = [0.0; N];
        hardware_zero_orbita
            .iter()
            .zip(current_pos.iter())
            .zip(hall_idx.iter())
            .zip(zero_hall_offsets.iter())
            .enumerate()
            .for_each(
                |(i, (((&hardware_zero, &current_pos), &hall_idx), &hall_zero))| {
                    let res = find_position_with_hall(
                        current_pos,
                        hardware_zero,
                        hall_zero,
                        hall_idx,
                        reductions,
                    );
                    offsets[i] = res.0;
                    found_turn[i] = res.1;
                },
            );
        debug!("Offsets: {:?}, turns: {:?}", offsets, found_turn);

        // // Security, did we found the same number of turn for each arm? (FIXME?)
        // if !(found_turn[0] == found_turn[1] && found_turn[1] == found_turn[2]) {
        //     log::error!("HallZero: Incoherent offsets!!");
        //     controller.offsets[0] = None;
        //     controller.offsets[1] = None;
        //     controller.offsets[2] = None;
        //     return Err(Box::new(MissingResisterErrror(
        //         "Hall sensor not found".to_string(),
        //     )));
        // }
        // offsets[0] *= reductions; //gearbox domain
        // offsets[1] *= reductions;
        // offsets[2] *= reductions;

        // offsets[0] %= TAU;
        // offsets[1] %= TAU;
        // offsets[2] %= TAU;

        Ok((offsets, found_turn))
    }

    pub fn get_hardware_zeros(&mut self) -> Result<[f32; N]> {
        Ok(self.hardware_zeros)
    }
    pub fn set_hardware_zeros(&mut self, zeros: [f32; N]) -> Result<()> {
        self.hardware_zeros = zeros;
        Ok(())
    }


    // read the axis sensors
    // disables the torque to avoid the noise
    // make a few tries to avoid nan values and errors
    // if there is an error, return an error
    pub async fn robust_read_axis_sensors(
        &mut self,
        n_read_tries: u8,
        n_read: u8,
    ) -> Result<[f32; N]> {

        let mut sensor_reads: [f32; N] = [0.0; N];
        for (i, s) in self.sensors.iter_mut().enumerate(){
            sensor_reads[i] = sensors_io::get_axis_sensors_robust(s, n_read_tries, n_read as u32).await?[0];
        } 
        #[cfg(feature = "orbita3d")]
        sensor_reads.swap(1, 2); // because of the wiring in Orbita3D
        
        // wait a bit to make sure the torque is enabled
        Timer::after(Duration::from_micros(100000)).await;

        Ok(sensor_reads)
    }


    //Find index for Orbita3D motors
    #[cfg(feature = "orbita3d")]
    pub async fn find_index_orbita3d(&mut self, donut_hall: &mut config::DonutHall<'d>) -> Result<HomingErrorFlag> {
        //FIXME:
        // - Maybe torque off is not so good, moving motor can induce motion in the torque off motor...

        // self.set_torque([false, false, false]).unwrap();

        let indices = self.find_index(donut_hall).unwrap_or_else(|e| {
            error!("Error finding index: {:?}", e);
            [255; N]
        });
        info!("Found indices: {:?}", indices);
        //TODO retry if 255 or duplicate

        if indices.contains(&255) {
            // errors in finding the Hall
            error!("Bad index!");
            #[cfg(not(feature = "ignore_errors"))]
            return Ok(HomingErrorFlag::IndexSearchFail);
        }
        if (1..indices.len()).any(|i| indices[i..].contains(&indices[i - 1])) {
            //thanks Stackoverflow
            error!("Duplicate index!");
            #[cfg(not(feature = "ignore_errors"))]
            return Ok(HomingErrorFlag::IndexSearchFail);
        }

        self.set_index_sensor(indices);
        self.set_torque([false; N]).unwrap(); //be sure to torque off to avoid noise in axis sensors?
        Timer::after(Duration::from_millis(10));

        // let zeros = [1.0193205177783966, 0.7377220094203949, 0.4328247159719467]; //Orbita domain
        let zeros = self.get_hardware_zeros()?;

        if zeros[0] == zeros[1] && zeros[1] == zeros[2] && zeros[0] == 0.0 {
            //Forgot to pass the zeros as argument! FIXME switch to a different zeroing mode?
            // => assuming HallZero mode
            error!("No zero given in paramter! => HallZero mode");
            // Set the initial position to the axis sensor values (used for pc-side "sofwtare" zeroring )

            // read the current axis sensors positions
            let mut init_sensors = match self.robust_read_axis_sensors(10, 100).await {
                Ok(sensor_values) => {
                    debug!("init sensors: {:?}", sensor_values);
                    sensor_values
                }
                Err(e) => {
                    error!("Error reading axis sensors: {:?}", e);
                    #[cfg(not(feature = "ignore_errors"))]
                    return Ok(HomingErrorFlag::AxisSensorReadFail);
                    #[cfg(feature = "ignore_errors")]
                    [0.0; N] // use the default value if ignoring errors
                }
            };

            init_sensors.iter_mut().for_each(|x| *x = wrap_to_pi(*x));
            debug!("init axis sensors: {:?}", init_sensors);
            match self.set_current_position(init_sensors) {
                Ok(_) => {
                    SHARED_MEMORY
                        .lock()
                        .await
                        .set_current_position(init_sensors[..N].try_into().unwrap());
                    // stupid rust thing to convert N array to N_AXIS array (N = 3, N_AXIS = 3)
                }
                Err(e) => {
                    error!("Error setting current position: {:?}", e);
                    #[cfg(not(feature = "ignore_errors"))]
                    return Ok(HomingErrorFlag::ZeroingFail);
                }
            }
            // #[cfg(feature = "orbita3d")]
            match self.set_target_position(init_sensors) {
                Ok(_) => {
                    SHARED_MEMORY
                        .lock()
                        .await
                        .set_target_position(init_sensors[..N].try_into().unwrap());
                    // stupid rust thing to convert N array to N_AXIS array (N = 3, N_AXIS = 3)
                }
                Err(e) => {
                    error!("Error setting target position: {:?}", e);
                    #[cfg(not(feature = "ignore_errors"))]
                    return Ok(HomingErrorFlag::ZeroingFail);
                }
            }
            return Ok(HomingErrorFlag::None);
        } else {
            info!("Hardware zeros: {:?}", zeros);
            let (mut offsets, found_turn) = self.compute_offset(indices, zeros).unwrap();

            if !(found_turn[0] == found_turn[1] && found_turn[1] == found_turn[2]) {
                //It may be possible in certain case?? But better forbid this
                error!("Incoherent number of turn found! {:?}", found_turn);
                #[cfg(not(feature = "ignore_errors"))]
                return Ok(HomingErrorFlag::ZeroingFail);
            }
            if offsets.iter().any(|&x| x.is_nan()) {
                // Check for NaN
                error!("Bad offsets! {:?}", offsets);
                #[cfg(not(feature = "ignore_errors"))]
                return Ok(HomingErrorFlag::ZeroingFail);
            }

            // read the current axis sensors positions
            let curpos = match self.robust_read_axis_sensors(10, 100).await {
                Ok(sensor_values) => {
                    debug!("init sensors: {:?}", sensor_values);
                    sensor_values
                }
                Err(e) => {
                    error!("Error reading axis sensors: {:?}", e);
                    #[cfg(not(feature = "ignore_errors"))]
                    return Ok(HomingErrorFlag::AxisSensorReadFail);
                    #[cfg(feature = "ignore_errors")]
                    [0.0; N] // use the default value if ignoring errors
                }
            };

            offsets[0] *= -1.0 / BrushlessMotor::ecx22().axis_ratio();
            offsets[1] *= -1.0 / BrushlessMotor::ecx22().axis_ratio();
            offsets[2] *= -1.0 / BrushlessMotor::ecx22().axis_ratio();

            offsets[0] += curpos[0];
            offsets[1] += curpos[1];
            offsets[2] += curpos[2];

            debug!("indices: {:?} offsets: {:?}", indices, offsets);
            self.set_current_position(offsets)?;
            return Ok(HomingErrorFlag::None);
        }
    }

    #[cfg(feature = "orbita2d")]
    pub async fn find_index_orbita2d(&mut self)  -> Result<HomingErrorFlag> {
        self.set_torque([false; N])?; //be sure to torque off to avoid noise in axis sensors?
        Timer::after(Duration::from_millis(10));
        // let zeros = [5.236674785614014, 1.6637036800384521]; //Orbita domain
        let zeros = self.get_hardware_zeros()?;

        if zeros[0] == zeros[1] && zeros[0] == 0.0 {
            //Forgot to pass the zeros as argument! FIXME switch to a different zeroing mode?
            error!("No zero given in paramter! => Relative zero mode");
            // do nothing
        } else {
            info!("Hardware zeros: {:?}", zeros);
            // read the axis sensors
            let curaxis = match self.robust_read_axis_sensors(10, 100).await {
                Ok(sensor_values) => {
                    debug!("init sensors: {:?}", sensor_values);
                    sensor_values
                }
                Err(e) => {
                    error!("Error reading axis sensors: {:?}", e);
                    #[cfg(not(feature = "ignore_errors"))]
                    return Ok(HomingErrorFlag::AxisSensorReadFail);
                    #[cfg(feature = "ignore_errors")]
                    [0.0; N] // use the default value if ignoring errors
                }
            };
    
            let mut axis_offset = curaxis;
            axis_offset[0] -= zeros[0];
            axis_offset[1] -= zeros[1];
            let mut motor_offsets = self.orbita2d_inverse_kinematics(axis_offset);

            // read the current motor posiitons
            let curpos = self.get_current_position()?;
            motor_offsets[0] += curpos[0];
            motor_offsets[1] += curpos[1];
            // set the offset
            self.set_current_position(motor_offsets)?;
        }
        return Ok(HomingErrorFlag::None);
    }

    #[cfg(feature = "orbita2d")]
    pub fn orbita2d_inverse_kinematics(&mut self, axis: [f32; N]) -> [f32; N] {
        let r = 1.0; // no axis ratio for Orbita2D
        #[cfg(feature = "ec45")]
        let r = 1.0 / BrushlessMotor::ec45().axis_ratio();
        #[cfg(feature = "ec60")]
        let r = 1.0 / BrushlessMotor::ec60().axis_ratio();

        let axis_pos= [
            wrap_to_pi(axis[0]),
            wrap_to_pi(axis[1]),
        ];

        let mut motor_pos = [0.0; N];
        // inverse kinematics
        motor_pos[0] = -(r * axis_pos[0] + r * axis_pos[1]);
        motor_pos[1] = -(r * axis_pos[0] - r * axis_pos[1]);

        motor_pos
    }

    #[cfg(feature = "orbita2d")]
    pub fn orbita2d_forward_kinematics(&mut self, motor_pos: [f32; N]) -> [f32; N] {
        let r = 1.0; // no axis ratio for Orbita2D
        #[cfg(feature = "ec45")]
        let r = 1.0 / BrushlessMotor::ec45().axis_ratio();
        #[cfg(feature = "ec60")]
        let r = 1.0 / BrushlessMotor::ec60().axis_ratio();

        let mut axis_pos = [0.0; N];
        // forward kinematics
        axis_pos[0] = (-motor_pos[0] - motor_pos[1]) / r / 2.0;
        axis_pos[1] = (-motor_pos[0] + motor_pos[1]) / r / 2.0;

        axis_pos[0] = wrap_to_pi(axis_pos[0]);
        axis_pos[1] = wrap_to_pi(axis_pos[1]);
        axis_pos
    }

}


// function wrapping an angle in radians to
// the range [-pi, pi]
pub fn wrap_to_pi(angle: f32) -> f32 {
    const PI: f32 = 3.14159265359;
    (((angle + PI) % (2.0 * PI)) + (2.0 * PI)) % (2.0 * PI) - PI
}

pub fn angle_diff(angle_a: f32, angle_b: f32) -> f32 {
    let mut angle = angle_a - angle_b;
    angle = (angle + PI) % TAU - PI;
    if angle < -PI {
        angle + TAU
    } else {
        angle
    }
}

pub fn hall_diff(hall_a: u8, hall_b: u8) -> f32 {
    // shortest hall difference (16 discrete Hall)
    let d: f32 = hall_a as f32 - hall_b as f32;
    if d >= 0.0 {
        if d >= 8.0 {
            d - 16.0
        } else {
            d
        }
    } else if d >= -8.0 {
        d
    } else {
        d + 16.0
    }
}

fn find_position_with_hall(
    current_position: f32,
    hardware_zero: f32,
    hall_zero: f32,
    hall_index: u8,
    reduction: f32,
) -> (f32, i16) {
    const MAX_TURN: usize = 3;
    let mut offset: [f32; MAX_TURN] = [0.0; MAX_TURN];
    let mut offset_search: [f32; MAX_TURN] = [0.0; MAX_TURN];
    let turn_offset = 2.0 * PI * reduction;
    let hall_offset = 2.0 * PI / 16.0 * reduction; //22.5° disk for each Hall sensor

    // let hall_diff = hall_diff(hall_index, hall_zero);

    let diff_gear = current_position * reduction - hardware_zero * reduction;
    let shortest_diff_gear = angle_diff(current_position * reduction, hardware_zero * reduction); //nul FIXME
    let shortest_to_zero = angle_diff(0.0, hardware_zero * reduction);

    let pos = (current_position * reduction) % TAU; //this should be the raw gearbox position
    let shortest_to_current = angle_diff(0.0, pos);
    let mut gearbox_turn = 0.0;

    debug!(
        "Diff: {:?} shortest diff: {:?} shortest_to_zero {:?} hall_zero_angle: {:?}",
        diff_gear, shortest_diff_gear, shortest_to_zero, hall_zero
    );

    for i in 0..offset.len() {
        // theoretical position of the gearbox starting from the zero and moving toward detected hall

        offset_search[i] = (hardware_zero * reduction) % TAU
            + (hall_zero * reduction) % TAU
            + ((i as f32 - (offset.len() / 2) as f32) * turn_offset) % TAU;
        offset_search[i] %= TAU;

        let residual = angle_diff(
            pos,
            (hardware_zero * reduction) % TAU + (hall_zero * reduction) % TAU,
        ) / reduction;

        // Offset to apply
        offset[i] = current_position
            - hall_zero
            - residual
            - (i as f32 - (offset.len() / 2) as f32)
                * (turn_offset / reduction - TAU * (reduction - reduction.floor()) / reduction);

        //in orbita ref
    }

    debug!(
        "Residual (gearbox) {:?} (orbita) {:?}",
        angle_diff(pos, (hall_zero * reduction) % TAU),
        angle_diff(pos, (hall_zero * reduction) % TAU) / reduction
    );
    debug!("possible offset (orbita domain): {:?}", offset);
    debug!("searching offset (gearbox domain): {:?}", offset_search);

    debug!(
        "current pos (gearbox): {:?} hardware_zero (gearbox): {:?} hall_idx: {:?} hall_zero: {:?} hall_offset: {:?} turn_offset: {:?}",
        pos,
        hardware_zero * reduction,
        hall_index as f32,
        hall_zero,
        hall_offset,
	turn_offset
    );

    let best = offset_search
        .iter()
        .map(|&p| {
            let d = angle_diff(p, pos).abs();
            debug!("Diff search: {:?}", d);
            d
        })
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| offset[i])
        .unwrap();

    let best_idx = offset.iter().position(|&x| x == best).unwrap();
    debug!(
        "best offset (orbita domain): {} gearbox domain: {:?}",
        best, offset_search[best_idx]
    );
    debug!(
        "It corresponds to {} turn (orbita domain)",
        best_idx as i16 - (offset.len() / 2) as i16
    );

    (best, best_idx as i16 - (offset.len() / 2) as i16)
}

// TODO: make this generic (how?)
impl<'d, const N: usize> RawMotorsIO<N> for Actuator<'d, N> {
    /// Check if the motors are ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; N]> {
        let mut res = [false; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.is_torque_on()?[0];
        }

        Ok(res)
    }
    /// Enable/Disable the torque
    fn set_torque(&mut self, on: [bool; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_torque([on[i]])?;
        }

        Ok(())
    }

    /// Get the control mode
    fn get_control_mode(&mut self) -> Result<[MotionMode; N]> {
        let mut res = [MotionMode::Stopped; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_control_mode()?[0];
        }

        Ok(res)
    }

    /// Set the control mode
    fn set_control_mode(&mut self, mode: MotionMode) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_control_mode(mode)?;
        }
        Ok(())
    }

    /// Get the current position of the motors (in radians)
    fn get_current_position(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = self.inverted * axis.get_current_position()?[0];
        }

        Ok(res)
    }

    fn set_current_position(&mut self, pos: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_current_position([self.inverted * pos[i]])?;
        }

        Ok(())
    }

    /// Get the current velocity of the motors (in radians per second)
    fn get_current_velocity(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = self.inverted * axis.get_current_velocity()?[0];
        }

        Ok(res)
    }
    /// Get the current torque of the motors (in Nm)
    fn get_current_torque(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = self.inverted * axis.get_current_torque()?[0];
        }

        Ok(res)
    }

    /// Get the current target position of the motors (in radians)
    fn get_target_position(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = self.inverted * axis.get_target_position()?[0];
        }

        Ok(res)
    }
    /// Set the current target position of the motors (in radians)
    fn set_target_position(&mut self, position: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_target_position([self.inverted * position[i]])?;
        }

        Ok(())
    }

    // torque feedforward
    fn set_torque_feedforward(&mut self, torque: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_torque_feedforward([self.inverted * torque[i]])?;
        }

        Ok(())
    }
    // get torque feedforward
    fn get_torque_feedforward(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = self.inverted * axis.get_torque_feedforward()?[0];
        }

        Ok(res)
    }

    // Set velocity feedforward
    fn set_velocity_feedforward(&mut self, velocity: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_velocity_feedforward([self.inverted * velocity[i]])?;
        }

        Ok(())
    }
    // get velocity feedforward
    fn get_velocity_feedforward(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = self.inverted * axis.get_velocity_feedforward()?[0];
        }

        Ok(res)
    }

    /// Get the current target velocity of the motors (in rpm)
    fn get_target_velocity(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = self.inverted * axis.get_target_velocity()?[0];
        }

        Ok(res)
    }
    /// Set the current target velocity of the motors (in rpm)
    fn set_target_velocity(&mut self, velocity: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_target_velocity([self.inverted * velocity[i]])?;
        }

        Ok(())
    }

    /// Get the current target torque of the motors (in ?? mA)
    fn get_target_torque(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = self.inverted * axis.get_target_torque()?[0];
        }

        Ok(res)
    }
    /// Set the current target torque of the motors (in ?? mA)
    fn set_target_torque(&mut self, torque: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_target_torque([self.inverted * torque[i]])?;
        }

        Ok(())
    }

    /// Get the velocity limit of the motors (in radians per second)
    fn get_velocity_limit(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_velocity_limit()?[0];
        }

        Ok(res)
    }
    /// Set the velocity limit of the motors (in radians per second)
    fn set_velocity_limit(&mut self, velocity: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_velocity_limit([velocity[i]])?;
        }

        Ok(())
    }

    /// Get the torque limit of the motors (in Nm)
    fn get_torque_flux_limit(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_torque_flux_limit()?[0];
        }

        Ok(res)
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_torque_flux_limit(&mut self, torque: [f32; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_torque_flux_limit([torque[i]])?;
        }

        Ok(())
    }

    /// Get the torque limit of the motors (in Nm)
    fn get_uq_ud_limit(&mut self) -> Result<[i16; N]> {
        let mut res = [0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_uq_ud_limit()?[0];
        }

        Ok(res)
    }
    /// Set the torque limit of the motors (in Nm)
    fn set_uq_ud_limit(&mut self, torque: [i16; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_uq_ud_limit([torque[i]])?;
        }

        Ok(())
    }

    ////////////////////

    /// Get the absolute velocity limit of the motors (in radians per second)
    fn get_velocity_limit_max(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_velocity_limit_max()?[0];
        }

        Ok(res)
    }
    /// Set the absolute velocity limit of the motors (in radians per second)
    // fn set_velocity_limit_max(&mut self, velocity: [f32; N]) -> Result<()> {
    //     for (i, axis) in self.axes.iter_mut().enumerate() {
    //         axis.set_velocity_limit_max([velocity[i]])?;
    //     }

    //     Ok(())
    // }

    /// Get the absolute torque limit of the motors (in Nm)
    fn get_torque_flux_limit_max(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_torque_flux_limit_max()?[0];
        }

        Ok(res)
    }
    /// Set the absolute torque limit of the motors (in Nm)
    // fn set_torque_flux_limit_max(&mut self, torque: [f32; N]) -> Result<()> {
    //     for (i, axis) in self.axes.iter_mut().enumerate() {
    //         axis.set_torque_flux_limit_max([torque[i]])?;
    //     }

    //     Ok(())
    // }

    /*
    /// Get the absolute torque limit of the motors (in Nm)
    fn get_uq_ud_limit_max(&mut self) -> Result<[i16; N]> {
        let mut res = [0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_uq_ud_limit_max()?[0];
        }

        Ok(res)
    }
    /// Set the absolute torque limit of the motors (in Nm)
    fn set_uq_ud_limit_max(&mut self, torque: [i16; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_uq_ud_limit_max([torque[i]])?;
        }

        Ok(())
    }
    */
    /////////////////////

    // get the driver states
    fn get_driver_state(&mut self) -> Result<[(); N]> {
        let mut res = [(); N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_driver_state()?[0];
        }

        Ok(res)
    }

    // get temperature
    fn get_board_temperature(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_board_temperature()?[0];
        }

        Ok(res)
    }

    // get DC bus voltage
    fn get_bus_voltage(&mut self) -> Result<[f32; N]> {
        let mut res = [0.0; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_bus_voltage()?[0];
        }

        Ok(res)
    }

    // /// Get the current PID gains of the motors
    // fn get_pid_gains(&mut self) -> Result<[Pid; N]> {
    //     let mut res = [Pid {
    //         p: 0,
    //         i: 0,
    //         // d: 0.0,
    //     }; N];
    //     for (i, axis) in self.axes.iter_mut().enumerate() {
    //         res[i] = axis.get_pid_gains()?[0];
    //     }
    //     Ok(res)
    // }
    // /// Set the current PID gains of the motors
    // fn set_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
    //     for (i, axis) in self.axes.iter_mut().enumerate() {
    //         axis.set_pid_gains([pid[i]])?;
    //     }

    //     Ok(())
    // }

    /// Get the current flux PID gains of the motors
    fn get_flux_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0,
            i: 0,
            // d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_flux_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current flux PID gains of the motors
    fn set_flux_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_flux_pid_gains([pid[i]])?;
        }

        Ok(())
    }

    /// Get the current torque PID gains of the motors
    fn get_torque_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0,
            i: 0,
            // d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_torque_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current torque PID gains of the motors
    fn set_torque_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_torque_pid_gains([pid[i]])?;
        }

        Ok(())
    }

    /// Get the current velocity PID gains of the motors
    fn get_velocity_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0,
            i: 0,
            // d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_velocity_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current velocity PID gains of the motors
    fn set_velocity_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_velocity_pid_gains([pid[i]])?;
        }

        Ok(())
    }

    /// Get the current position PID gains of the motors
    fn get_position_pid_gains(&mut self) -> Result<[Pid; N]> {
        let mut res = [Pid {
            p: 0,
            i: 0,
            // d: 0.0,
        }; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            res[i] = axis.get_position_pid_gains()?[0];
        }
        Ok(res)
    }
    /// Set the current position PID gains of the motors
    fn set_position_pid_gains(&mut self, pid: [Pid; N]) -> Result<()> {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            axis.set_position_pid_gains([pid[i]])?;
        }

        Ok(())
    }

    fn find_index(&mut self, donut_sensor: &mut DonutHall) -> Result<[u8; N]> {
        let mut indices: [u8; N] = [255; N];
        for (i, axis) in self.axes.iter_mut().enumerate() {
            let idx = axis.find_index(donut_sensor);
            match idx {
                Ok(val) => {
                    indices[i] = val[0];
                }
                Err(e) => indices[i] = 255,
            }
        }
        Ok(indices)
    }

   
}

impl<'d, const N: usize> RawSensorsIO<N> for Actuator<'d, N> {
    /// The axis sensor
    fn get_axis_sensors(&mut self) -> [Result<f32>; N] {
        let mut res = array::from_fn(|i| {
            match self.sensors[i].get_axis_sensors()[0] {
                Ok(val) => Ok(val),
                Err(_) => Err(IOError::SensorError),
            }
        });

        // FIXME: reordering the sensors because the Donut board is not in the same order as the motors...
        #[cfg(feature = "orbita3d")]
        res.swap(1, 2);
        return res;
    }
}
