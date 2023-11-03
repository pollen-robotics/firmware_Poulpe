use embassy_futures::join;
use embassy_stm32::spi::Error;

use super::axis::Axis;
use super::ventouse::VentouseKind;

pub struct Actuator<const N: usize> {
    axes: [VentouseKind; N],
}

impl<const N: usize> Actuator<N> {
    pub fn new(axes: [VentouseKind; N]) -> Self {
        Self { axes }
    }

    pub async fn init(&mut self) {
        join::join_array(self.axes.each_mut().map(|v| v.init())).await;
    }

    pub fn get_actual_position(&mut self) -> Result<[i32; N], Error> {
        let mut positions = [0; N];
        for (v, p) in self.axes.iter_mut().zip(positions.iter_mut()) {
            *p = v.get_actual_position()?;
        }
        Ok(positions)
    }

    pub fn set_target_position(&mut self, position: [i32; N]) -> Result<(), Error> {
        for (v, p) in self.axes.iter_mut().zip(position) {
            v.set_target_position(p)?;
        }
        Ok(())
    }
}
