use super::axis::Axis;
use super::ventouse::VentouseKind;

pub struct Actuator<const N: usize> {
    axes: [VentouseKind; N],
}

impl<const N: usize> Actuator<N> {
    pub fn new(axes: [VentouseKind; N]) -> Self {
        Self { axes }
    }

    pub fn init(&mut self) {
        for v in self.axes.iter_mut() {
            v.init();
        }
    }

    pub fn get_actual_position(&mut self) -> Result<(), ()> {
        for v in self.axes.iter_mut() {
            v.get_actual_position()?;
        }
        Ok(())
    }

    pub fn set_target_position(&mut self, position: [i32; N]) -> Result<(), ()> {
        for (v, p) in self.axes.iter_mut().zip(position) {
            v.set_target_position(p)?;
        }
        Ok(())
    }
}
