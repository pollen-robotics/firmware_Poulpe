pub trait Axis {
    fn init(&mut self);

    fn get_actual_position(&mut self) -> Result<(), ()>;
    fn set_target_position(&mut self, position: i32) -> Result<(), ()>;
}
