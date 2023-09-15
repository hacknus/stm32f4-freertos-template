use embedded_hal::digital::v2::OutputPin;

pub struct LED<PIN> {
    pin: PIN,
    state: bool,
}

#[allow(dead_code)]
impl<PIN, E> LED<PIN>
where
    PIN: OutputPin<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(pin: PIN) -> Self {
        LED { pin, state: false }
    }

    pub fn toggle(&mut self) {
        if self.state {
            self.state = false;
            self.pin.set_low().unwrap();
        } else {
            self.state = true;
            self.pin.set_high().unwrap();
        }
    }

    pub fn on(&mut self) {
        self.state = true;
        self.pin.set_high().unwrap();
    }

    pub fn off(&mut self) {
        self.state = false;
        self.pin.set_low().unwrap();
    }
}
