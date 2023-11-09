use embassy_stm32::{
    flash::{Bank1Region, Blocking, Flash},
    peripherals::FLASH,
    Peripheral,
};

use crate::config;

const USER_DATA_ADDR: u32 = 0x8_0000;

pub struct Storage {
    bank: Option<Bank1Region<'static, Blocking>>,
}

enum Addr {
    Id,
}
impl Addr {
    fn offset(&self) -> u32 {
        match *self {
            Addr::Id => 0,
        }
    }
}

impl Storage {
    pub const fn default() -> Self {
        Storage { bank: None }
    }

    pub fn init(&mut self, p: impl Peripheral<P = FLASH> + 'static) {
        self.bank = Some(Flash::new_blocking(p).into_blocking_regions().bank1_region);
    }

    fn read(&mut self, addr: Addr, buff: &mut [u8]) {
        self.bank
            .as_mut()
            .unwrap()
            .blocking_read(USER_DATA_ADDR + addr.offset(), buff)
            .unwrap();
    }
    fn write(&mut self, addr: Addr, data: &[u8]) {
        self.bank
            .as_mut()
            .unwrap()
            .blocking_write(USER_DATA_ADDR + addr.offset(), data)
            .unwrap();
    }
}

impl Storage {
    pub fn get_id(&mut self) -> u8 {
        let mut buff = [0_u8; 1];
        self.read(Addr::Id, &mut buff);

        let id = buff[0];
        if id != 0xff {
            id
        } else {
            config::DYNAMIXEL_DEFAULT_ID
        }
    }
    pub fn set_id(&mut self, new_id: u8) {
        self.write(Addr::Id, &[new_id]);
    }
}
