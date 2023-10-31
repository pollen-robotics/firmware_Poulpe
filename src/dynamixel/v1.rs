use defmt::{error, Format};
use embassy_stm32::{
    gpio::{low_level::Pin, AnyPin},
    usart::{BasicInstance, Uart},
};
use embassy_time::{Duration, Timer};

use crate::{UART_SLEEP_US_DIRHIGH, UART_SLEEP_US_DIRLOW};

use super::{packet::StatusPacket, InstructionPacket};

const MAX_BUFFER_LENGTH: usize = 256;

pub struct DynamixelSerialV1<'d, T, TxDma, RxDma>
where
    T: BasicInstance,
    TxDma: embassy_stm32::usart::TxDma<T>,
    RxDma: embassy_stm32::usart::RxDma<T>,
{
    usart: Uart<'d, T, TxDma, RxDma>,
    dir: AnyPin,
    id: u8,
}

impl<'d, T, TxDma, RxDma> DynamixelSerialV1<'d, T, TxDma, RxDma>
where
    T: BasicInstance,
    TxDma: embassy_stm32::usart::TxDma<T>,
    RxDma: embassy_stm32::usart::RxDma<T>,
{
    pub fn new(usart: Uart<'d, T, TxDma, RxDma>, dir: AnyPin, id: u8) -> Self {
        DynamixelSerialV1 { usart, dir, id }
    }

    pub async fn read_instruction_packet(&mut self) -> Result<InstructionPacket, DynamixelError> {
        self.dir.set_low();
        Timer::after(Duration::from_micros(UART_SLEEP_US_DIRLOW)).await;

        let mut buffer = [0u8; MAX_BUFFER_LENGTH];

        let res = self.usart.read_until_idle(&mut buffer).await;

        match res {
            Ok(n) => InstructionPacket::parse(&buffer[..n], self.id),
            Err(e) => {
                error!("Error reading from USART: {:?}", e);
                return Err(DynamixelError {});
            }
        }
    }

    pub async fn send_status_packet(&mut self, sp: &StatusPacket) -> Result<(), DynamixelError> {
        self.dir.set_high();
        Timer::after(Duration::from_micros(UART_SLEEP_US_DIRHIGH)).await;

        let mut buffer = [0u8; MAX_BUFFER_LENGTH];
        let n = sp.to_bytes(&mut buffer);

        match self.usart.write(&buffer[..n]).await {
            Ok(_) => Ok(()),
            Err(_) => Err(DynamixelError {}),
        }
    }
}

#[derive(Format)]
pub struct DynamixelError {}
