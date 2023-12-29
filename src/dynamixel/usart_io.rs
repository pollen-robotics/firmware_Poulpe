use defmt::{debug, Format, trace};
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    usart::{BasicInstance, Uart},
};
use embassy_time::{Duration, Timer};

use super::packet::{InstructionPacketKind, ParsingError, StatusPacket};

const MAX_BUFFER_LENGTH: usize = 256;
//Seems ok at 115200 with LOG=Info
// const UART_SLEEP_US_DIRLOW: u64 = 200;
// const UART_SLEEP_US_DIRHIGH: u64 = 300;


// 2Mbauds
const UART_SLEEP_US_DIRLOW: u64 = 100;
const UART_SLEEP_US_DIRHIGH: u64 = 150;


const MAX_READ_BUFFER_LENGTH: usize = 32;

pub struct DynamixelUsartIO<'d, T, TxDma, RxDma>
where
    T: BasicInstance,
    TxDma: embassy_stm32::usart::TxDma<T>,
    RxDma: embassy_stm32::usart::RxDma<T>,
{
    usart: Uart<'d, T, TxDma, RxDma>,
    dir: Output<'d, AnyPin>,
    id: u8,

    read_buffer: [u8; MAX_BUFFER_LENGTH],
}

impl<'d, T, TxDma, RxDma> DynamixelUsartIO<'d, T, TxDma, RxDma>
where
    T: BasicInstance,
    TxDma: embassy_stm32::usart::TxDma<T>,
    RxDma: embassy_stm32::usart::RxDma<T>,
{
    pub fn new(usart: Uart<'d, T, TxDma, RxDma>, dir: AnyPin, id: u8) -> Self {
        let mut dir = Output::new(dir, Level::Low, Speed::High);
        dir.set_low(); // Switch to reading by default

        DynamixelUsartIO {
            usart,
            dir,
            id,
            read_buffer: [0u8; MAX_BUFFER_LENGTH],
        }
    }

    pub async fn read(&mut self) -> Result<InstructionPacketKind, CommunicationError> {
        // We should always be in read mode when this method is called

        let mut total = 0;

        loop {
            let n = match self
                .usart
                .read_until_idle(&mut self.read_buffer[total..])
                .await
            {
                Ok(n) => n,
                Err(e) => return Err(CommunicationError::UartError(e)),
            };


            total += n;

            assert!(total <= MAX_BUFFER_LENGTH - MAX_READ_BUFFER_LENGTH);

            if n == 0 {
                continue;
            }

            if n < MAX_READ_BUFFER_LENGTH { //FIXME continue until full packet... => parse packet
              break;

		/*
		if total>=4{
		    if self.read_buffer[0] == 0xff && self.read_buffer[1] == 0xff && self.read_buffer[2]== self.id
		    {
			let mut length = self.read_buffer[3]as usize;
			length+=4;
			if total>=length{ //FIXME might have more than one packet in the buffer...
			    total=length; //cut here
			    break;
			}
			else {
			    continue;
			}
		    }
		    else{
			break;
		    }

		}
		else {
		    continue;
		}
*/

            }
        }

        debug!("Read {} bytes", total);
	trace!("Read {:#x}", &self.read_buffer[..total]);

        InstructionPacketKind::parse(&self.read_buffer[..total], self.id)
            .map_err(CommunicationError::DynamixelParsingError)
    }

    pub async fn write<const N: usize>(
        &mut self,
        sp: &StatusPacket<N>,
    ) -> Result<(), CommunicationError>
    where
        [u8; N + 6]: Sized,
    {
        self.dir.set_high(); // Switch to writing
        Timer::after(Duration::from_micros(UART_SLEEP_US_DIRHIGH)).await;

        let res = self.usart.write(&sp.to_bytes()).await;

        Timer::after(Duration::from_micros(UART_SLEEP_US_DIRLOW)).await;
        self.dir.set_low(); // Switch to reading

        res.map_err(CommunicationError::UartError)
    }
}

#[derive(Format)]
pub enum CommunicationError {
    UartError(embassy_stm32::usart::Error),
    DynamixelParsingError(ParsingError),
}
