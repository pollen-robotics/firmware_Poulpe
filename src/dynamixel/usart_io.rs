use defmt::{debug, trace, Format};
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    usart::{BasicInstance, Uart},
};
use embassy_time::{Duration, Timer};

use super::packet::{InstructionPacketKind, ParsingError, StatusPacket};

const MAX_BUFFER_LENGTH: usize = 512;
//Seems ok at 115200 with LOG=Info
// const UART_SLEEP_US_DIRLOW: u64 = 200;
// const UART_SLEEP_US_DIRHIGH: u64 = 300;

// 2Mbauds
const UART_SLEEP_US_DIRLOW: u64 = 100;
const UART_SLEEP_US_DIRHIGH: u64 = 150;

const MAX_READ_BUFFER_LENGTH: usize = 32;

#[derive(PartialEq)]
enum RxState {
    RxUnknown,
    RxReadingHeader1,
    RxReadingHeader2,
    RxReadingHeader3,
    RxReadingInstructionPacket,
    RxMessageReady,
}

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

            if n < MAX_READ_BUFFER_LENGTH {
                //FIXME continue until full packet... => parse packet
                break;
            }

            // if total>=4{
            // 	if self.read_buffer[0] == 0xff && self.read_buffer[1] == 0xff// && self.read_buffer[2]== self.id
            // 	{
            // 		let mut length = self.read_buffer[3]as usize;
            // 		length+=4;
            // 		if total>=length{ //FIXME might have more than one packet in the buffer...
            // 		    total=length; //cut here
            // 		    break;
            // 		}
            // 		else {
            // 		    continue;
            // 		}
            // 	    }
            // 	else{
            // 	    break;
            // 	}

            // 	}
            // else {
            // 	continue;
            // }
            // //TODO check that we dont read more than the sier of the buffer
        }

        debug!("Read {} bytes", total);
        trace!("Read {:#x}", &self.read_buffer[..total]);

        InstructionPacketKind::parse(&self.read_buffer[..total], self.id)
            .map_err(CommunicationError::DynamixelParsingError)
    }

    //Very bad test code with blocking u8 read
    pub async fn read_u8(&mut self) -> Result<InstructionPacketKind, CommunicationError> {
        // We should always be in read mode when this method is called

        let mut total = 0;
        let mut buf_idx: usize = 0;
        let mut rx_state = RxState::RxUnknown;
        let mut read_byte: u8 = 0;
        let mut nb_to_read: u8 = 0;
        loop {
            let ret = match self.usart.nb_read() {
                Ok(c) => read_byte = c,
                Err(e) => {
                    Timer::after(Duration::from_micros(1)).await;
                    continue;
                }
            };

            if (rx_state == RxState::RxUnknown) {
                // We are still waiting for the begining of the header (0xff)
                if (read_byte != 0xff) {
                    // reset_rx_buff();
                    total = 0;
                    buf_idx = 0;
                    nb_to_read = 0;
                    continue;
                } else {
                    rx_state = RxState::RxReadingHeader1;
                    // Wait for the rest of the header
                    self.read_buffer[buf_idx] = read_byte;
                    buf_idx += 1;
                    total += 1;

                    continue;
                }
            } else if (rx_state == RxState::RxReadingHeader1) {
                if (read_byte != 0xff) {
                    rx_state = RxState::RxUnknown;
                    // reset_rx_buff();
                    total = 0;
                    buf_idx = 0;
                    nb_to_read = 0;
                    continue;
                } else {
                    rx_state = RxState::RxReadingHeader2;
                    // Wait for the rest of the header
                    self.read_buffer[buf_idx] = read_byte;
                    buf_idx += 1;
                    total += 1;
                    continue;
                }
            } else if (rx_state == RxState::RxReadingHeader2) {
                rx_state = RxState::RxReadingHeader3;
                // Wait for the rest of the header
                self.read_buffer[buf_idx] = read_byte;
                buf_idx += 1;
                total += 1;

                continue;
            } else if (rx_state == RxState::RxReadingHeader3)
            //Here we should have read the complete header 0xff 0xff id length
            {
                rx_state = RxState::RxReadingInstructionPacket;
                // Wait for the rest of the header
                self.read_buffer[buf_idx] = read_byte;
                buf_idx += 1;
                total += 1;
                nb_to_read = read_byte;

                continue;
            } else if (rx_state == RxState::RxReadingInstructionPacket) {
                self.read_buffer[buf_idx] = read_byte;
                buf_idx += 1;
                total += 1;

                nb_to_read -= 1;
                if (nb_to_read == 0) {
                    rx_state = RxState::RxMessageReady;

                    debug!("Read {} bytes", total);
                    trace!("Read {:#x}", &self.read_buffer[..total]);

                    return InstructionPacketKind::parse(&self.read_buffer[..total], self.id)
                        .map_err(CommunicationError::DynamixelParsingError);
                }
            } else if (rx_state == RxState::RxMessageReady)
            //should not happen
            {
                return Err(CommunicationError::DynamixelParsingError(
                    ParsingError::InvalidPacket,
                ));
            }
        }
    }

    //Very bad test code with blocking u8 read
    pub async fn read_u8buf(&mut self) -> Result<InstructionPacketKind, CommunicationError> {
        // We should always be in read mode when this method is called

        let mut total = 0;
        let mut buf_idx: usize = 0;
        let mut rx_state = RxState::RxUnknown;
        let mut read_byte: u8 = 0;
        let mut nb_to_read: usize = 1;
        loop {
            let ret = match self
                .usart
                .read(&mut self.read_buffer[buf_idx..total + nb_to_read])
                .await
            {
                Ok(()) => {}
                Err(e) => {
                    Timer::after(Duration::from_micros(10)).await;
                    continue;
                }
            };

            if (rx_state == RxState::RxUnknown) {
                // We are still waiting for the begining of the header (0xff)
                if (self.read_buffer[buf_idx] != 0xff) {
                    // reset_rx_buff();
                    total = 0;
                    buf_idx = 0;
                    nb_to_read = 1;
                    continue;
                } else {
                    rx_state = RxState::RxReadingHeader1;
                    // Wait for the rest of the header
                    // self.read_buffer[buf_idx]=read_byte;
                    buf_idx += 1;
                    total += 1;
                    nb_to_read = 1;
                    continue;
                }
            } else if (rx_state == RxState::RxReadingHeader1) {
                if (self.read_buffer[buf_idx] != 0xff) {
                    rx_state = RxState::RxUnknown;
                    // reset_rx_buff();
                    total = 0;
                    buf_idx = 0;
                    nb_to_read = 1;
                    continue;
                } else {
                    rx_state = RxState::RxReadingHeader2;
                    // Wait for the rest of the header
                    // self.read_buffer[buf_idx]=read_byte;
                    buf_idx += 1;
                    total += 1;
                    nb_to_read = 1;
                    continue;
                }
            } else if (rx_state == RxState::RxReadingHeader2)
            //id
            {
                rx_state = RxState::RxReadingHeader3;
                // Wait for the rest of the header
                // self.read_buffer[buf_idx]=read_byte;
                buf_idx += 1;
                total += 1;
                nb_to_read = 1;

                continue;
            } else if (rx_state == RxState::RxReadingHeader3)
            //Here we should have read the complete header 0xff 0xff id length
            {
                rx_state = RxState::RxReadingInstructionPacket;
                // Wait for the rest of the header
                // self.read_buffer[buf_idx]=read_byte;

                total += 1;
                nb_to_read = self.read_buffer[buf_idx] as usize; //WTF why is it 0x00 here ??
                buf_idx += 1;
                total += nb_to_read;
                trace!("length: {} data {}", nb_to_read, self.read_buffer[0..total]);

                continue;
            } else if (rx_state == RxState::RxReadingInstructionPacket) {
                // self.read_buffer[buf_idx]=read_byte;
                // total+=self.read_buffer[buf_idx] as usize;
                // buf_idx+=self.read_buffer[buf_idx] as usize;

                rx_state = RxState::RxMessageReady;

                debug!("Read {} bytes", total);
                trace!("Read {:#x}", &self.read_buffer[..total]);

                return InstructionPacketKind::parse(&self.read_buffer[..total], self.id)
                    .map_err(CommunicationError::DynamixelParsingError);
            } else if (rx_state == RxState::RxMessageReady)
            //should not happen
            {
                return Err(CommunicationError::DynamixelParsingError(
                    ParsingError::InvalidPacket,
                ));
            }
        }
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
