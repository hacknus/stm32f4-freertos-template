
use stm32f4xx_hal::{
    dma::{config::DmaConfig, PeripheralToMemory, Stream2, StreamsTuple, Transfer},
    pac::{DMA1, USART3},
    prelude::*,
    rcc::RccExt,
    serial,
};
pub const BUFFER_SIZE: usize = 100;

