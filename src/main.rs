//#![deny(unsafe_code)]
#![no_main]
#![no_std]
// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::sync::Arc;
use core::panic::PanicInfo;

use arrform::{arrform, ArrForm};
use cortex_m::asm;
use cortex_m_rt::exception;
use cortex_m_rt::{entry, ExceptionFrame};
use freertos_rust::*;
use shared_bus;
use shared_pin::SharedPin;
use shift_register_driver::sipo::ShiftRegister;
use stm32f4xx_hal::adc::config::{AdcConfig, Dma, SampleTime, Scan, Sequence};
use stm32f4xx_hal::adc::{Adc, Temperature};
use stm32f4xx_hal::dma::config::DmaConfig;
use stm32f4xx_hal::dma::{StreamsTuple, Transfer};
use stm32f4xx_hal::gpio::{
    PA10, PA2, PA3, PA5, PA6, PA7, PA9, PB13, PB14, PB15, PC10, PC11, PC12, PD8, PD9,
};
use stm32f4xx_hal::otg_fs::USB;
use stm32f4xx_hal::pac::{Peripherals, SPI1, SPI2, SPI3, USART1, USART2, USART3};
use stm32f4xx_hal::serial::config::StopBits;
use stm32f4xx_hal::serial::{Config, Serial};
use stm32f4xx_hal::spi::{Mode as spiMode, Phase, Polarity, Spi};
use stm32f4xx_hal::{
    gpio::Edge,
    pac::{self, Interrupt},
    prelude::*,
};
use stm32f4xx_hal::{
    gpio::{PB6, PB7},
    i2c::{I2c, Mode as i2cMode},
    pac::I2C1,
};

use crate::devices::led::LED;
use crate::usb::{usb_init, usb_println, usb_read};

mod devices;
mod intrpt;
mod usb;

static mut SHUTDOWN: bool = false;

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

fn check_shutdown() -> bool {
    cortex_m::interrupt::free(|_| unsafe { SHUTDOWN })
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .hclk(48.MHz())
        .require_pll48clk()
        .pclk1(24.MHz())
        .pclk2(24.MHz())
        .freeze();

    let mut tick_timer = dp.TIM5.counter_ms(&clocks);
    tick_timer.start(2_678_400_000.millis()).unwrap(); // set the timeout to 31 days

    let mut delay = dp.TIM1.delay_us(&clocks);
    delay.delay(100.millis()); // apparently required for USB to set up properly...

    let mut tick_timer_adc = dp.TIM3.counter_ms(&clocks);
    tick_timer_adc.start(50000.millis()).unwrap(); // set the timeout to 50 s

    // initialize ports
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();

    // initialize DMA
    let dma_streams_2 = StreamsTuple::new(dp.DMA2);
    let dma_streams_1 = StreamsTuple::new(dp.DMA1);

    // initialize pins
    let mut _uart3_rede = gpiod.pd12.into_push_pull_output();

    // initialize leds
    let mut shared_fault_led = SharedPin::new(gpioe.pe4.into_push_pull_output());
    let mut stat_led = LED::new(gpioe.pe2.into_push_pull_output());
    let mut fault_1_led = LED::new(gpioe.pe3.into_push_pull_output());
    // let mut fault_2_led = LED::new(gpioe.pe4.into_push_pull_output());
    let mut fault_2_led = LED::new(shared_fault_led.clone());

    stat_led.on();
    fault_1_led.on();
    fault_2_led.on();

    // initialize switch
    let mut sw = gpioe.pe0.into_floating_input();
    let mut syscfg = dp.SYSCFG.constrain();
    sw.make_interrupt_source(&mut syscfg);
    sw.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    sw.enable_interrupt(&mut dp.EXTI);

    // initialize usb
    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    delay.delay(100.millis());
    unsafe {
        usb_init(usb);
        cp.NVIC
            .set_priority(stm32f4xx_hal::interrupt::DMA2_STREAM0, 1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::OTG_FS);
        cortex_m::peripheral::NVIC::unmask(sw.interrupt());
        cortex_m::peripheral::NVIC::unmask(stm32f4xx_hal::interrupt::DMA2_STREAM0)
    }

    // initialize UART3 for RS485
    let rx_3 = gpiod.pd9;
    let tx_3 = gpiod.pd8;
    let mut uart3: Serial<USART3, (PD8, PD9)> = Serial::new(
        dp.USART3,
        (tx_3, rx_3),
        Config::default()
            .baudrate(9600.bps())
            .parity_none()
            .stopbits(StopBits::STOP1),
        &clocks,
    )
        .unwrap();

    // let uart_buffer = cortex_m::singleton!(: [u8; 2] = [0; 2]).unwrap();
    // let mut transfer_uart3 = Transfer::init_memory_to_peripheral(
    //     dma_streams_1.4,
    //     uart3_tx,
    //     uart_buffer,
    //     None,
    //     DmaConfig::default()
    //         .memory_increment(true)
    //         .fifo_enable(true)
    //         .fifo_error_interrupt(true)
    //         .transfer_complete_interrupt(true),
    // );



    // let cryo_data_container =
    //     Arc::new(Mutex::new(cryo_data).expect("Failed to create data guard mutex"));
    // let cryo_data_container_main = cryo_data_container.clone();
    // let cryo_data_container_usb = cryo_data_container.clone();
    // let cryo_data_container_cryo = cryo_data_container.clone();
    //
    // let heater_command_queue = Arc::new(Queue::new(10).unwrap());
    // let heater_command_queue_main = heater_command_queue.clone();
    // let mut heater_command_queue_usb = heater_command_queue.clone();

    delay.delay(1000.millis());
    usb_println("boot up ok");
    fault_1_led.on();
    delay.delay(100.millis());

    Task::new()
        .name("CRYO TASK")
        .stack_size(512)
        .priority(TaskPriority(3))
        .start(move || {
            CurrentTask::delay(Duration::ms(5000));

            // initialize TMC5160 stepper driver
            usb_println("setting up cryo");

            // let mut turbo = TC400::new(&mut uart3);

            loop {


                CurrentTask::delay(Duration::ms(2000));
            }
        })
        .unwrap();



    Task::new()
        .name("USB TASK")
        .stack_size(2048)
        .priority(TaskPriority(3))
        .start(move || {
            let mut hk_period = 500.0;
            let mut hk = true;

            loop {
                if check_shutdown() {
                    usb_println("a panic occurred... stopping all threads...");
                    CurrentTask::delay(Duration::ms(1000));
                    break;
                }

                usb_println("running USB task...");

                // sample frequency
                CurrentTask::delay(Duration::ms(hk_period as u32 / 2));
                stat_led.toggle();
                CurrentTask::delay(Duration::ms(hk_period as u32 / 2));
                stat_led.toggle();
            }
        })
        .unwrap();

    FreeRtosUtils::start_scheduler();
}

#[exception]
#[allow(non_snake_case)]
unsafe fn DefaultHandler(_irqn: i16) {
    // custom default handler
    // irqn is negative for Cortex-M exceptions
    // irqn is positive for device specific (line IRQ)
    // panic!("Exception: {}", irqn);
    // TODO: check if panicking here works

    // Safety: the GPIO peripheral is static, and we're not racing anyone by
    // definition since we're in the process of panicking to a halt.

    // here we stop all other threads
    cortex_m::interrupt::free(|_| unsafe { SHUTDOWN = true });

    let dp = Peripherals::steal();

    // Turn off the Heater, it's connected to pin PC6.
    // initialize ports
    dp.GPIOC.odr.write(|w| w.odr6().clear_bit());
    dp.GPIOE.odr.write(|w| w.odr4().clear_bit());
}

#[exception]
#[allow(non_snake_case)]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    // Safety: the GPIO peripheral is static, and we're not racing anyone by
    // definition since we're in the process of panicking to a halt.
    let dp = Peripherals::steal();

    // Turn off the Heater, it's connected to pin PC6.
    // initialize ports
    dp.GPIOC.odr.write(|w| w.odr6().clear_bit());
    dp.GPIOE.odr.write(|w| w.odr4().clear_bit());

    loop {}
}

#[panic_handler]
unsafe fn custom_panic_handler(_info: &PanicInfo) -> ! {
    // Safety: the GPIO peripheral is static, and we're not racing anyone by
    // definition since we're in the process of panicking to a halt.

    // here we stop all other threads
    cortex_m::interrupt::free(|_| unsafe { SHUTDOWN = true });
    CurrentTask::delay(Duration::ms(100));

    let dp = Peripherals::steal();

    // Turn off the Heater, it's connected to pin PC6.
    // initialize ports
    dp.GPIOC.odr.write(|w| w.odr6().clear_bit());
    dp.GPIOE.odr.write(|w| w.odr4().clear_bit());

    // sets it into debug-mode and sets a breakpoint
    asm::bkpt();

    loop {}
}

// define what happens in an Out Of Memory (OOM) condition
// #[alloc_error_handler]
// unsafe fn alloc_error(_layout: Layout) -> ! {
//     // here we stop all other threads
//     cortex_m::interrupt::free(|_| unsafe { SHUTDOWN = true });
//
//     let dp = Peripherals::steal();
//
//     // Turn off the Heater, it's connected to pin PC6.
//     // initialize ports
//     dp.GPIOC.odr.write(|w| w.odr6().clear_bit());
//     dp.GPIOE.odr.write(|w| w.odr4().clear_bit());
//
//     // sets it into debug-mode and sets a breakpoint
//     asm::bkpt();
//     loop {}
// }

#[no_mangle]
#[allow(non_snake_case, unused_variables)]
unsafe fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    // here we stop all other threads
    cortex_m::interrupt::free(|_| unsafe { SHUTDOWN = true });

    let dp = Peripherals::steal();

    // Turn off the Heater, it's connected to pin PC6.
    // initialize ports
    dp.GPIOC.odr.write(|w| w.odr6().clear_bit());
    dp.GPIOE.odr.write(|w| w.odr4().clear_bit());

    // sets it into debug-mode and sets a breakpoint
    asm::bkpt();
}

#[no_mangle]
#[allow(non_snake_case, unused_variables)]
unsafe fn vApplicationMallocFailedHook() {
    // here we stop all other threads
    cortex_m::interrupt::free(|_| unsafe { SHUTDOWN = true });

    let dp = Peripherals::steal();

    // Turn off the Heater, it's connected to pin PC6.
    // initialize ports
    dp.GPIOC.odr.write(|w| w.odr6().clear_bit());
    dp.GPIOE.odr.write(|w| w.odr4().clear_bit());

    // sets it into debug-mode and sets a breakpoint
    asm::bkpt();
}
