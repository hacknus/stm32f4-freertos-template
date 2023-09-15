### STM32f4 freeRTOS Rust templae

Connect the board using an ST-Link V3 (with TagConnect) to a USB port on the computer. Be sure to power the board with
an
additional USB-C connector. (ST-Link does not provide power)  
First start the openocd server in a terminal/console window:
```openocd -f interface/stlink.cfg -f target/stm32f4x.cfg```   
Flash/Run the code on the hardware using:  
```cargo +nightly run --package stm32f4-freertos-template --bin stm32f4-freertos-template --target thumbv7em-none-eabihf --release```

To set up your system, be sure to
follow [this guide](https://docs.rust-embedded.org/discovery/f3discovery/03-setup/index.html)!  
Don't forget to install the [ARM GNU toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).  
Make sure you add did
```rustup install nightly``` and
```rustup target add thumbv7em-none-eabihf --toolchain nightly``` to add the target to the nightly toolchain.  
