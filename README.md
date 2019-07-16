# **This is pretty much work in progress and not usable at the moment at all!**

# `open-cleverpad`

> An open firmware for [MIDIPLUS SmartPAD][smartpad] (being a pretty cheap launchpad-like MIDI-controller)

This project is developed and maintained by [DerFetzer][team].

## Documentation

The used STM32F103RBT6 unfortunately has flash read protection activated and
there is a function that resets the microcontroller when there is activity on the SWD interface.

There is an update mechanism in the original firmware but I did not look at it for long.
Current updater can be downloaded at [MIDIPLUS homepage][firmware].

[s0len0id][solenoid] assembled a great documentation of stock firmware function [here][smartpad-tester].

In order to remove read protection you have to mass erase the device, **so the original firmware will be lost**.
You have to try during startup with something like the following:

``` console
$ openocd -f /usr/share/openocd/scripts/interface/stlink-v1.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg  -c init -c "reset halt" -c "flash banks" -c "stm32f1x mass_erase 0" -c "stm32f1x unlock 0"
```

For reverse engineered information about the used hardware see [hardware](hardware) folder.

## Dependencies

To build embedded programs using this template you'll need:

- `rust-std` components (pre-compiled `core` crate) for the ARM Cortex-M3
  targets. Run:

``` console
$ rustup target add thumbv7m-none-eabi
```
# License

This crate is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Credits

This crate is based on the [cortex-m-quickstart][template] template.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

## Code of Conduct

Contribution to this crate is organized under the terms of the [Rust Code of
Conduct][CoC], the maintainer of this crate, [DerFetzer][team], promises
to intervene to uphold that code of conduct.

[CoC]: https://www.rust-lang.org/policies/code-of-conduct
[team]: https://github.com/DerFetzer
[template]: https://github.com/rust-embedded/cortex-m-quickstart
[smartpad]: http://www.midiplus.com.tw/smartpad.htm
[firmware]: http://www.midiplus.com.tw/MIDIPLUS-Download.files/Firmware%20update/SmartPAD%20Firmware%20Update%20V0.15%2020171103.zip
[smartpad-tester]: https://github.com/s0len0id/smartpad-tester
[solenoid]: https://github.com/s0len0id
