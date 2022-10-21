# `open-cleverpad`

> An open firmware for [MIDIPLUS SmartPAD][smartpad] (being a pretty cheap launchpad-like MIDI-controller)

This project is developed and maintained by [DerFetzer][team].
Feature and pull requests are very welcome!

## Features

* All pads, encoders and the buttons at the bottom generate standard MIDI messages when used.
* Pads support 64 colors instead of only 8 with the stock firmware.
* All LEDs can be fully controlled via MIDI messages.
* Left buttons set the channel for encoder messages.
* Right buttons set the channel for the pads.
* Switch the channels via MIDI messages.

## MIDI Messages
Have a look at the included [Ardour MIDI map][midimap] to get an idea of how it could be used.

* Pads: `note_on/note_off channel=m note=0-64 velocity=127/0` where `m` can be set with the buttons on the right
* Buttons at the bottom: `note_on/note_off channel=8/9 note=0-3 velocity=127/0`
* Encoders: `control_change channel=n control=1-8 value=v time=0` where `n` can be set with the buttons on the left and `v` works according to `enc-l` [here][ardourmidi]
* Set encoder channel: send `control_change, control=110, value=0-7`
* Set pad channel: send `control_change, control=111, value=0-7`
* Set LED color: send `note_on` with the same channel and note as the pad/button generates, `note_off` disables the LED with any values
    * Buttons at the bottom: any `value` greater 0 enables the LED
    * Pads: `value` is interpreted as `0b0bbggrr`

## Installation

* **[Only once]** This firmware is designed to be used with the [HID-bootloader][bootloader].
    * Download the [repository][bootloader].
    * Compile the bootloader by executing `make smartpad` inside `./bootloader/F1` folder or use the precompiled binary from the [bin](bin) folder.
    * **Important: The original firmware will be lost after this step and cannot be restored!**
    * Flash the resulting binary via SWD as described [here][kikpadflash] or with any other method you are comfortable with.
* In order to enter bootloader mode you have to **push and hold the first pad in the third row during power up for about 1 second**.
* Use the precompiled binary from the [bin](bin) folder or run `cargo objcopy --release -- --bin open-cleverpad -O binary open-cleverpad.bin` to generate the binary from source.
* Flash the .bin file using `hid-flash` tool from `./cli` folder with something like the following: `hid-flash open-cleverpad.bin tty0`. Just use any serial device and ignore messages like `error, counldn't open [/dev/tty0]`.

## Developer Documentation

* The used STM32F103RBT6 unfortunately has flash read protection activated and
there is a function that resets the microcontroller when there is activity on the SWD interface.

* There is an update mechanism in the original firmware but I did not look at it for long.
Current updater can be downloaded at [MIDIPLUS homepage][firmware].

* [s0len0id][solenoid] assembled a great documentation of stock firmware function [here][smartpad-tester].

* In order to remove read protection you have to mass erase the device, **so the original firmware will be lost**.
You have to try during startup with something like the following:

``` console
$ openocd -f /usr/share/openocd/scripts/interface/stlink-v1.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg  -c init -c "reset halt" -c "flash banks" -c "stm32f1x mass_erase 0" -c "stm32f1x unlock 0"
```

* For reverse engineered information about the used hardware see [hardware](hardware) folder.

## Dependencies

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
[bootloader]: https://github.com/DerFetzer/STM32_HID_Bootloader
[midimap]: /ardour/midi_maps/Open-CleverPAD.map
[kikpadflash]: https://github.com/TheKikGen/kikpad#how-to-flash
[ardourmidi]: https://manual.ardour.org/using-control-surfaces/generic-midi/working-with-encoders/
