#![deny(warnings)]
#![no_main]
#![no_std]

use panic_reset as _;

mod hal;
mod hardware;
mod midi;
mod usb_midi;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI1, EXTI2, EXTI3])]
mod app {
    use crate::hal::ButtonEventEdge::{NegEdge, PosEdge};
    use crate::hal::{
        ButtonEvent, ButtonEventEdge, ButtonType, LedColor, LedEvent, LedEventType, ParameterType,
        COLOR_AQUA, COLOR_BLACK, COLOR_BLUE, COLOR_GREEN, COLOR_PURPLE, COLOR_RED, COLOR_WHITE,
        COLOR_YELLOW, DIRECTION_TYPES, MODE_TYPES,
    };
    use crate::hardware::{ButtonMatrix, ButtonMatrixPins, EncoderPins, Encoders, LedPins, Leds};
    use crate::midi::{
        ControlChange, EncoderMode, EncoderParameters, MidiMessage, NoteOff, NoteOn,
    };
    use crate::usb_midi;
    use crate::usb_midi::MidiClass;
    use asm_delay::{bitrate, AsmDelay};
    use dwt_systick_monotonic::DwtSystick;
    use heapless::spsc::{Consumer, Producer, Queue};
    use stm32f1xx_hal::pac::Peripherals;
    use stm32f1xx_hal::{
        // gpio::gpioa::*,
        gpio::*,
        pac,
        prelude::*,
        usb::{Peripheral, UsbBus, UsbBusType},
    };
    use usb_device::bus;
    use usb_device::prelude::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<72_000_000>;

    const LED_BANK_PERIOD: u32 = 1_000;
    const ENC_CAPTURE_PERIOD: u32 = 200; // ~5kHz
    const ENC_EVAL_PERIOD: u32 = 20_000; // ~50Hz
    const BUTTON_COL_PERIOD: u32 = 10_000; // ~.1kHz

    const VID: u16 = 0x1ACC;
    const PID: u16 = 0x3801;

    #[shared]
    struct Shared {
        leds: Leds,
        encoder_positions: [i32; 8],
        encoder_parameter_type: ParameterType,
        encoder_parameters: EncoderParameters,
        button_event_p: Producer<'static, ButtonEvent, 64>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        midi: MidiClass<'static, UsbBusType>,
    }

    #[local]
    struct Local {
        encoders: Encoders,
        button_matrix: ButtonMatrix,
        prev_encoder_positions: [i32; 8],
        prev_button_state: [u8; 11],
        button_event_c: Consumer<'static, ButtonEvent, 64>,
        // debug_pin_pa9: PA9<Output<PushPull>>,
        // debug_pin_pa10: PA10<Output<PushPull>>,
    }

    #[init(local = [BUTTON_QUEUE: Queue<ButtonEvent, 64> = Queue::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

        let dp: Peripherals = cx.device;
        let mut cp: rtic::export::Peripherals = cx.core;

        cp.DWT.enable_cycle_counter();

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let mut afio = dp.AFIO.constrain();

        // Acquire the GPIO peripherals
        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();
        let mut gpioc = dp.GPIOC.split();

        // Declare LED GPIOs
        let led_hs_en_l = gpioa
            .pa2
            .into_push_pull_output_with_state(&mut gpioa.crl, PinState::High);
        let led_hs_a0 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let led_hs_a1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let led_hs_a2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

        let led_ls_en_l = gpiob
            .pb12
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::High);
        let led_ls_dai = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
        let led_ls_dck = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        let led_ls_lat = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);

        let led_pins = LedPins {
            hs_en_l: led_hs_en_l,
            hs_a0: led_hs_a0,
            hs_a1: led_hs_a1,
            hs_a2: led_hs_a2,
            ls_en_l: led_ls_en_l,
            ls_dai: led_ls_dai,
            ls_dck: led_ls_dck,
            ls_lat: led_ls_lat,
        };

        // Declare encoder GPIOs
        let enc_a0 = gpioc.pc0.into_push_pull_output(&mut gpioc.crl);
        let enc_a1 = gpioc.pc1.into_push_pull_output(&mut gpioc.crl);
        let enc_a2 = gpioc.pc2.into_push_pull_output(&mut gpioc.crl);

        let enc_out_a = gpioa.pa1;
        let enc_out_b = gpioa.pa0;

        // QEI does not work because of encoder switching
        let encoder_pins = EncoderPins {
            a0: enc_a0,
            a1: enc_a1,
            a2: enc_a2,
            a: enc_out_a,
            b: enc_out_b,
        };

        // Declare button matrix GPIOs

        // Disable JTAG ports
        let (_, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let button_pins = ButtonMatrixPins {
            row1: gpioc.pc8,
            row2: gpioc.pc9,
            row3: gpioc.pc10,
            row4: gpioc.pc11,
            row5: gpioc.pc12,
            row6: gpioc.pc13,
            row7: gpioc.pc14,
            row8: gpioc.pc15,
            col1: gpiob
                .pb0
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col2: gpiob
                .pb1
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col3: gpiob
                .pb2
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col4: pb3.into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col5: pb4.into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col6: gpiob
                .pb5
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col7: gpiob
                .pb6
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col8: gpiob
                .pb7
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col9: gpiob
                .pb8
                .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High),
            col10: gpiob
                .pb9
                .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High),
            col11: gpiob
                .pb10
                .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High),
        };

        // Declare Debug-GPIOs
        let _debug_pa9 = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
        let _debug_pa10 = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);

        // Queues
        let (button_event_p, button_event_c): (
            Producer<'static, ButtonEvent, 64>,
            Consumer<'static, ButtonEvent, 64>,
        ) = cx.local.BUTTON_QUEUE.split();

        // USB
        let _usb_pullup = gpioa
            .pa8
            .into_push_pull_output_with_state(&mut gpioa.crh, PinState::High);

        let usb_dm = gpioa.pa11;
        let usb_dp = gpioa.pa12;

        let usb = Peripheral {
            usb: dp.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }

        let midi = usb_midi::MidiClass::new(unsafe { USB_BUS.as_ref().unwrap() });

        let usb_dev =
            UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, UsbVidPid(VID, PID))
                .manufacturer("derfetzer")
                .product("open-cleverpad")
                .max_power(500)
                .build();

        let button_delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));
        let encoder_delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));

        let mono = DwtSystick::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.sysclk().to_Hz());

        led_bank::spawn().unwrap();
        enc::spawn().unwrap();
        enc_eval::spawn().unwrap();
        button::spawn().unwrap();

        (
            Shared {
                leds: Leds::new(led_pins),
                encoder_positions: [0; 8],
                encoder_parameter_type: ParameterType::Volume,
                encoder_parameters: EncoderParameters {
                    mode: EncoderMode::EncL,
                    speed_multiplier: 10,
                },
                button_event_p,
                usb_dev,
                midi,
            },
            Local {
                encoders: Encoders::new(encoder_pins, encoder_delay),
                button_matrix: ButtonMatrix::new(button_pins, button_delay),
                prev_encoder_positions: [0; 8],
                prev_button_state: [0; 11],
                button_event_c,
                // debug_pin_pa9: debug_pa9,
                // debug_pin_pa10: debug_pa10,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [button_event_c], shared = [encoder_positions, encoder_parameter_type, encoder_parameters, leds, midi, button_event_p])]
    fn idle(mut cx: idle::Context) -> ! {
        let parameter_led_event = LedEvent::new(
            ButtonType::Parameter(ParameterType::Volume),
            LedEventType::Switch(true),
        );

        cx.shared.leds.lock(|l: &mut Leds| {
            let banks = l.get_banks();
            l.set_banks(parameter_led_event.apply_to_banks(banks));
        });

        let mut master_channel: u8 = 1;
        let mut master_channel_leds = [[[0_u32; 8]; 4]; 8];

        let master_led_event = LedEvent::new(ButtonType::Master(1), LedEventType::Switch(true));

        cx.shared.leds.lock(|l: &mut Leds| {
            let banks = l.get_banks();
            l.set_banks(master_led_event.apply_to_banks(banks));
        });

        loop {
            // Handle button events
            if let Some(e) = cx.local.button_event_c.dequeue() {
                let on = match e.event {
                    ButtonEventEdge::NegEdge => false,
                    ButtonEventEdge::PosEdge => true,
                };
                match e.btn {
                    // send midi
                    ButtonType::Pad { x, y } => {
                        let midi = match on {
                            true => NoteOn::new(master_channel - 1, y * 8 + x, 127)
                                .unwrap()
                                .to_bytes(),
                            false => NoteOff::new(master_channel - 1, y * 8 + x)
                                .unwrap()
                                .to_bytes(),
                        };
                        cx.shared
                            .midi
                            .lock(|m: &mut MidiClass<'static, UsbBusType>| {
                                m.enqueue(midi).unwrap()
                            });
                        rtic::pend(pac::Interrupt::USB_LP_CAN_RX0);
                    }
                    ButtonType::Master(channel) => {
                        if on {
                            // switch midi channel for pads
                            cx.shared.leds.lock(|l: &mut Leds| {
                                for i in 0..6 {
                                    for (j, v) in l.get_bank_value(i).into_iter().enumerate() {
                                        master_channel_leds[master_channel as usize - 1][j][i] = v;
                                    }

                                    l.set_bank_value(
                                        i,
                                        [
                                            master_channel_leds[channel as usize - 1][0][i],
                                            master_channel_leds[channel as usize - 1][1][i],
                                            master_channel_leds[channel as usize - 1][2][i],
                                            master_channel_leds[channel as usize - 1][3][i],
                                        ],
                                    );
                                }

                                let master_off_event = LedEvent::new(
                                    ButtonType::Master(master_channel),
                                    LedEventType::Switch(false),
                                );
                                let master_on_event =
                                    LedEvent::new(e.btn, LedEventType::Switch(true));

                                let mut banks = l.get_banks();
                                banks = master_off_event.apply_to_banks(banks);
                                banks = master_on_event.apply_to_banks(banks);
                                l.set_banks(banks);

                                master_channel = channel;
                            })
                        }
                    }
                    ButtonType::Parameter(param) => {
                        if on {
                            let encoder_parameter_type = cx
                                .shared
                                .encoder_parameter_type
                                .lock(|ept: &mut ParameterType| *ept);

                            let parameter_off_event = LedEvent::new(
                                ButtonType::Parameter(encoder_parameter_type),
                                LedEventType::Switch(false),
                            );
                            let parameter_on_event =
                                LedEvent::new(e.btn, LedEventType::Switch(true));

                            cx.shared
                                .encoder_parameter_type
                                .lock(|ept: &mut ParameterType| {
                                    *ept = param;
                                });

                            cx.shared.leds.lock(|l: &mut Leds| {
                                let mut banks = l.get_banks();
                                banks = parameter_off_event.apply_to_banks(banks);
                                banks = parameter_on_event.apply_to_banks(banks);
                                l.set_banks(banks);
                            });
                        }
                    }
                    ButtonType::Arrow(dir) => {
                        let midi = match on {
                            true => NoteOn::new(8, dir as u8, 127).unwrap().to_bytes(),
                            false => NoteOff::new(8, dir as u8).unwrap().to_bytes(),
                        };

                        cx.shared
                            .midi
                            .lock(|m: &mut MidiClass<'static, UsbBusType>| {
                                m.enqueue(midi).unwrap()
                            });
                        rtic::pend(pac::Interrupt::USB_LP_CAN_RX0);
                    }
                    ButtonType::Mode(mode) => {
                        let midi = match on {
                            true => NoteOn::new(9, mode as u8, 127).unwrap().to_bytes(),
                            false => NoteOff::new(9, mode as u8).unwrap().to_bytes(),
                        };

                        cx.shared
                            .midi
                            .lock(|m: &mut MidiClass<'static, UsbBusType>| {
                                m.enqueue(midi).unwrap()
                            });
                        rtic::pend(pac::Interrupt::USB_LP_CAN_RX0);
                    }
                };
            }

            // Handle midi messages
            let message = cx
                .shared
                .midi
                .lock(|m: &mut MidiClass<'static, UsbBusType>| m.dequeue());

            if let Some(b) = message {
                let mut led_event = None;
                let mut channel = 0;

                if let Some(note_on) = NoteOn::from_bytes(b) {
                    match note_on.channel {
                        0..=7 => {
                            let x = note_on.note % 8;
                            let y = note_on.note / 8;

                            let btn = ButtonType::Pad { x, y };

                            let rgb = match note_on.velocity {
                                127 => {
                                    // show preset color depending on y, for Ardour compatibility
                                    match y {
                                        0 => COLOR_GREEN,
                                        1 => COLOR_YELLOW,
                                        2 => COLOR_RED,
                                        3 => COLOR_PURPLE,
                                        4 => COLOR_AQUA,
                                        5 => COLOR_BLUE,
                                        6 | 7 => COLOR_WHITE,
                                        _ => unreachable!(),
                                    }
                                }
                                num => LedColor::from_value(num),
                            };

                            if x < 8 && y < 8 {
                                led_event =
                                    Some(LedEvent::new(btn, LedEventType::SwitchColor(rgb)));
                                channel = note_on.channel + 1;
                            }
                        }
                        8 if note_on.note < 4 => {
                            channel = master_channel;

                            let btn = ButtonType::Arrow(DIRECTION_TYPES[note_on.note as usize]);

                            led_event = Some(LedEvent::new(
                                btn,
                                LedEventType::Switch(note_on.velocity != 0),
                            ))
                        }
                        9 if note_on.note < 4 => {
                            channel = master_channel;

                            let btn = ButtonType::Mode(MODE_TYPES[note_on.note as usize]);

                            led_event = Some(LedEvent::new(
                                btn,
                                LedEventType::Switch(note_on.velocity != 0),
                            ));
                        }
                        _ => (), // ignore message
                    }
                } else if let Some(note_off) = NoteOff::from_bytes(b) {
                    match note_off.channel {
                        0..=7 => {
                            let x = note_off.note % 8;
                            let y = note_off.note / 8;

                            let btn = ButtonType::Pad { x, y };

                            if x < 8 && y < 8 {
                                led_event = Some(LedEvent::new(
                                    btn,
                                    LedEventType::SwitchColor(COLOR_BLACK),
                                ));
                                channel = note_off.channel + 1;
                            }
                        }
                        8 => {
                            channel = master_channel;

                            let btn = ButtonType::Arrow(DIRECTION_TYPES[note_off.note as usize]);

                            led_event = Some(LedEvent::new(btn, LedEventType::Switch(false)))
                        }
                        9 => {
                            channel = master_channel;

                            let btn = ButtonType::Mode(MODE_TYPES[note_off.note as usize]);

                            led_event = Some(LedEvent::new(btn, LedEventType::Switch(false)));
                        }
                        _ => (), // ignore message
                    }
                } else if let Some(cc) = ControlChange::from_bytes(b) {
                    if cc.controller == 110 && cc.value < 8 {
                        cx.shared.button_event_p.lock(
                            |bep: &mut Producer<'static, ButtonEvent, 64>| {
                                bep.enqueue(ButtonEvent {
                                    btn: ButtonType::Parameter(
                                        ParameterType::try_from(cc.value).unwrap(),
                                    ),
                                    event: ButtonEventEdge::PosEdge,
                                })
                                .unwrap()
                            },
                        )
                    } else if cc.controller == 111 && cc.value < 8 {
                        cx.shared.button_event_p.lock(
                            |bep: &mut Producer<'static, ButtonEvent, 64>| {
                                bep.enqueue(ButtonEvent {
                                    btn: ButtonType::Master(cc.value + 1),
                                    event: ButtonEventEdge::PosEdge,
                                })
                                .unwrap()
                            },
                        )
                    } else if cc.controller == 112 && cc.value < 4 {
                        if let Ok(mode) = EncoderMode::try_from(cc.value) {
                            cx.shared
                                .encoder_parameters
                                .lock(|m: &mut EncoderParameters| {
                                    *m = EncoderParameters { mode, ..*m }
                                });
                        }
                    } else if cc.controller == 113 && cc.value > 0 && cc.value <= 33 {
                        cx.shared
                            .encoder_parameters
                            .lock(|m: &mut EncoderParameters| {
                                *m = EncoderParameters {
                                    speed_multiplier: cc.value,
                                    ..*m
                                }
                            });
                    }
                }

                if let Some(le) = led_event {
                    if channel == master_channel {
                        cx.shared.leds.lock(|l: &mut Leds| {
                            let banks = l.get_banks();
                            l.set_banks(le.apply_to_banks(banks));
                        });
                    } else {
                        master_channel_leds[channel as usize - 1] =
                            le.apply_to_banks(master_channel_leds[channel as usize - 1])
                    }
                }
            }
        }
    }

    #[task(priority = 3, shared = [leds])]
    fn led_bank(mut cx: led_bank::Context) {
        let current_iteration: usize = cx
            .shared
            .leds
            .lock(|leds: &mut Leds| leds.write_next_bank());

        // Gamma correction (~2.8)
        let delay = match current_iteration {
            0 => LED_BANK_PERIOD.micros() / 50,
            1 => LED_BANK_PERIOD.micros() / 50 * 6,
            2 => LED_BANK_PERIOD.micros() / 50 * 15,
            3 => LED_BANK_PERIOD.micros() / 50 * 28,
            _ => unreachable!(),
        };

        led_bank::spawn_after(delay).unwrap();
    }

    #[task(priority = 2, local = [encoders], shared = [encoder_positions])]
    fn enc(mut cx: enc::Context) {
        let change: bool = cx.local.encoders.read();

        if change {
            cx.shared
                .encoder_positions
                .lock(|pos: &mut [i32; 8]| *pos = cx.local.encoders.get_positions());
        }

        enc::spawn_after(ENC_CAPTURE_PERIOD.micros()).unwrap();
    }

    #[task(local = [prev_encoder_positions], shared = [encoder_positions, encoder_parameter_type, encoder_parameters, midi])]
    fn enc_eval(mut cx: enc_eval::Context) {
        let new_encoder_positions = cx.shared.encoder_positions.lock(|&mut p: &mut [i32; 8]| p);
        let encoder_positions: [i32; 8] = *cx.local.prev_encoder_positions;

        // Handle encoder changes
        if new_encoder_positions != encoder_positions {
            for i in 0..8 {
                let diff: i32 = new_encoder_positions[i] - encoder_positions[i];
                if diff != 0 {
                    let midi = ControlChange::new(
                        cx.shared
                            .encoder_parameter_type
                            .lock(|t: &mut ParameterType| *t as u8),
                        i as u8 + 1,
                        cx.shared
                            .encoder_parameters
                            .lock(|m: &mut EncoderParameters| m.diff_to_value(diff)),
                    )
                    .unwrap()
                    .to_bytes();
                    cx.shared
                        .midi
                        .lock(|m: &mut MidiClass<'static, UsbBusType>| m.enqueue(midi).unwrap());
                    rtic::pend(pac::Interrupt::USB_LP_CAN_RX0);
                }
            }
            *cx.local.prev_encoder_positions = new_encoder_positions;
        }

        enc_eval::spawn_after(ENC_EVAL_PERIOD.micros()).unwrap();
    }

    #[task(priority = 2, local = [button_matrix, prev_button_state], shared = [button_event_p])]
    fn button(mut cx: button::Context) {
        cx.local.button_matrix.read();

        let deb_rows: [u8; 11] = cx.local.button_matrix.get_debounced_rows();

        if deb_rows != *cx.local.prev_button_state {
            for (col, deb_row) in deb_rows.iter().enumerate() {
                for row in 0..8 {
                    let edge = match (
                        ((cx.local.prev_button_state[col] >> row) & 1),
                        ((deb_row >> row) & 1),
                    ) {
                        (0, 1) => Some(PosEdge),
                        (1, 0) => Some(NegEdge),
                        _ => None,
                    };

                    if let Some(e) = edge {
                        cx.shared.button_event_p.lock(
                            |bep: &mut Producer<'static, ButtonEvent, 64>| {
                                bep.enqueue(ButtonEvent::new(row, col as u8, e)).unwrap()
                            },
                        )
                    }
                }
            }

            *cx.local.prev_button_state = deb_rows;
        }

        button::spawn_after(BUTTON_COL_PERIOD.micros()).unwrap();
    }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, midi])]
    fn usb_hp_can_tx(cx: usb_hp_can_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut midi = cx.shared.midi;

        (&mut usb_dev, &mut midi).lock(
            |usb_dev: &mut UsbDevice<'static, UsbBusType>,
             midi: &mut MidiClass<'static, UsbBusType>| {
                usb_poll(usb_dev, midi);
            },
        );
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, midi])]
    fn usb_lp_can_rx0(cx: usb_lp_can_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut midi = cx.shared.midi;

        (&mut usb_dev, &mut midi).lock(
            |usb_dev: &mut UsbDevice<'static, UsbBusType>,
             midi: &mut MidiClass<'static, UsbBusType>| {
                usb_poll(usb_dev, midi);
            },
        );
    }

    fn usb_poll<B: bus::UsbBus>(
        usb_dev: &mut UsbDevice<'static, B>,
        midi: &mut usb_midi::MidiClass<'static, B>,
    ) -> bool {
        if !midi.write_queue_is_empty() {
            match midi.write_queue_to_host() {
                Ok(_) => {}
                Err(UsbError::WouldBlock) => {} // If nothing is connected
                Err(_) => panic!(),
            }
        }

        if !usb_dev.poll(&mut [midi]) {
            return false;
        }

        matches!(midi.read_to_queue(), Ok(len) if len > 0)
    }
}
