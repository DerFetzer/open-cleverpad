#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]
#![allow(non_snake_case)]

#[allow(unused_imports)]
use panic_semihosting;

use rtfm::Instant;

use asm_delay::AsmDelay;

use stm32f1xx_hal::gpio::gpioa::{PA10, PA9};
use stm32f1xx_hal::gpio::State::High;
use stm32f1xx_hal::gpio::{Output, PushPull};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::prelude::*;

use crate::hardware::{ButtonMatrix, ButtonMatrixPins, EncoderPins, Encoders, LedPins, Leds};
use crate::midi::{ControlChange, MidiMessage, NoteOff, NoteOn};

use stm32_usbd::{UsbBus, UsbBusType};

use usb_device::bus;
use usb_device::prelude::*;

use crate::hal::ButtonEventEdge::{NegEdge, PosEdge};
use crate::hal::{
    ButtonEvent, ButtonEventEdge, ButtonType, Direction, LedColor, LedEvent, LedEventType,
    ParameterType,
};
use core::cmp::min;
use heapless::consts::*;
use heapless::spsc::{Consumer, Producer, Queue};

mod hal;
mod hardware;
mod midi;
mod usb_midi;

// SYSCLK = 72MHz --> clock_period = 13.9ns

const LED_BANK_PERIOD: u32 = 70_000; // ~1kHz
const ENC_CAPTURE_PERIOD: u32 = 14_000; // ~5kHz
const ENC_EVAL_PERIOD: u32 = 1_400_000; // ~50Hz
const BUTTON_COL_PERIOD: u32 = 700_000; // ~.1kHz

const STARTUP_DELAY: u32 = 70_000_000; // ~1Hz

const VID: u16 = 0x1122;
const PID: u16 = 0x3344;

#[rtfm::app(device = stm32f1xx_hal::pac)]
const APP: () = {
    static mut LEDS: Leds = ();
    static mut ENCODERS: Encoders = ();
    static mut BUTTON_MATRIX: ButtonMatrix = ();

    static mut ENCODER_POSITIONS: [i32; 8] = [0; 8];
    static mut PREV_ENCODER_POSITIONS: [i32; 8] = [0; 8];
    static mut ENCODER_PARAMETER_TYPE: ParameterType = ParameterType::Volume;
    static mut PREV_BUTTON_STATE: [u8; 11] = [0; 11];

    static mut USB_DEV: UsbDevice<'static, UsbBusType> = ();
    static mut MIDI: usb_midi::MidiClass<'static, UsbBusType> = ();

    static mut BUTTON_EVENT_P: Producer<'static, ButtonEvent, U4> = ();
    static mut BUTTON_EVENT_C: Consumer<'static, ButtonEvent, U4> = ();

    static mut DEBUG_PIN_PA9: PA9<Output<PushPull>> = ();
    static mut DEBUG_PIN_PA10: PA10<Output<PushPull>> = ();

    #[init(schedule = [led_bank, enc, enc_eval, button])]
    fn init() -> init::LateResources {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;
        static mut BUTTON_QUEUE: Option<Queue<ButtonEvent, U4>> = None;

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        // Acquire the GPIO peripherals
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

        // Declare LED GPIOs
        let led_hs_en_l = gpioa
            .pa2
            .into_push_pull_output_with_state(&mut gpioa.crl, High);
        let led_hs_a0 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let led_hs_a1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let led_hs_a2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

        let led_ls_en_l = gpiob
            .pb12
            .into_push_pull_output_with_state(&mut gpiob.crh, High);
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
        afio.mapr.disable_jtag();

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
                .into_open_drain_output_with_state(&mut gpiob.crl, High),
            col2: gpiob
                .pb1
                .into_open_drain_output_with_state(&mut gpiob.crl, High),
            col3: gpiob
                .pb2
                .into_open_drain_output_with_state(&mut gpiob.crl, High),
            col4: gpiob
                .pb3
                .into_open_drain_output_with_state(&mut gpiob.crl, High),
            col5: gpiob
                .pb4
                .into_open_drain_output_with_state(&mut gpiob.crl, High),
            col6: gpiob
                .pb5
                .into_open_drain_output_with_state(&mut gpiob.crl, High),
            col7: gpiob
                .pb6
                .into_open_drain_output_with_state(&mut gpiob.crl, High),
            col8: gpiob
                .pb7
                .into_open_drain_output_with_state(&mut gpiob.crl, High),
            col9: gpiob
                .pb8
                .into_open_drain_output_with_state(&mut gpiob.crh, High),
            col10: gpiob
                .pb9
                .into_open_drain_output_with_state(&mut gpiob.crh, High),
            col11: gpiob
                .pb10
                .into_open_drain_output_with_state(&mut gpiob.crh, High),
        };

        // Declare Debug-GPIOs
        let debug_pa9 = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
        let debug_pa10 = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);

        // Queues
        *BUTTON_QUEUE = Some(Queue::new());
        let (button_event_p, button_event_c) = BUTTON_QUEUE.as_mut().unwrap().split();

        // USB
        let _usb_pullup = gpioa
            .pa8
            .into_push_pull_output_with_state(&mut gpioa.crh, High);

        let usb_dm = gpioa.pa11;
        let usb_dp = gpioa.pa12;

        *USB_BUS = Some(UsbBus::new(device.USB, (usb_dm, usb_dp)));

        let midi = usb_midi::MidiClass::new(USB_BUS.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(VID, PID))
            .manufacturer("derfetzer")
            .product("open-cleverpad")
            .serial_number("12345678")
            .build();

        let button_delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));
        let encoder_delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));

        schedule
            .led_bank(Instant::now() + (STARTUP_DELAY + LED_BANK_PERIOD).cycles())
            .unwrap();
        schedule
            .enc(Instant::now() + (STARTUP_DELAY + ENC_CAPTURE_PERIOD).cycles())
            .unwrap();
        schedule
            .enc_eval(Instant::now() + (STARTUP_DELAY + ENC_EVAL_PERIOD).cycles())
            .unwrap();
        schedule
            .button(Instant::now() + (STARTUP_DELAY + BUTTON_COL_PERIOD).cycles())
            .unwrap();

        init::LateResources {
            LEDS: Leds::new(led_pins),
            ENCODERS: Encoders::new(encoder_pins, encoder_delay),
            BUTTON_MATRIX: ButtonMatrix::new(button_pins, button_delay),
            USB_DEV: usb_dev,
            MIDI: midi,
            BUTTON_EVENT_P: button_event_p,
            BUTTON_EVENT_C: button_event_c,
            DEBUG_PIN_PA9: debug_pa9,
            DEBUG_PIN_PA10: debug_pa10,
        }
    }

    #[idle(resources = [BUTTON_EVENT_C, ENCODER_POSITIONS, ENCODER_PARAMETER_TYPE, LEDS, MIDI, DEBUG_PIN_PA9])]
    fn idle() -> ! {
        let parameter_led_event = LedEvent::new(
            ButtonType::Parameter(ParameterType::Volume),
            LedEventType::Switch(true),
        );

        resources.LEDS.lock(|l| {
            let banks = l.get_banks();
            l.set_banks(parameter_led_event.apply_to_banks(banks));
        });

        let mut master_channel: u8 = 1;
        let mut master_channel_leds = [[0_u32; 8]; 8];

        let master_led_event = LedEvent::new(ButtonType::Master(1), LedEventType::Switch(true));

        resources.LEDS.lock(|l| {
            let banks = l.get_banks();
            l.set_banks(master_led_event.apply_to_banks(banks));
        });

        loop {
            // Handle button events
            if let Some(e) = resources.BUTTON_EVENT_C.dequeue() {
                let on = match e.event {
                    ButtonEventEdge::NegEdge => false,
                    ButtonEventEdge::PosEdge => true,
                };
                match e.btn {
                    // send MIDI
                    ButtonType::Pad { x, y } => {
                        let midi = match on {
                            true => NoteOn::new(master_channel - 1, y * 8 + x, 127)
                                .unwrap()
                                .to_bytes(),
                            false => NoteOff::new(master_channel - 1, y * 8 + x)
                                .unwrap()
                                .to_bytes(),
                        };
                        resources.MIDI.lock(|m| m.enqueue(midi));
                        rtfm::pend(pac::Interrupt::USB_LP_CAN_RX0);
                    }
                    ButtonType::Master(channel) => {
                        if on {
                            // switch MIDI channel for pads
                            resources.LEDS.lock(|l| {
                                for i in 0..6 {
                                    master_channel_leds[master_channel as usize - 1][i] =
                                        l.get_bank_value(i);
                                    l.set_bank_value(
                                        i,
                                        master_channel_leds[channel as usize - 1][i],
                                    );
                                }

                                let master_off_event = LedEvent::new(
                                    ButtonType::Master(master_channel as u8),
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
                            let encoder_parameter_type =
                                resources.ENCODER_PARAMETER_TYPE.lock(|ept| *ept);

                            let parameter_off_event = LedEvent::new(
                                ButtonType::Parameter(encoder_parameter_type),
                                LedEventType::Switch(false),
                            );
                            let parameter_on_event =
                                LedEvent::new(e.btn, LedEventType::Switch(true));

                            resources.ENCODER_PARAMETER_TYPE.lock(|ept| {
                                *ept = param;
                            });

                            resources.LEDS.lock(|l| {
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

                        resources.MIDI.lock(|m| m.enqueue(midi));
                        rtfm::pend(pac::Interrupt::USB_LP_CAN_RX0);

                        let led_event =
                            LedEvent::new(ButtonType::Arrow(dir), LedEventType::Switch(on));

                        resources.LEDS.lock(|l| {
                            let banks = l.get_banks();
                            l.set_banks(led_event.apply_to_banks(banks));
                        });
                    }
                    ButtonType::Mode(mode) => {
                        let midi = match on {
                            true => NoteOn::new(9, mode as u8, 127).unwrap().to_bytes(),
                            false => NoteOff::new(9, mode as u8).unwrap().to_bytes(),
                        };

                        resources.MIDI.lock(|m| m.enqueue(midi));
                        rtfm::pend(pac::Interrupt::USB_LP_CAN_RX0);

                        let led_event =
                            LedEvent::new(ButtonType::Mode(mode), LedEventType::Switch(on));

                        resources.LEDS.lock(|l| {
                            let banks = l.get_banks();
                            l.set_banks(led_event.apply_to_banks(banks));
                        });
                    }
                    b => {
                        let led_event = LedEvent::new(b, LedEventType::Switch(on));

                        resources.LEDS.lock(|l| {
                            let banks = l.get_banks();
                            l.set_banks(led_event.apply_to_banks(banks));
                        });
                    }
                };
            }

            // Handle MIDI messages
            let message = resources.MIDI.lock(|m| m.dequeue());

            if let Some(b) = message {
                let mut led_event = None;
                let mut channel = 0;

                if let Some(note_on) = NoteOn::from_bytes(b) {
                    let x = note_on.note % 8;
                    let y = note_on.note / 8;

                    let btn = ButtonType::Pad { x, y };

                    let rgb = match note_on.velocity {
                        127 => {
                            // show preset color depending on y, for Ardour compatibility
                            match y {
                                0 => LedColor::Green,
                                1 => LedColor::Yellow,
                                2 => LedColor::Red,
                                3 => LedColor::Purple,
                                4 => LedColor::Aqua,
                                5 => LedColor::Blue,
                                6 | 7 => LedColor::White,
                                _ => panic!("This should never happen!"),
                            }
                        }
                        num => LedColor::from_value(num),
                    };

                    if x < 8 && y < 8 {
                        led_event = Some(LedEvent::new(btn, LedEventType::SwitchColor(rgb)));
                        channel = note_on.channel + 1;
                    }
                };

                if let Some(note_off) = NoteOff::from_bytes(b) {
                    let x = note_off.note % 8;
                    let y = note_off.note / 8;

                    let btn = ButtonType::Pad { x, y };

                    if x < 8 && y < 8 {
                        led_event = Some(LedEvent::new(
                            btn,
                            LedEventType::SwitchColor(LedColor::Black),
                        ));
                        channel = note_off.channel + 1;
                    }
                };

                if let Some(le) = led_event {
                    if channel == master_channel {
                        resources.LEDS.lock(|l| {
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

    #[task(priority = 3, schedule = [led_bank], resources = [LEDS])]
    fn led_bank() {
        resources.LEDS.write_next_bank();

        schedule
            .led_bank(scheduled + LED_BANK_PERIOD.cycles())
            .unwrap();
    }

    #[task(priority = 2, schedule = [enc], resources = [ENCODERS, ENCODER_POSITIONS, DEBUG_PIN_PA10])]
    fn enc() {
        resources.DEBUG_PIN_PA10.set_high();

        let change = resources.ENCODERS.read();

        if change {
            *resources.ENCODER_POSITIONS = resources.ENCODERS.get_positions();
        }

        schedule
            .enc(scheduled + ENC_CAPTURE_PERIOD.cycles())
            .unwrap();

        resources.DEBUG_PIN_PA10.set_low();
    }

    #[task(schedule = [enc_eval], resources = [ENCODER_POSITIONS, PREV_ENCODER_POSITIONS, ENCODER_PARAMETER_TYPE, MIDI])]
    fn enc_eval() {
        let new_encoder_positions = resources.ENCODER_POSITIONS.lock(|&mut p| p);
        let encoder_positions = *resources.PREV_ENCODER_POSITIONS;

        // Handle encoder changes
        if new_encoder_positions != encoder_positions {
            for i in 0..8 {
                let diff: i32 = new_encoder_positions[i] - encoder_positions[i];
                if diff != 0 {
                    let offset: u8 = match diff.abs() {
                        1 => 1,
                        num => min(num * 10, 63) as u8,
                    };

                    let value = if diff.signum() == -1 {
                        // most significant bit => direction
                        offset | (1 << 6)
                    } else {
                        offset
                    };

                    let midi = ControlChange::new(
                        *resources.ENCODER_PARAMETER_TYPE as u8,
                        i as u8 + 1,
                        value,
                    )
                    .unwrap()
                    .to_bytes();
                    resources.MIDI.lock(|m| m.enqueue(midi));
                    rtfm::pend(pac::Interrupt::USB_LP_CAN_RX0);
                }
            }
            *resources.PREV_ENCODER_POSITIONS = new_encoder_positions;
        }

        schedule
            .enc_eval(scheduled + ENC_EVAL_PERIOD.cycles())
            .unwrap();
    }

    #[task(priority = 2, schedule = [button], resources = [BUTTON_MATRIX, PREV_BUTTON_STATE, BUTTON_EVENT_P])]
    fn button() {
        resources.BUTTON_MATRIX.read();

        let deb_rows = resources.BUTTON_MATRIX.get_debounced_rows();

        if deb_rows != *resources.PREV_BUTTON_STATE {
            for col in 0..11 {
                for row in 0..8 {
                    let edge = match (
                        ((resources.PREV_BUTTON_STATE[col] >> row) & 1),
                        ((deb_rows[col] >> row) & 1),
                    ) {
                        (0, 1) => Some(PosEdge),
                        (1, 0) => Some(NegEdge),
                        _ => None,
                    };

                    if let Some(e) = edge {
                        resources
                            .BUTTON_EVENT_P
                            .enqueue(ButtonEvent::new(row, col as u8, e));
                    }
                }
            }

            *resources.PREV_BUTTON_STATE = deb_rows;
        }

        schedule
            .button(scheduled + BUTTON_COL_PERIOD.cycles())
            .unwrap();
    }

    #[interrupt(resources = [USB_DEV, MIDI])]
    fn USB_HP_CAN_TX() {
        usb_poll(&mut resources.USB_DEV, &mut resources.MIDI);
    }

    #[interrupt(resources = [USB_DEV, MIDI])]
    fn USB_LP_CAN_RX0() {
        usb_poll(&mut resources.USB_DEV, &mut resources.MIDI);
    }

    extern "C" {
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
    }
};

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    midi: &mut usb_midi::MidiClass<'static, B>,
) -> bool {
    if !midi.write_queue_is_empty() {
        midi.write_queue_to_host();
    }

    if !usb_dev.poll(&mut [midi]) {
        return false;
    }

    match midi.read_to_queue() {
        Ok(len) if len > 0 => true,
        _ => false,
    }
}
