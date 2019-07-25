#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]
#![allow(non_snake_case)]

#[allow(unused_imports)]
use panic_semihosting;

use cortex_m::asm::{wfi, delay};

use cortex_m_semihosting::hprintln;

use rtfm::Instant;

use asm_delay::AsmDelay;

use stm32f1xx_hal::gpio::State::High;

use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::qei::Qei;

use crate::hardware::{ButtonMatrix, ButtonMatrixPins, EncoderPins, Encoders, LedPins, Leds};
use crate::midi::{MidiMessage, NoteOn};

use stm32_usbd::{UsbBus, UsbBusType};

use usb_device::bus;
use usb_device::prelude::*;

use crate::hal::ButtonEventEdge::{NegEdge, PosEdge};
use crate::hal::{ButtonEvent, ButtonEventEdge};
use heapless::consts::*;
use heapless::spsc::{Consumer, Producer, Queue};
use stm32f1xx_hal::gpio::gpioa::{PA10, PA9};
use stm32f1xx_hal::gpio::{Output, PushPull};

mod hal;
mod hardware;
mod midi;
mod usb_midi;

// SYSCLK = 72MHz --> clock_period = 13.9ns

const LED_BANK_PERIOD: u32 = 70_000; // ~1kHz
const ENC_SEL_PERIOD: u32 = 70_000; // ~1kHz
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
    static mut PREV_BUTTON_STATE: [u8; 11] = [0; 11];

    static mut USB_DEV: UsbDevice<'static, UsbBusType> = ();
    static mut MIDI: usb_midi::MidiClass<'static, UsbBusType> = ();

    static mut BUTTON_EVENT_P: Producer<'static, ButtonEvent, U4> = ();
    static mut BUTTON_EVENT_C: Consumer<'static, ButtonEvent, U4> = ();

    static mut DEBUG_PIN_PA9: PA9<Output<PushPull>> = ();
    static mut DEBUG_PIN_PA10: PA10<Output<PushPull>> = ();

    #[init(schedule = [led_bank, enc, button])]
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
            .enc(Instant::now() + (STARTUP_DELAY + ENC_SEL_PERIOD).cycles())
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

    #[idle(resources = [BUTTON_EVENT_C, ENCODER_POSITIONS])]
    fn idle() -> ! {
        let mut encoder_positions = [0_i32; 8];

        loop {
            if let Some(e) = resources.BUTTON_EVENT_C.dequeue() {
                hprintln!("{:?}", e).unwrap();
            }
            resources.ENCODER_POSITIONS.lock(|&mut p| {
                if p != encoder_positions {
                    encoder_positions = p;
                    hprintln!("{:?}", p).unwrap();
                }
            });

            delay(10_000_000);
        }
    }

    #[task(priority = 3, schedule = [led_bank], resources = [LEDS])]
    fn led_bank() {
        resources.LEDS.write_next_bank();

        schedule
            .led_bank(scheduled + LED_BANK_PERIOD.cycles())
            .unwrap();
    }

    #[task(priority = 2, schedule = [enc], spawn = [activate_debug_leds], resources = [ENCODERS, ENCODER_POSITIONS])]
    fn enc() {
        let change = resources.ENCODERS.read();

        if change {
            *resources.ENCODER_POSITIONS = resources.ENCODERS.get_positions();
        }

        schedule.enc(scheduled + ENC_SEL_PERIOD.cycles()).unwrap();
    }

    #[task(priority = 2, schedule = [button], resources = [BUTTON_MATRIX, PREV_BUTTON_STATE, BUTTON_EVENT_P, DEBUG_PIN_PA9])]
    fn button() {
        resources.DEBUG_PIN_PA9.set_high();

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

        resources.DEBUG_PIN_PA9.set_low();
    }

    #[task(resources = [LEDS, MIDI])]
    fn activate_debug_leds() {
        resources.LEDS.lock(|l| l.set_bank_value(0, 0xFFFFFFFF));
        resources.MIDI.lock(|m| {
            m.enqueue(NoteOn::new(0, 0x37, 0x47).unwrap().to_bytes());
            match m.dequeue() {
                Some(message) => {
                    if let Some(note_on) = NoteOn::from_bytes(message) {
                        //hprintln!("{:?}", note_on).unwrap()
                    }
                }
                _ => (),
            }
        });
        let mut delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));
        delay.delay_ms(200_u32);
        resources.LEDS.lock(|l| l.set_bank_value(0, 0x0));
    }

    #[interrupt(resources = [USB_DEV, MIDI], spawn = [activate_debug_leds])]
    fn USB_HP_CAN_TX() {
        if usb_poll(&mut resources.USB_DEV, &mut resources.MIDI) {
            spawn.activate_debug_leds();
        };
    }

    #[interrupt(resources = [USB_DEV, MIDI], spawn = [activate_debug_leds])]
    fn USB_LP_CAN_RX0() {
        if usb_poll(&mut resources.USB_DEV, &mut resources.MIDI) {
            spawn.activate_debug_leds();
        };
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
    if !usb_dev.poll(&mut [midi]) {
        return false;
    }

    if !midi.write_queue_is_empty() {
        midi.write_queue_to_host();
    }

    match midi.read_to_queue() {
        Ok(len) if len > 0 => true,
        _ => false,
    }
}
