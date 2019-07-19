#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]
#![allow(non_snake_case)]

use panic_semihosting;

use cortex_m::asm::{delay, wfi};

/*#[cfg(debug_assertions)]
use cortex_m_semihosting::hprintln;*/

use rtfm::Instant;

use asm_delay::AsmDelay;

use stm32f1xx_hal::device::TIM2;
use stm32f1xx_hal::gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5};
use stm32f1xx_hal::gpio::gpiob::{
    PB0, PB1, PB10, PB12, PB13, PB14, PB15, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9,
};
use stm32f1xx_hal::gpio::gpioc::{PC0, PC1, PC10, PC11, PC12, PC13, PC14, PC15, PC2, PC8, PC9};
use stm32f1xx_hal::gpio::State::High;
use stm32f1xx_hal::gpio::{Floating, Input, OpenDrain, Output, PushPull};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::qei::Qei;

use stm32_usbd::{UsbBus, UsbBusType};
use usb_device::bus;
use usb_device::prelude::*;

mod usb_midi;

// SYSCLK = 72MHz --> clock_period = 13.9ns

#[cfg(debug_assertions)]
const LED_BANK_PERIOD: u32 = 700_000; // ~.1kHz
#[cfg(debug_assertions)]
const ENC_SEL_PERIOD: u32 = 70_000; // ~1kHz
#[cfg(debug_assertions)]
const BUTTON_COL_PERIOD: u32 = 700_000; // ~.1kHz

#[cfg(not(debug_assertions))]
const LED_BANK_PERIOD: u32 = 70_000; // ~1kHz
#[cfg(not(debug_assertions))]
const ENC_SEL_PERIOD: u32 = 70_000; // ~1kHz
#[cfg(not(debug_assertions))]
const BUTTON_COL_PERIOD: u32 = 700_000; // ~.1kHz

const VID: u16 = 0x1122;
const PID: u16 = 0x3344;

#[rtfm::app(device = stm32f1xx_hal::pac)]
const APP: () = {
    static mut LEDS: Leds = ();
    static mut ENCODERS: Encoders = ();
    static mut BUTTON_MATRIX: ButtonMatrix = ();

    static mut USB_DEV: UsbDevice<'static, UsbBusType> = ();
    static mut MIDI: usb_midi::MidiClass<'static, UsbBusType> = ();

    #[init(schedule = [led_bank, enc, button])]
    fn init() -> init::LateResources {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

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

        // QEI
        let qei = Qei::tim2(
            device.TIM2,
            (enc_out_b, enc_out_a),
            &mut afio.mapr,
            &mut rcc.apb1,
        );

        // QEI does not work because of encoder switching
        let encoder_pins = EncoderPins {
            a0: enc_a0,
            a1: enc_a1,
            a2: enc_a2,
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

        schedule
            .led_bank(Instant::now() + LED_BANK_PERIOD.cycles())
            .unwrap();
        schedule
            .enc(Instant::now() + ENC_SEL_PERIOD.cycles())
            .unwrap();
        schedule
            .button(Instant::now() + BUTTON_COL_PERIOD.cycles())
            .unwrap();

        let button_delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));

        init::LateResources {
            LEDS: Leds::new(led_pins),
            ENCODERS: Encoders::new(qei, encoder_pins),
            BUTTON_MATRIX: ButtonMatrix::new(button_pins, button_delay),
            USB_DEV: usb_dev,
            MIDI: midi,
        }
    }

    #[idle]
    fn idle() -> ! {
        loop {
            wfi();
        }
    }

    #[task(priority = 3, schedule = [led_bank], resources = [LEDS])]
    fn led_bank() {
        resources.LEDS.write_next_bank();

        schedule
            .led_bank(scheduled + LED_BANK_PERIOD.cycles())
            .unwrap();
    }

    #[task(priority = 3, schedule = [enc], spawn = [activate_debug_leds], resources = [ENCODERS])]
    fn enc() {
        if resources.ENCODERS.next_encoder() {
            spawn.activate_debug_leds();
        }

        schedule.enc(scheduled + ENC_SEL_PERIOD.cycles()).unwrap();
    }

    #[task(priority = 2, schedule = [button], resources = [BUTTON_MATRIX])]
    fn button() {
        resources.BUTTON_MATRIX.read();

        schedule
            .button(scheduled + BUTTON_COL_PERIOD.cycles())
            .unwrap();
    }

    #[task(resources = [LEDS])]
    fn activate_debug_leds() {
        resources.LEDS.lock(|l| l.set_bank_value(0, 0xFFFFFFFF));
        let mut delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));
        delay.delay_ms(200_u32);
        resources.LEDS.lock(|l| l.set_bank_value(0, 0x0));
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
) {
    if !usb_dev.poll(&mut [midi]) {
        return;
    }
}

pub struct ButtonMatrix {
    pins: ButtonMatrixPins,
    rows: [u8; 11],
    delay: AsmDelay,
}

impl ButtonMatrix {
    fn new(pins: ButtonMatrixPins, delay: AsmDelay) -> Self {
        ButtonMatrix {
            pins,
            rows: [0; 11],
            delay,
        }
    }

    fn get_rows(self) -> [u8; 11] {
        self.rows
    }

    fn read(&mut self) {
        for i in 0..11 {
            match i {
                0 => self.pins.col1.set_low(),
                1 => self.pins.col2.set_low(),
                2 => self.pins.col3.set_low(),
                3 => self.pins.col4.set_low(),
                4 => self.pins.col5.set_low(),
                5 => self.pins.col6.set_low(),
                6 => self.pins.col7.set_low(),
                7 => self.pins.col8.set_low(),
                8 => self.pins.col9.set_low(),
                9 => self.pins.col10.set_low(),
                10 => self.pins.col11.set_low(),
                _ => panic!("This should never happen"),
            };
            self.delay.delay_us(1_u32);

            let mut row: u8 = 0;
            row |= (self.pins.row1.is_high() as u8) << 0;
            row |= (self.pins.row2.is_high() as u8) << 1;
            row |= (self.pins.row3.is_high() as u8) << 2;
            row |= (self.pins.row4.is_high() as u8) << 3;
            row |= (self.pins.row5.is_high() as u8) << 4;
            row |= (self.pins.row6.is_high() as u8) << 5;
            row |= (self.pins.row7.is_high() as u8) << 6;
            row |= (self.pins.row8.is_high() as u8) << 7;

            self.rows[i] = row;

            match i {
                0 => self.pins.col1.set_high(),
                1 => self.pins.col2.set_high(),
                2 => self.pins.col3.set_high(),
                3 => self.pins.col4.set_high(),
                4 => self.pins.col5.set_high(),
                5 => self.pins.col6.set_high(),
                6 => self.pins.col7.set_high(),
                7 => self.pins.col8.set_high(),
                8 => self.pins.col9.set_high(),
                9 => self.pins.col10.set_high(),
                10 => self.pins.col11.set_high(),
                _ => panic!("This should never happen"),
            };
        }
    }
}

pub struct ButtonMatrixPins {
    row1: PC8<Input<Floating>>,
    row2: PC9<Input<Floating>>,
    row3: PC10<Input<Floating>>,
    row4: PC11<Input<Floating>>,
    row5: PC12<Input<Floating>>,
    row6: PC13<Input<Floating>>,
    row7: PC14<Input<Floating>>,
    row8: PC15<Input<Floating>>,
    col1: PB0<Output<OpenDrain>>,
    col2: PB1<Output<OpenDrain>>,
    col3: PB2<Output<OpenDrain>>,
    col4: PB3<Output<OpenDrain>>,
    col5: PB4<Output<OpenDrain>>,
    col6: PB5<Output<OpenDrain>>,
    col7: PB6<Output<OpenDrain>>,
    col8: PB7<Output<OpenDrain>>,
    col9: PB8<Output<OpenDrain>>,
    col10: PB9<Output<OpenDrain>>,
    col11: PB10<Output<OpenDrain>>,
}

pub struct Encoders {
    qei: Qei<TIM2, (PA0<Input<Floating>>, PA1<Input<Floating>>)>,
    pins: EncoderPins,
    positions: [i16; 8],
    current_encoder: usize,
    last_count: u16,
}

impl Encoders {
    fn new(
        qei: Qei<TIM2, (PA0<Input<Floating>>, PA1<Input<Floating>>)>,
        pins: EncoderPins,
    ) -> Self {
        Encoders {
            qei,
            pins,
            positions: [0; 8],
            current_encoder: 0,
            last_count: 0,
        }
    }

    fn get_positions(self) -> [i16; 8] {
        self.positions
    }

    fn next_encoder(&mut self) -> bool {
        let current_count = self.qei.count();
        let last_count = self.last_count;
        let diff = current_count.wrapping_sub(last_count) as i16;

        self.positions[self.current_encoder] += diff;

        self.current_encoder += 1;
        if self.current_encoder == 8 {
            self.current_encoder = 0;
        }

        if self.current_encoder & 1 << 0 == 0 {
            self.pins.a0.set_low();
        } else {
            self.pins.a0.set_high();
        }
        if self.current_encoder & 1 << 1 == 0 {
            self.pins.a1.set_low();
        } else {
            self.pins.a1.set_high();
        }
        if self.current_encoder & 1 << 2 == 0 {
            self.pins.a2.set_low();
        } else {
            self.pins.a2.set_high();
        }

        self.last_count = self.qei.count();

        diff != 0
    }
}

pub struct EncoderPins {
    a0: PC0<Output<PushPull>>,
    a1: PC1<Output<PushPull>>,
    a2: PC2<Output<PushPull>>,
}

pub struct Leds {
    pins: LedPins,
    pub banks: [u32; 8],
    current_bank: usize,
}

impl Leds {
    fn new(pins: LedPins) -> Self {
        Leds {
            pins,
            banks: [0; 8],
            current_bank: 0,
        }
    }

    fn get_bank_value(&self, bank: usize) -> u32 {
        self.banks[bank]
    }

    fn set_bank_value(&mut self, bank: usize, value: u32) {
        self.banks[bank] = value;
    }

    fn write_next_bank(&mut self) {
        self.current_bank += 1;
        if self.current_bank == 8 {
            self.current_bank = 0;
        }

        self.pins.hs_en_l.set_high();
        self.pins.ls_en_l.set_high();

        if self.current_bank & 1 << 0 == 0 {
            self.pins.hs_a0.set_low();
        } else {
            self.pins.hs_a0.set_high();
        }
        if self.current_bank & 1 << 1 == 0 {
            self.pins.hs_a1.set_low();
        } else {
            self.pins.hs_a1.set_high();
        }
        if self.current_bank & 1 << 2 == 0 {
            self.pins.hs_a2.set_low();
        } else {
            self.pins.hs_a2.set_high();
        }

        for i in 0..32 {
            if self.banks[self.current_bank] & (1 << i) == 0 {
                self.pins.ls_dai.set_low();
            } else {
                self.pins.ls_dai.set_high();
            }
            delay(4);
            self.pins.ls_dck.set_high();
            delay(4);
            self.pins.ls_dck.set_low();
        }
        delay(4);
        self.pins.ls_lat.set_high();
        delay(4);
        self.pins.ls_lat.set_low();
        delay(4);

        self.pins.ls_en_l.set_low();
        self.pins.hs_en_l.set_low();
    }
}

struct LedPins {
    hs_en_l: PA2<Output<PushPull>>,
    hs_a0: PA3<Output<PushPull>>,
    hs_a1: PA4<Output<PushPull>>,
    hs_a2: PA5<Output<PushPull>>,
    ls_en_l: PB12<Output<PushPull>>,
    ls_dai: PB15<Output<PushPull>>,
    ls_dck: PB14<Output<PushPull>>,
    ls_lat: PB13<Output<PushPull>>,
}
