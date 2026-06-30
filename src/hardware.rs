use core::convert::Infallible;

use crate::Mono;
use crate::hal::{ControlButton, ControlButtonLedEvent, LedEvent, Pad, PadColor, PadLedEvent, Rgb};
use cortex_m::asm::delay;
use embedded_graphics::prelude::*;
use rtic_monotonics::rtic_time::embedded_hal_async::delay::DelayNs;
use stm32f1xx_hal::gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5};
use stm32f1xx_hal::gpio::gpiob::{
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB12, PB13, PB14, PB15,
};
use stm32f1xx_hal::gpio::gpioc::{PC0, PC1, PC2, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15};
use stm32f1xx_hal::gpio::{Floating, Input, OpenDrain, Output, PushPull};

pub struct ButtonMatrix {
    pins: ButtonMatrixPins,
    rows: [u8; 11],
    rows_neg1: [u8; 11],
    rows_neg2: [u8; 11],
    debounced_rows: [u8; 11],
}

impl ButtonMatrix {
    pub fn new(pins: ButtonMatrixPins) -> Self {
        ButtonMatrix {
            pins,
            rows: [0; 11],
            rows_neg1: [0; 11],
            rows_neg2: [0; 11],
            debounced_rows: [0; 11],
        }
    }

    pub fn get_row(&mut self, row_num: usize) -> u8 {
        self.rows[row_num]
    }

    pub fn get_rows(&mut self) -> [u8; 11] {
        self.rows
    }

    pub fn get_debounced_row(&mut self, row_num: usize) -> u8 {
        self.debounced_rows[row_num]
    }

    pub fn get_debounced_rows(&mut self) -> [u8; 11] {
        self.debounced_rows
    }

    pub async fn read(&mut self) {
        self.rows_neg2.copy_from_slice(&self.rows_neg1);
        self.rows_neg1.copy_from_slice(&self.rows);

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
                _ => unreachable!(),
            };
            Mono.delay_us(2).await;

            let mut row: u8 = 0;
            row |= self.pins.row1.is_low() as u8;
            row |= (self.pins.row2.is_low() as u8) << 1;
            row |= (self.pins.row3.is_low() as u8) << 2;
            row |= (self.pins.row4.is_low() as u8) << 3;
            row |= (self.pins.row5.is_low() as u8) << 4;
            row |= (self.pins.row6.is_low() as u8) << 5;
            row |= (self.pins.row7.is_low() as u8) << 6;
            row |= (self.pins.row8.is_low() as u8) << 7;

            self.rows[i] = row;

            for (i, row) in self.debounced_rows.iter_mut().enumerate() {
                *row = (self.rows[i] & *row)
                    | (self.rows_neg1[i] & *row)
                    | (self.rows_neg2[i] & *row)
                    | (self.rows[i] & self.rows_neg1[i] & self.rows_neg2[i])
            }

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
                _ => unreachable!(),
            };
        }
    }
}

pub struct ButtonMatrixPins {
    pub row1: PC8<Input<Floating>>,
    pub row2: PC9<Input<Floating>>,
    pub row3: PC10<Input<Floating>>,
    pub row4: PC11<Input<Floating>>,
    pub row5: PC12<Input<Floating>>,
    pub row6: PC13<Input<Floating>>,
    pub row7: PC14<Input<Floating>>,
    pub row8: PC15<Input<Floating>>,
    pub col1: PB0<Output<OpenDrain>>,
    pub col2: PB1<Output<OpenDrain>>,
    pub col3: PB2<Output<OpenDrain>>,
    pub col4: PB3<Output<OpenDrain>>,
    pub col5: PB4<Output<OpenDrain>>,
    pub col6: PB5<Output<OpenDrain>>,
    pub col7: PB6<Output<OpenDrain>>,
    pub col8: PB7<Output<OpenDrain>>,
    pub col9: PB8<Output<OpenDrain>>,
    pub col10: PB9<Output<OpenDrain>>,
    pub col11: PB10<Output<OpenDrain>>,
}

pub struct Encoders {
    pins: EncoderPins,
    positions: [i32; 8],
    a_state: u8,
    a: u8,
    a_neg1: u8,
    a_neg2: u8,
    b_state: u8,
    b: u8,
    b_neg1: u8,
    b_neg2: u8,
}

impl Encoders {
    pub fn new(pins: EncoderPins) -> Self {
        Encoders {
            pins,
            positions: [0; 8],
            a_state: 0,
            a: 0,
            a_neg1: 0,
            a_neg2: 0,
            b_state: 0,
            b: 0,
            b_neg1: 0,
            b_neg2: 0,
        }
    }

    pub fn get_positions(&mut self) -> [i32; 8] {
        self.positions
    }

    pub async fn read(&mut self) -> bool {
        let mut change = false;

        self.a_neg2 = self.a_neg1;
        self.a_neg1 = self.a;

        self.b_neg2 = self.b_neg1;
        self.b_neg1 = self.b;

        for i in 0..8 {
            if i & (1 << 0) == 0 {
                self.pins.a0.set_low();
            } else {
                self.pins.a0.set_high();
            }
            if i & (1 << 1) == 0 {
                self.pins.a1.set_low();
            } else {
                self.pins.a1.set_high();
            }
            if i & (1 << 2) == 0 {
                self.pins.a2.set_low();
            } else {
                self.pins.a2.set_high();
            }

            Mono.delay_us(2).await;

            if self.pins.a.is_low() {
                self.a |= 1_u8 << i;
            } else {
                self.a &= !(1_u8 << i);
            }

            if self.pins.b.is_low() {
                self.b |= 1_u8 << i;
            } else {
                self.b &= !(1_u8 << i);
            }

            let a_stable = match (
                (self.a & (1 << i)) > 0,
                (self.a_neg1 & (1 << i)) > 0,
                (self.a_neg2 & (1 << i)) > 0,
            ) {
                (false, false, false) => Some(false),
                (true, true, true) => Some(true),
                _ => None,
            };

            let b_stable = match (
                (self.b & (1 << i)) > 0,
                (self.b_neg1 & (1 << i)) > 0,
                (self.b_neg2 & (1 << i)) > 0,
            ) {
                (false, false, false) => Some(false),
                (true, true, true) => Some(true),
                _ => None,
            };

            match (
                a_stable,
                b_stable,
                (self.a_state & (1 << i)) > 0,
                (self.b_state & (1 << i)) > 0,
            ) {
                (Some(true), Some(true), false, true) => {
                    change = true;
                    self.positions[i] += 1;
                }
                (Some(true), Some(true), true, false) => {
                    change = true;
                    self.positions[i] -= 1;
                }
                _ => (),
            };

            match a_stable {
                Some(false) => self.a_state &= !(1_u8 << i), // a stable false
                Some(true) => self.a_state |= 1_u8 << i,     // a stable true
                _ => (),
            };

            match b_stable {
                Some(false) => self.b_state &= !(1_u8 << i), // b stable false
                Some(true) => self.b_state |= 1_u8 << i,     // b stable true
                _ => (),
            };
        }

        change
    }
}

pub struct EncoderPins {
    pub a0: PC0<Output<PushPull>>,
    pub a1: PC1<Output<PushPull>>,
    pub a2: PC2<Output<PushPull>>,
    pub a: PA1<Input<Floating>>,
    pub b: PA0<Input<Floating>>,
}

pub const NUM_TIME_SLOTS: usize = 3;

pub const NUM_PAD_BANKS: usize = 6;
pub const NUM_BUTTON_BANKS: usize = 2;

pub const NUM_BANKS: usize = NUM_PAD_BANKS + NUM_BUTTON_BANKS;

#[derive(Clone, Copy)]
pub enum Bank {
    Pad(PadBank),
    Control(ControlButtonBank),
}

#[derive(Clone, Copy)]
pub enum PadBank {
    B0 = 0,
    R0 = 1,
    G0 = 2,
    B1 = 3,
    R1 = 4,
    G1 = 5,
}

impl PadBank {
    pub const fn all() -> [Self; 6] {
        [Self::B0, Self::R0, Self::G0, Self::B1, Self::R1, Self::G1]
    }
}

#[derive(Clone, Copy)]
pub enum ControlButtonBank {
    C0 = 0,
    C1 = 1,
}

impl ControlButtonBank {
    pub const fn all() -> [Self; 2] {
        [Self::C0, Self::C1]
    }
}

#[derive(Default, Copy, Clone)]
pub struct PadLedsState(pub [[u32; NUM_PAD_BANKS]; NUM_TIME_SLOTS]);

impl PadLedsState {
    pub fn apply_event(&mut self, event: PadLedEvent) {
        let x = event.pad.x;
        let y = event.pad.y;

        let color = event.color;

        let (bank_r, bank_g, bank_b) = if y < 4 {
            (
                PadBank::R0 as usize,
                PadBank::G0 as usize,
                PadBank::B0 as usize,
            )
        } else {
            (
                PadBank::R1 as usize,
                PadBank::G1 as usize,
                PadBank::B1 as usize,
            )
        };

        let bit = 31 - (((y % 4) * 8) + x);

        for (new_bank, Rgb { r, g, b }) in self.0.iter_mut().zip(color.as_rgb().into_iter()) {
            if r {
                new_bank[bank_r] |= 1 << bit;
            } else {
                new_bank[bank_r] &= !(1 << bit);
            }

            if g {
                new_bank[bank_g] |= 1 << bit;
            } else {
                new_bank[bank_g] &= !(1 << bit);
            }

            if b {
                new_bank[bank_b] |= 1 << bit;
            } else {
                new_bank[bank_b] &= !(1 << bit);
            }
        }
    }
}

impl DrawTarget for PadLedsState {
    type Color = PadColor;

    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels.into_iter() {
            if point.x >= 0 && point.x < 8 && point.y >= 0 && point.y < 8 {
                let event = PadLedEvent {
                    pad: Pad {
                        x: point.x as u8,
                        y: point.y as u8,
                    },
                    color,
                };
                self.apply_event(event);
            }
        }
        Ok(())
    }
}

impl OriginDimensions for PadLedsState {
    fn size(&self) -> Size {
        Size::new(8, 8)
    }
}

#[derive(Default, Copy, Clone)]
pub struct ControlButtonLedsState(pub [[u32; NUM_BUTTON_BANKS]; NUM_TIME_SLOTS]);

impl ControlButtonLedsState {
    pub fn apply_event(&mut self, event: ControlButtonLedEvent) {
        let on = event.on;

        match event.btn {
            ControlButton::Master(i) => {
                let bank = ControlButtonBank::C1 as usize;
                let bit = 32 - i;

                for new_bank in self.0.iter_mut() {
                    if on {
                        new_bank[bank] |= 1 << bit;
                    } else {
                        new_bank[bank] &= !(1 << bit);
                    }
                }
            }
            ControlButton::Arrow(d) => {
                let bank = ControlButtonBank::C0 as usize;

                let bit = 31 - d as u8;

                for new_bank in self.0.iter_mut() {
                    if on {
                        new_bank[bank] |= 1 << bit;
                    } else {
                        new_bank[bank] &= !(1 << bit);
                    }
                }
            }
            ControlButton::Mode(m) => {
                let bank = ControlButtonBank::C0 as usize;
                let bit = 27 - m as u8;

                for new_bank in self.0.iter_mut() {
                    if on {
                        new_bank[bank] |= 1 << bit;
                    } else {
                        new_bank[bank] &= !(1 << bit);
                    }
                }
            }
            ControlButton::Parameter(p) => {
                let bank = ControlButtonBank::C0 as usize;
                let bit = 23 - p as u8;

                for new_bank in self.0.iter_mut() {
                    if on {
                        new_bank[bank] |= 1 << bit;
                    } else {
                        new_bank[bank] &= !(1 << bit);
                    }
                }
            }
        }
    }
}
pub struct Leds {
    pins: LedPins,
    pad_state: PadLedsState,
    control_button_state: ControlButtonLedsState,
    current_bank: usize,
    current_time_slot: usize,
}

impl Leds {
    pub fn new(mut pins: LedPins) -> Self {
        pins.hs_en_l.set_low();
        pins.ls_en_l.set_low();
        Leds {
            pins,
            pad_state: Default::default(),
            control_button_state: Default::default(),
            current_bank: 0,
            current_time_slot: 0,
        }
    }

    pub fn get_pad_state(&self) -> PadLedsState {
        self.pad_state
    }
    pub fn get_pad_state_mut(&mut self) -> &mut PadLedsState {
        &mut self.pad_state
    }

    pub fn get_button_state(&self) -> ControlButtonLedsState {
        self.control_button_state
    }

    pub fn apply_event(&mut self, event: LedEvent) {
        match event {
            LedEvent::Pad(pad_led_event) => self.apply_pad_event(pad_led_event),
            LedEvent::ControlButton(control_button_led_event) => {
                self.apply_control_button_event(control_button_led_event)
            }
        }
    }

    pub fn apply_control_button_event(&mut self, event: ControlButtonLedEvent) {
        self.control_button_state.apply_event(event);
    }

    pub fn apply_pad_event(&mut self, event: PadLedEvent) {
        self.pad_state.apply_event(event);
    }

    /// Gets the bank values for all time slots
    pub fn get_bank_slot_values(&mut self, bank: Bank) -> [u32; NUM_TIME_SLOTS] {
        match bank {
            Bank::Pad(bank) => [
                self.pad_state.0[0][bank as usize],
                self.pad_state.0[1][bank as usize],
                self.pad_state.0[2][bank as usize],
            ],
            Bank::Control(bank) => [
                self.control_button_state.0[0][bank as usize],
                self.control_button_state.0[1][bank as usize],
                self.control_button_state.0[2][bank as usize],
            ],
        }
    }

    pub fn set_pad_state(&mut self, state: PadLedsState) {
        self.pad_state = state;
    }

    pub fn set_button_state(&mut self, state: ControlButtonLedsState) {
        self.control_button_state = state;
    }

    /// Sets the bank values for all time slots
    pub fn set_bank_slot_values(&mut self, bank: Bank, value: [u32; NUM_TIME_SLOTS]) {
        for (i, v) in value.into_iter().enumerate() {
            match bank {
                Bank::Pad(bank) => {
                    self.pad_state.0[i][bank as usize] = v;
                }
                Bank::Control(bank) => {
                    self.control_button_state.0[i][bank as usize] = v;
                }
            }
        }
    }

    /// Returns the number of the current time slot.
    // #[inline(never)]
    pub fn write_next_bank(&mut self) -> usize {
        // VDD is around 4.2V since the USB voltage has polaity protection via a normal diode.
        // Switching characteristics are only given for 3.3V and 5V so all calculations use an
        // estimate middle value.
        //
        // 72MHz clock speed has a cycle time of around 13ns

        self.current_bank += 1;
        if self.current_bank == NUM_BANKS {
            self.current_bank = 0;

            self.current_time_slot += 1;
            if self.current_time_slot == NUM_TIME_SLOTS {
                self.current_time_slot = 0;
            }
        }
        let mut bank_value = if self.current_bank < NUM_PAD_BANKS {
            self.pad_state.0[self.current_time_slot][self.current_bank]
        } else {
            self.control_button_state.0[self.current_time_slot][self.current_bank - NUM_PAD_BANKS]
        };

        // Shift values into LED drivers
        for _ in 0..32 {
            if bank_value & 1 == 0 {
                self.pins.ls_dai.set_low();
            } else {
                self.pins.ls_dai.set_high();
            }
            bank_value >>= 1;
            self.pins.ls_dck.set_high();
            // Pulse width for DCK should be higher than 15ns
            delay(2);
            self.pins.ls_dck.set_low();
        }

        // Disable output before switching common anodes
        self.pins.hs_en_l.set_high();
        self.pins.ls_en_l.set_high();

        // Wait for propagation delay and current fall time: 18ns + 15ns = 33ns
        delay(3);

        self.pins.ls_lat.set_high();
        // Pulse width for LAT should be higher than 15ns
        delay(2);
        self.pins.ls_lat.set_low();

        // Set mux address -> enable common anode
        if self.current_bank & (1 << 0) == 0 {
            self.pins.hs_a0.set_low();
        } else {
            self.pins.hs_a0.set_high();
        }
        if self.current_bank & (1 << 1) == 0 {
            self.pins.hs_a1.set_low();
        } else {
            self.pins.hs_a1.set_high();
        }
        if self.current_bank & (1 << 2) == 0 {
            self.pins.hs_a2.set_low();
        } else {
            self.pins.hs_a2.set_high();
        }

        // Wait for LEDs in previous bank to go out
        delay(4);

        self.pins.ls_en_l.set_low();
        self.pins.hs_en_l.set_low();

        self.current_time_slot
    }
}

pub struct LedPins {
    pub hs_en_l: PA2<Output<PushPull>>,
    pub hs_a0: PA3<Output<PushPull>>,
    pub hs_a1: PA4<Output<PushPull>>,
    pub hs_a2: PA5<Output<PushPull>>,
    pub ls_en_l: PB12<Output<PushPull>>,
    pub ls_dai: PB15<Output<PushPull>>,
    pub ls_dck: PB14<Output<PushPull>>,
    pub ls_lat: PB13<Output<PushPull>>,
}
