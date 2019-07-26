use cortex_m::asm::delay;

use asm_delay::AsmDelay;

use stm32f1xx_hal::device::TIM2;
use stm32f1xx_hal::gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5};
use stm32f1xx_hal::gpio::gpiob::{
    PB0, PB1, PB10, PB12, PB13, PB14, PB15, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9,
};
use stm32f1xx_hal::gpio::gpioc::{PC0, PC1, PC10, PC11, PC12, PC13, PC14, PC15, PC2, PC8, PC9};

use stm32f1xx_hal::gpio::{Floating, Input, OpenDrain, Output, PushPull};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::qei::Qei;

pub struct ButtonMatrix {
    pins: ButtonMatrixPins,
    rows: [u8; 11],
    rows_neg1: [u8; 11],
    rows_neg2: [u8; 11],
    delay: AsmDelay,
}

impl ButtonMatrix {
    pub fn new(pins: ButtonMatrixPins, delay: AsmDelay) -> Self {
        ButtonMatrix {
            pins,
            rows: [0; 11],
            rows_neg1: [0; 11],
            rows_neg2: [0; 11],
            delay,
        }
    }

    pub fn get_row(&mut self, row_num: usize) -> u8 {
        self.rows[row_num]
    }

    pub fn get_rows(&mut self) -> [u8; 11] {
        self.rows
    }

    pub fn get_debounced_row(&mut self, row_num: usize) -> u8 {
        self.rows_neg2[row_num] & self.rows_neg1[row_num] & self.rows[row_num]
    }

    pub fn get_debounced_rows(&mut self) -> [u8; 11] {
        let mut debounced_rows = [0_u8; 11];

        for i in 0..11 {
            debounced_rows[i] = self.rows_neg2[i] & self.rows_neg1[i] & self.rows[i]
        }

        debounced_rows
    }

    pub fn read(&mut self) {
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
                _ => panic!("This should never happen"),
            };
            self.delay.delay_us(10_u32);

            let mut row: u8 = 0;
            row |= (self.pins.row1.is_low() as u8) << 0;
            row |= (self.pins.row2.is_low() as u8) << 1;
            row |= (self.pins.row3.is_low() as u8) << 2;
            row |= (self.pins.row4.is_low() as u8) << 3;
            row |= (self.pins.row5.is_low() as u8) << 4;
            row |= (self.pins.row6.is_low() as u8) << 5;
            row |= (self.pins.row7.is_low() as u8) << 6;
            row |= (self.pins.row8.is_low() as u8) << 7;

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
    a_neg3: u8,
    b_state: u8,
    b: u8,
    b_neg1: u8,
    b_neg2: u8,
    b_neg3: u8,
    delay: AsmDelay,
}

impl Encoders {
    pub fn new(pins: EncoderPins, delay: AsmDelay) -> Self {
        Encoders {
            pins,
            positions: [0; 8],
            a_state: 0,
            a: 0,
            a_neg1: 0,
            a_neg2: 0,
            a_neg3: 0,
            b_state: 0,
            b: 0,
            b_neg1: 0,
            b_neg2: 0,
            b_neg3: 0,
            delay,
        }
    }

    pub fn get_positions(&mut self) -> [i32; 8] {
        self.positions
    }

    pub fn read(&mut self) -> bool {
        let mut change = false;

        self.a_neg3 = self.a_neg2;
        self.a_neg2 = self.a_neg1;
        self.a_neg1 = self.a;

        self.b_neg3 = self.b_neg2;
        self.b_neg2 = self.b_neg1;
        self.b_neg1 = self.b;

        for i in 0..8 {
            if i & 1 << 0 == 0 {
                self.pins.a0.set_low();
            } else {
                self.pins.a0.set_high();
            }
            if i & 1 << 1 == 0 {
                self.pins.a1.set_low();
            } else {
                self.pins.a1.set_high();
            }
            if i & 1 << 2 == 0 {
                self.pins.a2.set_low();
            } else {
                self.pins.a2.set_high();
            }

            self.delay.delay_us(20_u32);

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
                (self.a & 1 << i) > 0,
                (self.a_neg1 & 1 << i) > 0,
                (self.a_neg2 & 1 << i) > 0,
                (self.a_neg3 & 1 << i) > 0,
            ) {
                (false, false, false, false) => Some(false),
                (true, true, true, true) => Some(true),
                _ => None,
            };

            let b_stable = match (
                (self.b & 1 << i) > 0,
                (self.b_neg1 & 1 << i) > 0,
                (self.b_neg2 & 1 << i) > 0,
                (self.b_neg3 & 1 << i) > 0,
            ) {
                (false, false, false, false) => Some(false),
                (true, true, true, true) => Some(true),
                _ => None,
            };

            match (
                a_stable,
                b_stable,
                (self.a_state & 1 << i) > 0,
                (self.b_state & 1 << i) > 0,
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
                (Some(false)) => self.a_state &= !(1_u8 << i), // a stable false
                (Some(true)) => self.a_state |= 1_u8 << i,     // a stable true
                _ => (),
            };

            match b_stable {
                (Some(false)) => self.b_state &= !(1_u8 << i), // b stable false
                (Some(true)) => self.b_state |= 1_u8 << i,     // b stable true
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

pub struct Leds {
    pins: LedPins,
    banks: [u32; 8],
    current_bank: usize,
}

impl Leds {
    pub fn new(pins: LedPins) -> Self {
        Leds {
            pins,
            banks: [0; 8],
            current_bank: 0,
        }
    }

    pub fn get_banks(&mut self) -> [u32; 8] {
        self.banks
    }

    pub fn get_bank_value(&mut self, bank: usize) -> u32 {
        self.banks[bank]
    }

    pub fn set_banks(&mut self, banks: [u32; 8]) {
        self.banks = banks;
    }

    pub fn set_bank_value(&mut self, bank: usize, value: u32) {
        self.banks[bank] = value;
    }

    pub fn write_next_bank(&mut self) {
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
