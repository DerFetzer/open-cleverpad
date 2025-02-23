use cortex_m::asm::delay;
use num_enum::TryFromPrimitive;

#[derive(Clone, Copy, Debug)]
pub enum ButtonType {
    Pad { x: u8, y: u8 },
    Master(u8),
    Arrow(Direction),
    Mode(ModeType),
    Parameter(ParameterType),
}

#[derive(Clone, Copy, Debug)]
pub enum Direction {
    Up = 0,
    Down,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug)]
pub enum ModeType {
    Clip = 0,
    Mode1,
    Mode2,
    Set,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum ParameterType {
    Volume = 0,
    SendA,
    SendB,
    Pan,
    Control1,
    Control2,
    Control3,
    Control4,
}

#[derive(Clone, Copy, Debug)]
pub enum ButtonEventEdge {
    PosEdge,
    NegEdge,
}

#[derive(Clone, Copy, Debug)]
pub struct ButtonEvent {
    pub btn: ButtonType,
    pub event: ButtonEventEdge,
}

pub static PARAMETER_TYPES: [ParameterType; 8] = [
    ParameterType::Volume,
    ParameterType::SendA,
    ParameterType::SendB,
    ParameterType::Pan,
    ParameterType::Control1,
    ParameterType::Control2,
    ParameterType::Control3,
    ParameterType::Control4,
];

pub static DIRECTION_TYPES: [Direction; 4] = [
    Direction::Up,
    Direction::Down,
    Direction::Left,
    Direction::Right,
];

pub static MODE_TYPES: [ModeType; 4] = [
    ModeType::Clip,
    ModeType::Mode1,
    ModeType::Mode2,
    ModeType::Set,
];

impl ButtonEvent {
    pub fn new(row: u8, col: u8, event: ButtonEventEdge) -> ButtonEvent {
        let btn = match col {
            0..=7 => {
                let x = row;
                let y = col;

                ButtonType::Pad { x, y }
            }
            8 => ButtonType::Master(row + 1),
            9 => match row {
                0..=3 => ButtonType::Arrow(DIRECTION_TYPES[row as usize]),
                4..=7 => ButtonType::Mode(MODE_TYPES[row as usize - 4]),
                _ => unreachable!(),
            },
            10 => ButtonType::Parameter(PARAMETER_TYPES[row as usize]),
            _ => unreachable!(),
        };

        ButtonEvent { btn, event }
    }
}

pub struct LedEvent {
    btn: ButtonType,
    event: LedEventType,
}

#[derive(Clone, Copy, Debug)]
pub enum LedEventType {
    Switch(bool),
    SwitchColor(LedColor),
}

pub const COLOR_BLACK: LedColor = LedColor { r: 0, g: 0, b: 0 };
pub const COLOR_WHITE: LedColor = LedColor { r: 3, g: 3, b: 3 };
pub const COLOR_YELLOW: LedColor = LedColor { r: 3, g: 3, b: 0 };
pub const COLOR_AQUA: LedColor = LedColor { r: 0, g: 3, b: 3 };
pub const COLOR_PURPLE: LedColor = LedColor { r: 3, g: 0, b: 3 };
pub const COLOR_BLUE: LedColor = LedColor { r: 0, g: 0, b: 3 };
pub const COLOR_GREEN: LedColor = LedColor { r: 0, g: 3, b: 0 };
pub const COLOR_RED: LedColor = LedColor { r: 3, g: 0, b: 0 };

#[derive(Clone, Copy, Debug)]
pub struct LedColor {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct Rgb {
    pub r: bool,
    pub g: bool,
    pub b: bool,
}

impl LedColor {
    pub fn new(r: u8, g: u8, b: u8) -> Option<LedColor> {
        if r < 4 && g < 4 && b < 4 {
            Some(LedColor { r, g, b })
        } else {
            None
        }
    }

    pub fn from_value(v: u8) -> LedColor {
        LedColor::new(v & 0b11, (v & (0b11 << 2)) >> 2, (v & (0b11 << 4)) >> 4).unwrap()
    }

    pub fn as_rgb(&self) -> [Rgb; 4] {
        let mut rgb = [Rgb {
            r: false,
            g: false,
            b: false,
        }; 4];

        if self.r >= 1 {
            rgb[0].r = true;
        }
        if self.r >= 2 {
            rgb[1].r = true;
        }
        if self.r >= 3 {
            rgb[2].r = true;
        }

        if self.g >= 1 {
            rgb[0].g = true;
        }
        if self.g >= 2 {
            rgb[1].g = true;
        }
        if self.g >= 3 {
            rgb[2].g = true;
        }

        if self.b >= 1 {
            rgb[0].b = true;
        }
        if self.b >= 2 {
            rgb[1].b = true;
        }
        if self.b >= 3 {
            rgb[2].b = true;
        }

        rgb
    }
}

impl LedEvent {
    pub fn new(btn: ButtonType, event: LedEventType) -> LedEvent {
        LedEvent { btn, event }
    }

    pub fn apply_to_banks(&self, banks: [[u32; 8]; 4]) -> [[u32; 8]; 4] {
        let mut new_banks = banks;

        match (self.event, self.btn) {
            (LedEventType::Switch(s), ButtonType::Master(i)) => {
                let bank = 7;
                let bit = 32 - i;

                for new_bank in new_banks.iter_mut() {
                    if s {
                        new_bank[bank] |= 1 << bit;
                    } else {
                        new_bank[bank] &= !(1 << bit);
                    }
                }
            }
            (LedEventType::Switch(s), ButtonType::Arrow(d)) => {
                let bank = 6;
                let bit = 31 - d as u8;

                for new_bank in new_banks.iter_mut() {
                    if s {
                        new_bank[bank] |= 1 << bit;
                    } else {
                        new_bank[bank] &= !(1 << bit);
                    }
                }
            }
            (LedEventType::Switch(s), ButtonType::Mode(m)) => {
                let bank = 6;
                let bit = 27 - m as u8;

                for new_bank in new_banks.iter_mut() {
                    if s {
                        new_bank[bank] |= 1 << bit;
                    } else {
                        new_bank[bank] &= !(1 << bit);
                    }
                }
            }
            (LedEventType::Switch(s), ButtonType::Parameter(p)) => {
                let bank = 6;
                let bit = 23 - p as u8;

                for new_bank in new_banks.iter_mut() {
                    if s {
                        new_bank[bank] |= 1 << bit;
                    } else {
                        new_bank[bank] &= !(1 << bit);
                    }
                }
            }
            (LedEventType::SwitchColor(color), ButtonType::Pad { x, y }) => {
                let (bank_r, bank_g, bank_b) = if y < 4 { (1, 2, 0) } else { (4, 5, 3) };

                let bit = 31 - (((y % 4) * 8) + x);

                for (new_bank, Rgb { r, g, b }) in
                    new_banks.iter_mut().zip(color.as_rgb().into_iter())
                {
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
            _ => unreachable!(),
        };

        new_banks
    }
}

pub fn delay_us(us: u32) {
    const SYSCLK_HZ: u32 = 72_000_000;

    delay(72 * us);
}
