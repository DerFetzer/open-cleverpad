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

#[derive(Clone, Copy, Debug)]
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
                _ => panic!("This should never happen!"),
            },
            10 => ButtonType::Parameter(PARAMETER_TYPES[row as usize]),
            _ => panic!("This should never happen!"),
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

#[derive(Clone, Copy, Debug)]
pub enum LedColor {
    Black = 0,
    White = 16,
    Yellow = 32,
    Aqua = 48,
    Purple = 64,
    Blue = 80,
    Green = 96,
    Red = 112,
}

#[derive(Clone, Copy, Debug)]
pub struct Rgb {
    pub r: bool,
    pub g: bool,
    pub b: bool,
}

impl LedColor {
    pub fn from_value(v: u8) -> LedColor {
        match v {
            0..=15 => LedColor::Black,
            16..=31 => LedColor::White,
            32..=47 => LedColor::Yellow,
            48..=63 => LedColor::Aqua,
            64..=79 => LedColor::Purple,
            80..=95 => LedColor::Blue,
            96..=111 => LedColor::Green,
            _ => LedColor::Red,
        }
    }

    pub fn as_rgb(&self) -> Rgb {
        match self {
            LedColor::Black => Rgb {
                r: false,
                g: false,
                b: false,
            },
            LedColor::White => Rgb {
                r: true,
                g: true,
                b: true,
            },
            LedColor::Yellow => Rgb {
                r: true,
                g: true,
                b: false,
            },
            LedColor::Aqua => Rgb {
                r: false,
                g: true,
                b: true,
            },
            LedColor::Purple => Rgb {
                r: true,
                g: false,
                b: true,
            },
            LedColor::Blue => Rgb {
                r: false,
                g: false,
                b: true,
            },
            LedColor::Green => Rgb {
                r: false,
                g: true,
                b: false,
            },
            LedColor::Red => Rgb {
                r: true,
                g: false,
                b: false,
            },
        }
    }
}

impl LedEvent {
    pub fn new(btn: ButtonType, event: LedEventType) -> LedEvent {
        LedEvent { btn, event }
    }

    pub fn apply_to_banks(&self, banks: [u32; 8]) -> [u32; 8] {
        let mut new_banks = banks;

        match (self.event, self.btn) {
            (LedEventType::Switch(s), ButtonType::Master(i)) => {
                let bank = 7;
                let bit = 32 - i;

                if s {
                    new_banks[bank] |= 1 << bit;
                } else {
                    new_banks[bank] &= !(1 << bit);
                }
            }
            (LedEventType::Switch(s), ButtonType::Arrow(d)) => {
                let bank = 6;
                let bit = 31 - d as u8;

                if s {
                    new_banks[bank] |= 1 << bit;
                } else {
                    new_banks[bank] &= !(1 << bit);
                }
            }
            (LedEventType::Switch(s), ButtonType::Mode(m)) => {
                let bank = 6;
                let bit = 27 - m as u8;

                if s {
                    new_banks[bank] |= 1 << bit;
                } else {
                    new_banks[bank] &= !(1 << bit);
                }
            }
            (LedEventType::Switch(s), ButtonType::Parameter(p)) => {
                let bank = 6;
                let bit = 23 - p as u8;

                if s {
                    new_banks[bank] |= 1 << bit;
                } else {
                    new_banks[bank] &= !(1 << bit);
                }
            }
            (LedEventType::SwitchColor(color), ButtonType::Pad { x, y }) => {
                let (bank_r, bank_g, bank_b) = if y < 4 { (1, 2, 0) } else { (4, 5, 3) };

                let bit = 31 - (((y % 4) * 8) + x);

                let Rgb { r, g, b } = color.as_rgb();
                if r {
                    new_banks[bank_r] |= 1 << bit;
                } else {
                    new_banks[bank_r] &= !(1 << bit);
                }

                if g {
                    new_banks[bank_g] |= 1 << bit;
                } else {
                    new_banks[bank_g] &= !(1 << bit);
                }

                if b {
                    new_banks[bank_b] |= 1 << bit;
                } else {
                    new_banks[bank_b] &= !(1 << bit);
                }
            }
            _ => panic!("Invalid LED and button types!"),
        };

        new_banks
    }
}
