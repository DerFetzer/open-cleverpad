use cortex_m::asm::delay;
use embedded_graphics::pixelcolor::PixelColor;
use num_enum::TryFromPrimitive;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Button {
    Pad(Pad),
    Control(ControlButton),
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Pad {
    pub x: u8,
    pub y: u8,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum ControlButton {
    Master(u8),
    Arrow(Direction),
    Mode(Mode),
    Parameter(Parameter),
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Direction {
    Up = 0,
    Down,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Mode {
    Clip = 0,
    Mode1,
    Mode2,
    Set,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, TryFromPrimitive, defmt::Format)]
#[repr(u8)]
pub enum Parameter {
    Volume = 0,
    SendA,
    SendB,
    Pan,
    Control1,
    Control2,
    Control3,
    Control4,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum ButtonEventEdge {
    PosEdge,
    NegEdge,
}

pub static PARAMETER_TYPES: [Parameter; 8] = [
    Parameter::Volume,
    Parameter::SendA,
    Parameter::SendB,
    Parameter::Pan,
    Parameter::Control1,
    Parameter::Control2,
    Parameter::Control3,
    Parameter::Control4,
];

pub static DIRECTION_TYPES: [Direction; 4] = [
    Direction::Up,
    Direction::Down,
    Direction::Left,
    Direction::Right,
];

pub static MODE_TYPES: [Mode; 4] = [Mode::Clip, Mode::Mode1, Mode::Mode2, Mode::Set];

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct ButtonEvent {
    pub btn: Button,
    pub event: ButtonEventEdge,
}

impl ButtonEvent {
    pub fn new(row: u8, col: u8, event: ButtonEventEdge) -> ButtonEvent {
        let btn = match col {
            0..=7 => {
                let x = row;
                let y = col;

                Button::Pad(Pad { x, y })
            }
            8 => Button::Control(ControlButton::Master(row + 1)),
            9 => match row {
                0..=3 => Button::Control(ControlButton::Arrow(DIRECTION_TYPES[row as usize])),
                4..=7 => Button::Control(ControlButton::Mode(MODE_TYPES[row as usize - 4])),
                _ => unreachable!(),
            },
            10 => Button::Control(ControlButton::Parameter(PARAMETER_TYPES[row as usize])),
            _ => unreachable!(),
        };

        ButtonEvent { btn, event }
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub enum LedEvent {
    Pad(PadLedEvent),
    ControlButton(ControlButtonLedEvent),
}

#[derive(Clone, Copy, defmt::Format)]
pub struct PadLedEvent {
    pub pad: Pad,
    pub color: PadColor,
}

#[derive(Clone, Copy, defmt::Format)]
pub struct ControlButtonLedEvent {
    pub btn: ControlButton,
    pub on: bool,
}

pub const COLOR_BLACK: PadColor = PadColor { r: 0, g: 0, b: 0 };
pub const COLOR_WHITE: PadColor = PadColor { r: 3, g: 3, b: 3 };
pub const COLOR_YELLOW: PadColor = PadColor { r: 3, g: 3, b: 0 };
pub const COLOR_AQUA: PadColor = PadColor { r: 0, g: 3, b: 3 };
pub const COLOR_PURPLE: PadColor = PadColor { r: 3, g: 0, b: 3 };
pub const COLOR_BLUE: PadColor = PadColor { r: 0, g: 0, b: 3 };
pub const COLOR_GREEN: PadColor = PadColor { r: 0, g: 3, b: 0 };
pub const COLOR_RED: PadColor = PadColor { r: 3, g: 0, b: 0 };

/// Describes a pad's color using RGB values in the range of [0, 3].
///
/// The time is divided into 4 slots. The values tell how many of the slots the corresponding LED
/// should be turned on.
///
/// Time slots have different lengths to implement some kind of gamma correction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub struct PadColor {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl PixelColor for PadColor {
    type Raw = ();
}

/// Binary RGB value.
///
/// Indicates which of the LEDs should be on.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Rgb {
    pub r: bool,
    pub g: bool,
    pub b: bool,
}

impl PadColor {
    pub fn new(r: u8, g: u8, b: u8) -> Option<PadColor> {
        if r < 4 && g < 4 && b < 4 {
            Some(PadColor { r, g, b })
        } else {
            None
        }
    }

    /// The 2-bit values for the color components are encoded into the lowest 6 bits.
    pub fn from_value(v: u8) -> PadColor {
        PadColor::new(v & 0b11, (v & (0b11 << 2)) >> 2, (v & (0b11 << 4)) >> 4).unwrap()
    }

    /// Calculates which LEDs should be on in each of the 4 cycles.
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

pub fn delay_us(us: u32) {
    const SYSCLK_MHZ: u32 = 72;

    delay(SYSCLK_MHZ * us);
}
