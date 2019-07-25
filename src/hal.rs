#[derive(Debug)]
pub enum ButtonType {
    Pad { x: u8, y: u8 },
    Master(u8),
    Arrow(Direction),
    Mode(ModeType),
    Parameter(ParameterType),
}

#[derive(Clone, Copy, Debug)]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug)]
pub enum ModeType {
    Clip,
    One,
    Two,
    Set,
}

#[derive(Clone, Copy, Debug)]
pub enum ParameterType {
    Volume,
    SendA,
    SendB,
    Pan,
    Control1,
    Control2,
    Control3,
    Control4,
}

#[derive(Debug)]
pub enum ButtonEventEdge {
    PosEdge,
    NegEdge,
}

#[derive(Debug)]
pub struct ButtonEvent {
    btn: ButtonType,
    event: ButtonEventEdge,
}

static PARAMETER_TYPES: [ParameterType; 8] = [
    ParameterType::Volume,
    ParameterType::SendA,
    ParameterType::SendB,
    ParameterType::Pan,
    ParameterType::Control1,
    ParameterType::Control2,
    ParameterType::Control3,
    ParameterType::Control4,
];

static DIRECTION_TYPES: [Direction; 4] = [
    Direction::Up,
    Direction::Down,
    Direction::Left,
    Direction::Right,
];

static MODE_TYPES: [ModeType; 4] = [ModeType::Clip, ModeType::One, ModeType::Two, ModeType::Set];

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
