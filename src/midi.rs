pub trait MidiMessage {
    fn to_bytes(&self) -> [u8; 4];
    fn from_bytes(bytes: [u8; 4]) -> Option<Self>
    where
        Self: Sized;
}

#[derive(Debug)]
pub struct NoteOn {
    pub channel: u8,
    pub note: u8,
    pub velocity: u8,
}

impl NoteOn {
    pub fn new(channel: u8, note: u8, velocity: u8) -> Option<NoteOn> {
        if channel <= 0x0F && note <= 0x7F && velocity <= 0x7F {
            Some(NoteOn {
                channel,
                note,
                velocity,
            })
        } else {
            None
        }
    }
}

impl MidiMessage for NoteOn {
    fn to_bytes(&self) -> [u8; 4] {
        [0x09, 0x90 | (self.channel & 0x0F), self.note, self.velocity]
    }

    fn from_bytes(bytes: [u8; 4]) -> Option<Self> {
        match bytes {
            [0x09, status, note, velocity] if status & 0xF0 == 0x90 => {
                NoteOn::new(status & 0x0F, note, velocity)
            }
            _ => None,
        }
    }
}

pub struct NoteOff {
    pub channel: u8,
    pub note: u8,
}

impl NoteOff {
    pub fn new(channel: u8, note: u8) -> Option<NoteOff> {
        if channel <= 0x0F && note <= 0x7F {
            Some(NoteOff { channel, note })
        } else {
            None
        }
    }
}

impl MidiMessage for NoteOff {
    fn to_bytes(&self) -> [u8; 4] {
        [0x08, 0x80 | (self.channel & 0x0F), self.note, 0x00]
    }

    fn from_bytes(bytes: [u8; 4]) -> Option<Self> {
        match bytes {
            [0x08, status, note, _] if status & 0xF0 == 0x80 => NoteOff::new(status & 0x0F, note),
            _ => None,
        }
    }
}

pub struct ControlChange {
    pub channel: u8,
    pub controller: u8,
    pub value: u8,
}

impl ControlChange {
    pub fn new(channel: u8, controller: u8, value: u8) -> Option<ControlChange> {
        if channel <= 0x0F && controller <= 0x7F && value <= 0x7F {
            Some(ControlChange {
                channel,
                controller,
                value,
            })
        } else {
            None
        }
    }
}

impl MidiMessage for ControlChange {
    fn to_bytes(&self) -> [u8; 4] {
        [
            0x0B,
            0xB0 | (self.channel & 0x0F),
            self.controller,
            self.value,
        ]
    }

    fn from_bytes(bytes: [u8; 4]) -> Option<Self> {
        match bytes {
            [0x0B, status, controller, value] if status & 0xF0 == 0xB0 => {
                ControlChange::new(status & 0x0F, controller, value)
            }
            _ => None,
        }
    }
}
