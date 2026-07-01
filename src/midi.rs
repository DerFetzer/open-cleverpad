use core::cmp::min;
use heapless::Vec;
use num_enum::TryFromPrimitive;

pub type MidiEventPacket = [u8; 4];

pub trait MidiMessage {
    fn to_bytes(&self) -> MidiEventPacket;
    fn from_bytes(bytes: MidiEventPacket) -> Option<Self>
    where
        Self: Sized;
}

#[derive(Debug, defmt::Format)]
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
    fn to_bytes(&self) -> MidiEventPacket {
        [0x09, 0x90 | (self.channel & 0x0F), self.note, self.velocity]
    }

    fn from_bytes(bytes: MidiEventPacket) -> Option<Self> {
        match bytes {
            [0x09, status, note, velocity] if status & 0xF0 == 0x90 => {
                NoteOn::new(status & 0x0F, note, velocity)
            }
            _ => None,
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
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
    fn to_bytes(&self) -> MidiEventPacket {
        [0x08, 0x80 | (self.channel & 0x0F), self.note, 0x00]
    }

    fn from_bytes(bytes: MidiEventPacket) -> Option<Self> {
        match bytes {
            [0x08, status, note, _] if status & 0xF0 == 0x80 => NoteOff::new(status & 0x0F, note),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
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
    fn to_bytes(&self) -> MidiEventPacket {
        [
            0x0B,
            0xB0 | (self.channel & 0x0F),
            self.controller,
            self.value,
        ]
    }

    fn from_bytes(bytes: MidiEventPacket) -> Option<Self> {
        match bytes {
            [0x0B, status, controller, value] if status & 0xF0 == 0xB0 => {
                ControlChange::new(status & 0x0F, controller, value)
            }
            _ => None,
        }
    }
}

macro_rules! real_time_message {
    ($name:ident, $byte:literal) => {
        #[derive(Copy, Clone)]
        pub struct $name;

        impl MidiMessage for $name {
            fn to_bytes(&self) -> MidiEventPacket {
                [0x0F, $byte, 0x00, 0x00]
            }

            fn from_bytes(bytes: MidiEventPacket) -> Option<Self>
            where
                Self: Sized,
            {
                if bytes[0] == 0x0F && bytes[1] == $byte {
                    Some(Self)
                } else {
                    None
                }
            }
        }
    };
}

real_time_message!(Clock, 0xF8);
real_time_message!(Start, 0xFA);
real_time_message!(Continue, 0xFB);
real_time_message!(Stop, 0xFC);

const SYSEX_MAX_SIZE: usize = 128;

#[derive(defmt::Format)]
pub struct SysEx {
    data: Vec<u8, SYSEX_MAX_SIZE>,
}

#[derive(defmt::Format)]
pub enum SysExBufferResponse {
    SysEx(SysEx),
    InSysEx,
    Other(MidiEventPacket),
}

#[derive(defmt::Format)]
pub enum SysExBufferError {
    Invalid,
    Overflow,
}

const SYSEX_START: u8 = 0xF0;
const SYSEX_END: u8 = 0xF7;

#[derive(defmt::Format)]
pub struct SysExBuffer {
    message_in_progress: bool,
    buf: Vec<u8, SYSEX_MAX_SIZE>,
}

impl SysExBuffer {
    pub const fn new() -> Self {
        Self {
            message_in_progress: false,
            buf: Vec::new(),
        }
    }

    pub fn handle_packet(
        &mut self,
        bytes: MidiEventPacket,
    ) -> Result<SysExBufferResponse, SysExBufferError> {
        if ![0x04, 0x05, 0x06, 0x07].contains(&bytes[0]) {
            // No SysEx packet
            return Ok(SysExBufferResponse::Other(bytes));
        }
        if bytes[0] == 0x04 {
            if bytes[1] == SYSEX_START {
                defmt::info!("SysEx start");
                if self.message_in_progress {
                    self.reset();
                    return Err(SysExBufferError::Invalid);
                }
                self.message_in_progress = true;
                self.buf.clear();
                self.buf.extend_from_slice(&bytes[2..]).unwrap();
                Ok(SysExBufferResponse::InSysEx)
            } else {
                defmt::info!("SysEx continues");
                if !self.message_in_progress {
                    return Err(SysExBufferError::Invalid);
                }
                if self.buf.extend_from_slice(&bytes[1..]).is_err() {
                    Err(SysExBufferError::Overflow)
                } else {
                    Ok(SysExBufferResponse::InSysEx)
                }
            }
        } else if bytes[0] == 0x05 {
            if bytes[1] == SYSEX_END {
                defmt::info!("SysEx ends with following single byte");
                if self.message_in_progress {
                    Ok(self.finish_sys_ex())
                } else {
                    Err(SysExBufferError::Invalid)
                }
            } else {
                defmt::info!("Single-byte System Comon Message");
                Ok(SysExBufferResponse::Other(bytes))
            }
        } else if bytes[0] == 0x06 {
            defmt::info!("SysEx ends with following two bytes");
            if bytes[1] == SYSEX_START && bytes[2] == SYSEX_END {
                defmt::info!("Empty SysEx");
                return Ok(SysExBufferResponse::SysEx(SysEx {
                    data: Default::default(),
                }));
            }
            if !self.message_in_progress {
                return Err(SysExBufferError::Invalid);
            }
            if bytes[2] == SYSEX_END {
                if self.buf.push(bytes[1]).is_err() {
                    Err(SysExBufferError::Overflow)
                } else {
                    Ok(self.finish_sys_ex())
                }
            } else {
                Err(SysExBufferError::Invalid)
            }
        } else if bytes[0] == 0x07 {
            defmt::info!("SysEx ends with following three bytes");
            if bytes[1] == SYSEX_START && bytes[3] == SYSEX_END {
                defmt::info!("One byte SysEx");
                let mut data = Vec::new();
                data.push(bytes[2]).unwrap();
                return Ok(SysExBufferResponse::SysEx(SysEx { data }));
            }
            if !self.message_in_progress {
                return Err(SysExBufferError::Invalid);
            }
            if bytes[3] == SYSEX_END {
                if self.buf.extend_from_slice(&bytes[1..3]).is_err() {
                    Err(SysExBufferError::Overflow)
                } else {
                    Ok(self.finish_sys_ex())
                }
            } else {
                Err(SysExBufferError::Invalid)
            }
        } else {
            Ok(SysExBufferResponse::Other(bytes))
        }
    }

    fn finish_sys_ex(&mut self) -> SysExBufferResponse {
        let sysex = SysEx {
            data: self.buf.clone(),
        };
        self.buf.clear();
        self.message_in_progress = false;
        SysExBufferResponse::SysEx(sysex)
    }

    fn reset(&mut self) {
        self.message_in_progress = false;
        self.buf.clear();
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, TryFromPrimitive, defmt::Format)]
#[repr(u8)]
pub enum EncoderMode {
    EncR = 0,
    EncL,
    Enc2,
    EncB,
    Abs,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, defmt::Format)]
pub struct EncoderParameters {
    pub mode: EncoderMode,
    pub speed_multiplier: u8,
}

impl EncoderParameters {
    pub fn diff_to_value(&self, diff: i32) -> u8 {
        let offset: u8 = match diff.abs() {
            1 => 1,
            num => min(num * self.speed_multiplier as i32, 63) as u8,
        };
        let value = match self.mode {
            EncoderMode::EncR => {
                if diff.signum() != -1 {
                    // most significant bit => direction
                    offset | (1 << 6)
                } else {
                    offset
                }
            }
            EncoderMode::EncL => {
                if diff.signum() == -1 {
                    // most significant bit => direction
                    offset | (1 << 6)
                } else {
                    offset
                }
            }
            EncoderMode::Enc2 => (offset as i32 * diff.signum()) as u8,
            EncoderMode::EncB => (offset as i32 * diff.signum()).saturating_add(64) as u8,
            EncoderMode::Abs => unimplemented!(),
        };
        value & 0x7F
    }
    pub fn apply_diff_to_abs_value(&self, diff: i32, value: u8) -> u8 {
        let offset: u8 = match diff.abs() {
            1 => 1,
            num => min(num * self.speed_multiplier as i32, 127) as u8,
        };
        value
            .saturating_add_signed((offset as i32 * diff.signum().clamp(-127, 127)) as i8)
            .clamp(0, 127)
    }
}
