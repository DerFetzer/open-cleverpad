#![allow(unused)]

use core::hash::Hash;
use embedded_graphics::{
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};
use heapless::{Vec, index_set::FnvIndexSet};

use crate::{
    app::update_leds_state,
    hal::{Button, COLOR_BLACK, COLOR_WHITE, PadColor},
    hardware::{LedsState, PadLedsState},
    midi::{Clock, MidiEventPacket, MidiMessage, NoteOff, NoteOn},
    usb_midi::MidiEventPacketSender,
};

#[derive(defmt::Format, Clone, Copy, Default)]
pub enum Scale {
    Major,
    #[default]
    Minor,
}

impl Scale {
    pub fn halftone_offset_from_root(&self, index: u8) -> u8 {
        let offsets = match self {
            Scale::Major => [0, 2, 4, 5, 7, 9, 11, 12],
            Scale::Minor => [0, 2, 3, 5, 7, 8, 10, 12],
        };
        offsets[index as usize]
    }
}

#[derive(defmt::Format, Clone, Copy, Eq)]
pub struct SequencerNote {
    pitch: u8,
    velocity: u8,
    length: u8,
}

impl PartialEq for SequencerNote {
    fn eq(&self, other: &Self) -> bool {
        // Inside a step only the pitch is relevant
        self.pitch == other.pitch
    }
}

impl Hash for SequencerNote {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.pitch.hash(state);
    }
}

impl SequencerNote {
    pub fn new(octave: u8, index: u8, velocity: u8, length: u8) -> Self {
        if octave > 10 || index >= 8 || velocity > 127 {
            defmt::panic!("Invalid note: {}, {}, {}", octave, index, velocity);
        }
        Self {
            pitch: octave << 4 | index & 0x0F,
            velocity,
            length,
        }
    }
    fn get_midi_pitch(&self, key: u8, scale: Scale) -> u8 {
        let octave = self.get_octave();
        let index = self.get_index();

        octave * 12 + key + scale.halftone_offset_from_root(index)
    }

    fn get_octave(&self) -> u8 {
        self.pitch >> 4
    }
    fn get_index(&self) -> u8 {
        self.pitch & 0x0F
    }
}

#[derive(Default, Clone)]
pub struct SequencerStep {
    notes: FnvIndexSet<SequencerNote, 8>,
}

pub struct SequencerChannel {
    steps: Vec<SequencerStep, 16>,
    position: u8,
    midi_channel: u8,
    key: u8,
    scale: Scale,
}

impl SequencerChannel {
    pub const fn new() -> Self {
        Self {
            steps: Vec::new(),
            position: 0,
            midi_channel: 0,
            key: 0,
            scale: Scale::Minor,
        }
    }

    pub fn init(&mut self) {
        self.steps
            .extend_from_slice(
                &[const {
                    SequencerStep {
                        notes: FnvIndexSet::new(),
                    }
                }; 8],
            )
            .unwrap();
    }

    pub fn step<F>(&mut self, mut midi_callback: F)
    where
        F: FnMut(MidiEventPacket),
    {
        if self.position == 0 {
            // Stop all notes that are not already stopped from the last iteration
            for (step_position, step) in self.steps.iter().enumerate() {
                if step_position == 0 {
                    continue;
                }
                for note in step.notes.iter() {
                    if (step_position as u8).saturating_add(note.length) >= self.steps.len() as u8 {
                        let note_off = NoteOff::new(
                            self.midi_channel,
                            note.get_midi_pitch(self.key, self.scale),
                        )
                        .unwrap();
                        defmt::info!("note_off: {}", note_off);
                        midi_callback(note_off.to_bytes());
                    }
                }
            }
        } else {
            // Stop all notes that should stop at this position
            for (step_position, step) in self.steps[..self.position as usize].iter().enumerate() {
                for note in step.notes.iter() {
                    if (step_position as u8).saturating_add(note.length) == self.position {
                        let note_off = NoteOff::new(
                            self.midi_channel,
                            note.get_midi_pitch(self.key, self.scale),
                        )
                        .unwrap();
                        defmt::info!("note_off: {}", note_off);
                        midi_callback(note_off.to_bytes());
                    }
                }
            }
        }

        // Play all notes from this step
        for note_on in self.steps[self.position as usize].notes.iter() {
            let note_on = NoteOn::new(
                self.midi_channel,
                note_on.get_midi_pitch(self.key, self.scale),
                note_on.velocity,
            )
            .unwrap();
            defmt::info!("note_on: {}", note_on);
            midi_callback(note_on.to_bytes());
        }
        self.position += 1;
        if self.position as usize >= self.steps.len() {
            self.position = 0;
        }
    }

    pub fn reset_position(&mut self) {
        self.position = 0;
    }

    pub fn change_step<F>(&mut self, step: u8, index: u8, octave: u8, mut midi_callback: F)
    where
        F: FnMut(MidiEventPacket),
    {
        defmt::info!("change_step: {}, {}, {}", step, index, octave);
        let step = &mut self.steps[step as usize];
        // First row is next octave
        let (octave, index) = if index == 0 {
            (octave + 1, 0)
        } else {
            // Pad row numbering is inverted
            (octave, 7 - index)
        };
        let note = SequencerNote::new(octave, index, 127, 2);
        if step.notes.remove(&note) {
            defmt::info!("Remove note: {}", note);

            let note_off =
                NoteOff::new(self.midi_channel, note.get_midi_pitch(self.key, self.scale)).unwrap();
            defmt::info!("note_off: {}", note_off);

            midi_callback(note_off.to_bytes());
        } else {
            defmt::info!("Add note: {}", note);
            if step.notes.insert(note).is_err() {
                defmt::warn!("Notes are full");
            }
        }
    }

    pub fn draw(&self, octave: u8, pads: &mut PadLedsState) {
        pads.clear(COLOR_BLACK).unwrap();
        let line_style = PrimitiveStyle::with_stroke(PadColor { r: 1, g: 1, b: 1 }, 1);
        Line::new(
            Point::new(self.position as i32, 0),
            Point::new(self.position as i32, 7),
        )
        .into_styled(line_style)
        .draw(pads)
        .unwrap();

        for (step_num, step) in self.steps.iter().enumerate() {
            pads.draw_iter(
                step.notes
                    .iter()
                    .filter(|n| {
                        n.get_octave() == octave
                            || (n.get_octave() == octave + 1 && n.get_index() == 0)
                    })
                    .map(|n| {
                        if n.get_octave() == octave + 1 && n.get_index() == 0 {
                            // Root note of next octave
                            Pixel(Point::new(step_num as i32, 0), COLOR_WHITE)
                        } else {
                            Pixel(
                                Point::new(step_num as i32, 7 - n.get_index() as i32),
                                COLOR_WHITE,
                            )
                        }
                    }),
            )
            .unwrap();
        }
    }
}

pub const NUM_SEQUENCER_CHANNELS: usize = 4;

pub struct Sequencer {
    leds_state: LedsState,
    channels: [SequencerChannel; NUM_SEQUENCER_CHANNELS],
    active_channel: u8,
    active_octave: u8,
    step_divider: u8,
    step_counter: u8,
    midi_sender: Option<MidiEventPacketSender>,
}

impl Sequencer {
    pub const fn new() -> Self {
        Self {
            leds_state: LedsState::new(),
            channels: [const { SequencerChannel::new() }; NUM_SEQUENCER_CHANNELS],
            active_channel: 0,
            active_octave: 5,
            step_counter: 0,
            step_divider: 8, // 24ppqn / 8 = 4 -> 1/16th note
            midi_sender: None,
        }
    }

    pub fn init(&mut self, midi_sender: MidiEventPacketSender) {
        self.midi_sender = Some(midi_sender);
        for channel in &mut self.channels {
            channel.init();
        }
    }

    pub fn reset_position(&mut self) {
        for channel in &mut self.channels {
            channel.reset_position();
        }
        self.draw();
    }

    pub fn draw(&mut self) {
        self.channels[self.active_channel as usize]
            .draw(self.active_octave, &mut self.leds_state.pad_state);
        update_leds_state::spawn(self.leds_state).ok();
    }

    /// Has to be called with 24ppqn
    pub fn step(&mut self) {
        self.midi_sender
            .as_mut()
            .unwrap()
            .try_send(Clock.to_bytes())
            .ok();
        self.step_counter = self.step_counter.wrapping_add(1);
        if self.step_counter.is_multiple_of(self.step_divider) {
            for channel in &mut self.channels {
                channel.step(|packet| {
                    self.midi_sender.as_mut().unwrap().try_send(packet).ok();
                });
            }
        }
        self.draw();
    }

    pub fn set_active_channel(&mut self, channel: u8) {
        self.active_channel = channel;
        self.draw();
    }

    pub fn set_active_octave(&mut self, octave: u8) {
        self.active_octave = octave;
        self.draw();
    }

    pub fn handle_button_press(&mut self, button: Button) {
        match button {
            Button::Pad(pad) => {
                let channel = &mut self.channels[self.active_channel as usize];
                let step = pad.x;
                let index = pad.y;

                channel.change_step(step, index, self.active_octave, |packet| {
                    self.midi_sender.as_mut().unwrap().try_send(packet).ok();
                });
            }
            Button::Control(control_button) => {} // TODO
        }
        self.draw();
    }
}
