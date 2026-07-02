use crate::{
    app::update_leds_state,
    hal::{
        Button, ButtonEvent, ButtonEventEdge, COLOR_AQUA, COLOR_BLACK, COLOR_BLUE, COLOR_GREEN,
        COLOR_PURPLE, COLOR_RED, COLOR_WHITE, COLOR_YELLOW, ControlButton, ControlButtonLedEvent,
        DIRECTION_TYPES, EncoderEvent, LedEvent, MODE_TYPES, Pad, PadColor, PadLedEvent, Parameter,
    },
    hardware::{ControlButtonLedsState, LedsState, PadLedsState},
    midi::{ControlChange, EncoderMode, EncoderParameters, Midi, MidiMessage, NoteOff, NoteOn},
    usb_midi::MidiEventPacketSender,
};

#[allow(unused)]
pub trait Handler {
    fn draw(&self);
    fn handle_button_event(&mut self, event: ButtonEvent);
    fn handle_midi(&mut self, midi: Midi);
    fn handle_encoder_event(&mut self, event: EncoderEvent);
}

pub struct ControllerHandler {
    current_channel: u8,
    control_button_leds_state: ControlButtonLedsState,
    leds_state_for_channel: [PadLedsState; 8],
    encoder_parameter_type: Parameter,
    abs_encoder_positions: [[u8; 8]; 8],
    encoder_parameters: [EncoderParameters; 8],
    midi_sender: MidiEventPacketSender,
}

impl ControllerHandler {
    pub fn new(midi_sender: MidiEventPacketSender) -> Self {
        let master_led_event = ControlButtonLedEvent {
            btn: ControlButton::Master(1),
            on: true,
        };
        let mut control_button_leds_state = ControlButtonLedsState::new();
        control_button_leds_state.apply_event(master_led_event);

        let parameter_led_event = ControlButtonLedEvent {
            btn: ControlButton::Parameter(Parameter::Volume),
            on: true,
        };
        control_button_leds_state.apply_event(parameter_led_event);

        let slf = Self {
            current_channel: 1,
            control_button_leds_state,
            leds_state_for_channel: Default::default(),
            encoder_parameter_type: Parameter::Volume,
            abs_encoder_positions: Default::default(),
            encoder_parameters: [EncoderParameters {
                mode: EncoderMode::EncL,
                speed_multiplier: 10,
            }; 8],
            midi_sender,
        };
        slf.draw();
        slf
    }

    pub fn draw(&self) {
        update_leds_state::spawn(LedsState {
            pad_state: self.leds_state_for_channel[self.current_channel as usize - 1],
            control_button_state: self.control_button_leds_state,
        })
        .ok();
    }

    pub fn handle_button_event(&mut self, event: ButtonEvent) {
        let on = event.event == ButtonEventEdge::PosEdge;
        match event.btn {
            // send midi
            Button::Pad(pad) => {
                self.handle_pad_button_event(pad, on);
            }
            Button::Control(ControlButton::Master(new_channel)) => {
                if on {
                    // switch midi channel for pads
                    self.handle_master_button_event(new_channel);
                }
            }
            Button::Control(ControlButton::Parameter(param)) => {
                self.handle_parameter_button_event(param, on);
            }
            Button::Control(ControlButton::Arrow(dir)) => {
                let midi = match on {
                    true => NoteOn::new(8, dir as u8, 127).unwrap().to_bytes(),
                    false => NoteOff::new(8, dir as u8).unwrap().to_bytes(),
                };
                self.midi_sender.try_send(midi).ok();
            }
            Button::Control(ControlButton::Mode(mode)) => {
                let midi = match on {
                    true => NoteOn::new(9, mode as u8, 127).unwrap().to_bytes(),
                    false => NoteOff::new(9, mode as u8).unwrap().to_bytes(),
                };
                self.midi_sender.try_send(midi).ok();
            }
        };
    }

    fn handle_parameter_button_event(&mut self, param: Parameter, on: bool) {
        if on {
            let parameter_off_event = ControlButtonLedEvent {
                btn: ControlButton::Parameter(self.encoder_parameter_type),
                on: false,
            };
            let parameter_on_event = ControlButtonLedEvent {
                btn: ControlButton::Parameter(param),
                on: true,
            };

            self.encoder_parameter_type = param;

            self.control_button_leds_state
                .apply_event(parameter_off_event);
            self.control_button_leds_state
                .apply_event(parameter_on_event);

            self.draw();
        }
    }

    fn handle_pad_button_event(&mut self, pad: Pad, on: bool) {
        let packet = match on {
            true => NoteOn::new(self.current_channel - 1, pad.y * 8 + pad.x, 127)
                .unwrap()
                .to_bytes(),
            false => NoteOff::new(self.current_channel - 1, pad.y * 8 + pad.x)
                .unwrap()
                .to_bytes(),
        };
        self.midi_sender.try_send(packet).ok();
    }

    fn handle_master_button_event(&mut self, new_channel: u8) {
        // Set channel LED state for all Pad banks
        let master_off_event = ControlButtonLedEvent {
            btn: ControlButton::Master(self.current_channel),
            on: false,
        };
        self.control_button_leds_state.apply_event(master_off_event);

        let master_on_event = ControlButtonLedEvent {
            btn: ControlButton::Master(new_channel),
            on: true,
        };
        self.control_button_leds_state.apply_event(master_on_event);

        self.current_channel = new_channel;

        self.draw();
    }

    pub fn handle_midi(&mut self, midi: Midi) {
        let mut led_event = None;
        let mut channel = self.current_channel;

        match midi {
            Midi::NoteOn(note_on) => {
                defmt::info!("{}", note_on);
                match note_on.channel {
                    0..=7 => {
                        let x = note_on.note % 8;
                        let y = note_on.note / 8;

                        let rgb = match note_on.velocity {
                            127 => {
                                // show preset color depending on y, for Ardour compatibility
                                match y {
                                    0 => COLOR_GREEN,
                                    1 => COLOR_YELLOW,
                                    2 => COLOR_RED,
                                    3 => COLOR_PURPLE,
                                    4 => COLOR_AQUA,
                                    5 => COLOR_BLUE,
                                    6 | 7 => COLOR_WHITE,
                                    _ => unreachable!(),
                                }
                            }
                            num => PadColor::from_value(num),
                        };

                        if x < 8 && y < 8 {
                            led_event = Some(LedEvent::Pad(PadLedEvent {
                                pad: Pad { x, y },
                                color: rgb,
                            }));
                            channel = note_on.channel + 1;
                        }
                    }
                    8 if note_on.note < 4 => {
                        led_event = Some(LedEvent::ControlButton(ControlButtonLedEvent {
                            btn: ControlButton::Arrow(DIRECTION_TYPES[note_on.note as usize]),
                            on: note_on.velocity != 0,
                        }));
                    }
                    9 if note_on.note < 4 => {
                        led_event = Some(LedEvent::ControlButton(ControlButtonLedEvent {
                            btn: ControlButton::Mode(MODE_TYPES[note_on.note as usize]),
                            on: note_on.velocity != 0,
                        }));
                    }
                    _ => (), // ignore message
                }
            }
            Midi::NoteOff(note_off) => {
                defmt::info!("{}", note_off);
                match note_off.channel {
                    0..=7 => {
                        let x = note_off.note % 8;
                        let y = note_off.note / 8;

                        if x < 8 && y < 8 {
                            led_event = Some(LedEvent::Pad(PadLedEvent {
                                pad: Pad { x, y },
                                color: COLOR_BLACK,
                            }));
                            channel = note_off.channel + 1;
                        }
                    }
                    8 => {
                        led_event = Some(LedEvent::ControlButton(ControlButtonLedEvent {
                            btn: ControlButton::Arrow(DIRECTION_TYPES[note_off.note as usize]),
                            on: false,
                        }));
                    }
                    9 => {
                        led_event = Some(LedEvent::ControlButton(ControlButtonLedEvent {
                            btn: ControlButton::Mode(MODE_TYPES[note_off.note as usize]),
                            on: false,
                        }));
                    }
                    _ => (), // ignore message
                }
            }
            Midi::CC(cc) => {
                defmt::info!("{}", cc);
                if cc.controller > 0 && cc.controller < 9 {
                    let encoder_type = cc.channel & 0b111;
                    self.abs_encoder_positions[encoder_type as usize][cc.controller as usize - 1] =
                        cc.value;
                } else if cc.controller == 110 && cc.value < 8 {
                    self.handle_button_event(ButtonEvent {
                        btn: Button::Control(ControlButton::Parameter(
                            Parameter::try_from(cc.value).unwrap(),
                        )),
                        event: ButtonEventEdge::PosEdge,
                    });
                } else if cc.controller == 111 && cc.value < 8 {
                    self.handle_button_event(ButtonEvent {
                        btn: Button::Control(ControlButton::Master(cc.value + 1)),
                        event: ButtonEventEdge::PosEdge,
                    });
                } else if cc.controller == 112 {
                    if let Ok(mode) = EncoderMode::try_from(cc.value & 0b111) {
                        let parameter_type = cc.channel & 0b111;
                        self.encoder_parameters[parameter_type as usize] = EncoderParameters {
                            mode,
                            ..self.encoder_parameters[parameter_type as usize]
                        };
                    }
                } else if cc.controller == 113 && cc.value > 0 && cc.value <= 33 {
                    let parameter_type = cc.channel & 0b111;
                    self.encoder_parameters[parameter_type as usize] = EncoderParameters {
                        speed_multiplier: cc.value,
                        ..self.encoder_parameters[parameter_type as usize]
                    };
                }
            }
            _ => {}
        }
        if let Some(le) = led_event {
            defmt::info!("Handle {}", le);
            match le {
                LedEvent::Pad(pad_led_event) => {
                    self.leds_state_for_channel[channel as usize - 1].apply_event(pad_led_event)
                }
                LedEvent::ControlButton(control_button_led_event) => self
                    .control_button_leds_state
                    .apply_event(control_button_led_event),
            }
            self.draw();
        }
    }

    pub fn handle_encoder_event(&mut self, event: EncoderEvent) {
        for (i, diff) in event
            .0
            .into_iter()
            .enumerate()
            .filter(|(_i, diff)| *diff != 0)
        {
            let parameter_type = self.encoder_parameter_type as u8;
            let encoder_parameters = self.encoder_parameters[parameter_type as usize];
            let value = if encoder_parameters.mode != EncoderMode::Abs {
                encoder_parameters.diff_to_value(diff)
            } else {
                let abs_value = self.abs_encoder_positions[parameter_type as usize][i];
                let new_abs_value = encoder_parameters.apply_diff_to_abs_value(diff, abs_value);
                self.abs_encoder_positions[parameter_type as usize][i] = new_abs_value;
                new_abs_value
            };

            let midi = ControlChange::new(parameter_type, i as u8 + 1, value).unwrap();
            let midi_bytes = midi.to_bytes();
            defmt::info!("Create {} from encoders", midi);
            self.midi_sender.try_send(midi_bytes).ok();
        }
    }
}
