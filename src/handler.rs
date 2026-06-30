use crate::{
    hal::{ControlButton, ControlButtonLedEvent, PadLedEvent},
    hardware::{Bank, Leds, PadBank, PadLedsState},
    midi::{MidiEventPacket, MidiMessage, NoteOff, NoteOn},
};

pub struct PadLedHandler {
    current_channel: u8,
    leds_state_for_channel: [PadLedsState; 8],
}

impl PadLedHandler {
    pub fn new(leds: &mut Leds) -> Self {
        let master_led_event = ControlButtonLedEvent {
            btn: ControlButton::Master(1),
            on: true,
        };
        leds.apply_control_button_event(master_led_event);

        Self {
            current_channel: 1,
            leds_state_for_channel: [leds.get_pad_state(); 8],
        }
    }

    pub fn get_current_channel(&self) -> u8 {
        self.current_channel
    }

    pub fn handle_pad_button_event(&self, x: u8, y: u8, on: bool) -> MidiEventPacket {
        // master_channel_leds_state[channel as usize - 1] =
        //     le.apply_to_leds_state(master_channel_leds_state[channel as usize - 1])
        match on {
            true => NoteOn::new(self.current_channel - 1, y * 8 + x, 127)
                .unwrap()
                .to_bytes(),
            false => NoteOff::new(self.current_channel - 1, y * 8 + x)
                .unwrap()
                .to_bytes(),
        }
    }

    pub fn handle_master_button_event(&mut self, new_channel: u8, leds: &mut Leds) {
        // Set channel LED state for all Pad banks
        for pb in PadBank::all() {
            leds.set_bank_slot_values(
                Bank::Pad(pb),
                [
                    self.leds_state_for_channel[new_channel as usize - 1].0[0][pb as usize],
                    self.leds_state_for_channel[new_channel as usize - 1].0[1][pb as usize],
                    self.leds_state_for_channel[new_channel as usize - 1].0[2][pb as usize],
                ],
            );
        }

        let master_off_event = ControlButtonLedEvent {
            btn: ControlButton::Master(self.current_channel),
            on: false,
        };
        let master_on_event = ControlButtonLedEvent {
            btn: ControlButton::Master(new_channel),
            on: true,
        };

        leds.apply_control_button_event(master_off_event);
        leds.apply_control_button_event(master_on_event);

        self.current_channel = new_channel;
    }

    pub fn apply_led_event(&mut self, led_event: PadLedEvent, channel: u8) {
        self.leds_state_for_channel[channel as usize - 1].apply_event(led_event);
    }
}
