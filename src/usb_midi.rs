use usb_device::class_prelude::*;
use usb_device::Result;

pub const USB_CLASS_AUDIO: u8 = 0x01;
const AUDIO_SUBCLASS_CONTROL: u8 = 0x01;
const AUDIO_SUBCLASS_MS: u8 = 0x03;

const AC_TYPE_HEADER: u8 = 0x01;
const MS_TYPE_HEADER: u8 = 0x01;
const MS_TYPE_GENERAL: u8 = 0x01;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

const MS_MIDI_IN_JACK: u8 = 0x02;
const MS_MIDI_OUT_JACK: u8 = 0x03;

pub struct MidiClass<'a, B: UsbBus> {
    ac_if: InterfaceNumber,
    ms_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
}

impl<'a, B: UsbBus> MidiClass<'a, B> {
    pub fn new(alloc: &UsbBusAllocator<B>) -> MidiClass<'_, B> {
        MidiClass {
            ac_if: alloc.interface(),
            ms_if: alloc.interface(),
            read_ep: alloc.bulk(64),
            write_ep: alloc.bulk(64),
        }
    }
}

impl<B: UsbBus> UsbClass<B> for MidiClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.interface(self.ac_if, USB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, 0)?;
        writer.write(
            CS_INTERFACE,
            &[
                AC_TYPE_HEADER,
                0x00,
                0x01,
                0x09,
                0x00,
                0x01,
                self.ms_if.into(),
            ],
        )?;

        writer.interface(self.ms_if, USB_CLASS_AUDIO, AUDIO_SUBCLASS_MS, 0)?;
        writer.write(
            CS_INTERFACE,
            &[MS_TYPE_HEADER, 0x01, 0x00, 0x32, 0x00 /*length_total*/],
        )?;
        writer.write(CS_INTERFACE, &[MS_MIDI_IN_JACK, 0x01, 0x01, 0x00])?;
        writer.write(
            CS_INTERFACE,
            &[MS_MIDI_OUT_JACK, 0x01, 0x02, 0x01, 0x00, 0x00, 0x00],
        )?;

        writer.endpoint(&self.read_ep)?;
        writer.write(CS_ENDPOINT, &[MS_TYPE_GENERAL, 0x01, 0x01])?;

        writer.endpoint(&self.write_ep)?;
        writer.write(CS_ENDPOINT, &[MS_TYPE_GENERAL, 0x01, 0x02])?;

        Ok(())
    }
}
