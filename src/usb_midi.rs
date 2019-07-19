use usb_device::class_prelude::*;
use usb_device::Result;
use core::borrow::Borrow;

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
    write_next: Option<[u8; 4]>,
    read_buf: [u8; 64],
    write_buf: [u8; 64]
}

impl<'a, B: UsbBus> MidiClass<'a, B> {
    pub fn new(alloc: &UsbBusAllocator<B>) -> MidiClass<'_, B> {
        MidiClass {
            ac_if: alloc.interface(),
            ms_if: alloc.interface(),
            read_ep: alloc.bulk(64),
            write_ep: alloc.bulk(64),
            write_next: None,
            read_buf: [0; 64],
            write_buf: [0; 64]
        }
    }

    pub fn write(&mut self, data: &[u8]) -> Result<usize> {
        match self.write_ep.write(data) {
            Ok(count) => Ok(count),
            Err(UsbError::WouldBlock) => Ok(0),
            e => e
        }
    }

    pub fn write_next(&mut self) -> Result<usize> {
        if let Some(data) = self.write_next {
            match self.write(data.borrow()) {
                Ok(count) if count == data.len() => {
                    self.write_next = None;
                    Ok(count)
                },
                e => e
            }
        }
        else {
            Ok(0)
        }
    }

    pub fn set_next(&mut self, data: [u8; 4]) -> Result<usize> {
        if let None = self.write_next {
            self.write_next = Some(data);
            Ok(4)
        }
        else {
            Err(UsbError::Unsupported)
        }
    }

    pub fn has_next(&mut self) -> bool {
        self.write_next.is_some()
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
