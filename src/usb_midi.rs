use core::borrow::Borrow;
use heapless::{consts::*, spsc::Queue};
use usb_device::class_prelude::*;

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
    read_queue: Queue<[u8; 4], U64, u8>,
    write_queue: Queue<[u8; 4], U16, u8>,
    need_zlp: bool,
}

impl<'a, B: UsbBus> MidiClass<'a, B> {
    pub fn new(alloc: &UsbBusAllocator<B>) -> MidiClass<'_, B> {
        MidiClass {
            ac_if: alloc.interface(),
            ms_if: alloc.interface(),
            read_ep: alloc.bulk(64),
            write_ep: alloc.bulk(64),
            read_queue: Queue::u8(),
            write_queue: Queue::u8(),
            need_zlp: false,
        }
    }

    pub fn write(&mut self, data: &[u8]) -> usb_device::Result<usize> {
        if self.need_zlp {
            return Ok(0);
        }

        if data.len() == 64 {
            self.need_zlp = true;
        }

        match self.write_ep.write(data) {
            Ok(count) => Ok(count),
            Err(UsbError::WouldBlock) => Ok(0),
            e => e,
        }
    }

    pub fn read_to_queue(&mut self) -> usb_device::Result<usize> {
        let mut buf = [0_u8; 64];
        let len = match self.read_ep.read(&mut buf) {
            Ok(0) | Err(UsbError::WouldBlock) => return Ok(0),
            Ok(count) => count,
            e => return e,
        };

        let mut message = [0_u8; 4];
        for i in 0..len / 4 {
            message.copy_from_slice(&buf[i * 4..(i * 4) + 4]);
            match self.read_queue.enqueue(message) {
                Ok(_) => (),
                _ => return Err(UsbError::BufferOverflow),
            }
        }
        Ok(len)
    }

    pub fn write_queue_to_host(&mut self) -> usb_device::Result<usize> {
        // TODO: In case of error content of queue is lost. Find better implementation!
        let mut data = [0_u8; 64];
        let mut i = 0;

        while !self.write_queue_is_empty() {
            &data[i * 4..(i * 4) + 4].copy_from_slice(&self.write_queue.dequeue().unwrap()[..]);
            i += 1;
        }

        self.write(data[..4 * i].borrow())
    }

    pub fn enqueue(&mut self, data: [u8; 4]) -> Result<(), [u8; 4]> {
        self.write_queue.enqueue(data)
    }

    pub fn dequeue(&mut self) -> Option<[u8; 4]> {
        self.read_queue.dequeue()
    }

    pub fn write_queue_is_empty(&mut self) -> bool {
        self.write_queue.is_empty()
    }

    pub fn read_queue_is_empty(&mut self) -> bool {
        self.read_queue.is_empty()
    }
}

impl<B: UsbBus> UsbClass<B> for MidiClass<'_, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
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
            &[MS_TYPE_HEADER, 0x00, 0x01, 0x32, 0x00 /*length_total*/],
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

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if self.need_zlp && addr == self.write_ep.address() {
            self.need_zlp = false;
            self.write_ep.write(&[]).ok();
        }
    }
}
