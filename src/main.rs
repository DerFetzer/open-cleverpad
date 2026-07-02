#![no_main]
#![no_std]

use panic_reset as _;
//use panic_halt as _;

mod hal;
mod handler;
mod hardware;
#[allow(static_mut_refs)]
mod log;
mod midi;
mod sequencer;
mod usb_midi;

use rtic_monotonics::stm32::prelude::*;

stm32_tim3_monotonic!(Mono, 1_000_000);

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI1, EXTI2, EXTI3])]
mod app {
    use crate::hal::ButtonEventEdge::{NegEdge, PosEdge};
    use crate::hal::{ButtonEvent, ButtonEventEdge, COLOR_BLACK, COLOR_RED, EncoderEvent};
    use crate::handler::ControllerHandler;
    use crate::hardware::{
        ButtonMatrix, ButtonMatrixPins, EncoderPins, Encoders, LedPins, Leds, LedsState,
        NUM_TIME_SLOTS, PadLedsState,
    };
    use crate::midi::{Midi, MidiEventPacket, SysExBuffer, SysExBufferResponse};
    use crate::sequencer::Sequencer;
    use crate::usb_midi::{MidiClass, MidiEventPacketSender};
    use crate::{Mono, log, usb_midi};
    use cortex_m::peripheral::NVIC;
    use heapless::spsc::{Consumer, Producer, Queue};
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Sender;
    use rtic_sync::make_channel;
    use stm32f1xx_hal::pac::{Peripherals, TIM1, TIM2, TIM3};
    use stm32f1xx_hal::rcc::{self, RccBus};
    use stm32f1xx_hal::timer::{CounterHz, CounterUs, Event};
    use stm32f1xx_hal::{
        // gpio::gpioa::*,
        gpio::*,
        prelude::*,
        rcc::BusTimerClock,
        usb::{Peripheral, UsbBus, UsbBusType},
    };
    use usb_device::bus;
    use usb_device::prelude::*;
    use usbd_serial::SerialPort;

    const LED_BANK_PERIOD: u32 = 1_101;
    const LED_BANK_DIVIDER: u32 = 15;

    // Gamma correction (~2.8)
    // See gamma.py
    const GAMMA_FACTORS: [u32; NUM_TIME_SLOTS] = [1, 4, 10];

    const ENC_CAPTURE_PERIOD: u64 = 223; // ~5kHz
    const ENC_EVAL_PERIOD: u64 = 21_005; // ~50Hz
    const BUTTON_COL_PERIOD: u64 = 11_507; // ~.1kHz

    const VID: u16 = 0x1ACC;
    const PID: u16 = 0x3801;

    #[shared]
    struct Shared {
        leds_state: Option<LedsState>,
        encoder_positions: [i32; 8],
        button_event_p: Producer<'static, ButtonEvent>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        midi: MidiClass<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,
        sequencer: &'static mut Sequencer,
        sequencer_timer_handle: CounterHz<TIM2>,
    }

    #[local]
    struct Local {
        leds: Leds,
        encoders: Encoders,
        button_matrix: ButtonMatrix,
        prev_encoder_positions: [i32; 8],
        prev_button_state: [u8; 11],
        button_event_c: Consumer<'static, ButtonEvent>,
        encoder_event_p: Producer<'static, EncoderEvent>,
        encoder_event_c: Consumer<'static, EncoderEvent>,
        led_timer_handle: CounterUs<TIM1>,
        tx_midi: Sender<'static, MidiEventPacket, 32>,
        // debug_pin_pa9: PA9<Output<PushPull>>,
        // debug_pin_pa10: PA10<Output<PushPull>>,
    }

    #[init(local = [BUTTON_QUEUE: Queue<ButtonEvent, 64> = Queue::new(), ENCODER_QUEUE: Queue<EncoderEvent, 64> = Queue::new(), USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None, SEQUENCER: Sequencer = Sequencer::new()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        assert_eq!(GAMMA_FACTORS.into_iter().sum::<u32>(), LED_BANK_DIVIDER);

        let dp: Peripherals = cx.device;

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = dp.FLASH.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let mut rcc = dp.RCC.freeze(
            rcc::Config::hse(8.MHz()).sysclk(72.MHz()).pclk1(36.MHz()),
            &mut flash.acr,
        );

        assert!(rcc.clocks.usbclk_valid());

        Mono::start(<TIM3 as RccBus>::Bus::timer_clock(&rcc.clocks).raw());

        let mut afio = dp.AFIO.constrain(&mut rcc);

        // Acquire the GPIO peripherals
        let mut gpioa = dp.GPIOA.split(&mut rcc);
        let mut gpiob = dp.GPIOB.split(&mut rcc);
        let mut gpioc = dp.GPIOC.split(&mut rcc);

        // Declare LED GPIOs
        let led_hs_en_l = gpioa
            .pa2
            .into_push_pull_output_with_state(&mut gpioa.crl, PinState::High);
        let led_hs_a0 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let led_hs_a1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let led_hs_a2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

        let led_ls_en_l = gpiob
            .pb12
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::High);
        let led_ls_dai = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
        let led_ls_dck = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        let led_ls_lat = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);

        let led_pins = LedPins {
            hs_en_l: led_hs_en_l,
            hs_a0: led_hs_a0,
            hs_a1: led_hs_a1,
            hs_a2: led_hs_a2,
            ls_en_l: led_ls_en_l,
            ls_dai: led_ls_dai,
            ls_dck: led_ls_dck,
            ls_lat: led_ls_lat,
        };

        // Declare encoder GPIOs
        let enc_a0 = gpioc.pc0.into_push_pull_output(&mut gpioc.crl);
        let enc_a1 = gpioc.pc1.into_push_pull_output(&mut gpioc.crl);
        let enc_a2 = gpioc.pc2.into_push_pull_output(&mut gpioc.crl);

        let enc_out_a = gpioa.pa1;
        let enc_out_b = gpioa.pa0;

        // QEI does not work because of encoder switching
        let encoder_pins = EncoderPins {
            a0: enc_a0,
            a1: enc_a1,
            a2: enc_a2,
            a: enc_out_a,
            b: enc_out_b,
        };

        // Declare button matrix GPIOs

        // Disable JTAG ports
        let (_, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let button_pins = ButtonMatrixPins {
            row1: gpioc.pc8,
            row2: gpioc.pc9,
            row3: gpioc.pc10,
            row4: gpioc.pc11,
            row5: gpioc.pc12,
            row6: gpioc.pc13,
            row7: gpioc.pc14,
            row8: gpioc.pc15,
            col1: gpiob
                .pb0
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col2: gpiob
                .pb1
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col3: gpiob
                .pb2
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col4: pb3.into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col5: pb4.into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col6: gpiob
                .pb5
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col7: gpiob
                .pb6
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col8: gpiob
                .pb7
                .into_open_drain_output_with_state(&mut gpiob.crl, PinState::High),
            col9: gpiob
                .pb8
                .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High),
            col10: gpiob
                .pb9
                .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High),
            col11: gpiob
                .pb10
                .into_open_drain_output_with_state(&mut gpiob.crh, PinState::High),
        };

        // Declare Debug-GPIOs
        let _debug_pa9 = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
        let _debug_pa10 = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);

        // Queues
        let (button_event_p, button_event_c): (
            Producer<'static, ButtonEvent>,
            Consumer<'static, ButtonEvent>,
        ) = cx.local.BUTTON_QUEUE.split();

        let (encoder_event_p, encoder_event_c): (
            Producer<'static, EncoderEvent>,
            Consumer<'static, EncoderEvent>,
        ) = cx.local.ENCODER_QUEUE.split();

        // USB
        let _usb_pullup = gpioa
            .pa8
            .into_push_pull_output_with_state(&mut gpioa.crh, PinState::High);

        let usb_dm = gpioa.pa11;
        let usb_dp = gpioa.pa12;

        let usb = Peripheral {
            usb: dp.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        cx.local.USB_BUS.replace(UsbBus::new(usb));
        let usb_bus = cx.local.USB_BUS.as_ref().unwrap();

        let serial = SerialPort::new(usb_bus);

        let (tx_midi, rx_midi) = make_channel!(MidiEventPacket, 32);
        let midi = usb_midi::MidiClass::new(usb_bus, rx_midi);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .strings(&[StringDescriptors::new(LangID::EN)
                .manufacturer("derfetzer")
                .product("open-cleverpad")])
            .unwrap()
            .composite_with_iads()
            .max_packet_size_0(16)
            .unwrap()
            .max_power(500)
            .unwrap()
            .build();

        // Use timer for LEDs to reduce jitter/flicker
        let mut led_timer_handle = dp.TIM1.counter_us(&mut rcc);
        led_timer_handle
            .start(LED_BANK_PERIOD.micros() / LED_BANK_DIVIDER)
            .unwrap();
        led_timer_handle.listen(Event::Update);

        let sequencer_timer_handle = dp.TIM2.counter_hz(&mut rcc);
        // sequencer_start::spawn(120).unwrap();

        cx.local
            .SEQUENCER
            .init(MidiEventPacketSender::new(tx_midi.clone()));

        enc::spawn().unwrap();
        enc_eval::spawn().unwrap();
        button::spawn().unwrap();

        (
            Shared {
                leds_state: None,
                encoder_positions: [0; 8],
                button_event_p,
                usb_dev,
                midi,
                serial,
                sequencer: cx.local.SEQUENCER,
                sequencer_timer_handle,
            },
            Local {
                leds: Leds::new(led_pins),
                encoders: Encoders::new(encoder_pins),
                button_matrix: ButtonMatrix::new(button_pins),
                prev_encoder_positions: [0; 8],
                prev_button_state: [0; 11],
                button_event_c,
                encoder_event_p,
                encoder_event_c,
                led_timer_handle,
                tx_midi,
                // debug_pin_pa9: debug_pa9,
                // debug_pin_pa10: debug_pa10,
            },
        )
    }

    #[idle(local = [button_event_c, sysex_buffer: SysExBuffer = SysExBuffer::new(), tx_midi, encoder_event_c], shared = [midi, button_event_p, sequencer])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut pad_handler =
            ControllerHandler::new(MidiEventPacketSender::new(cx.local.tx_midi.clone()));
        let sequencer_active = false;

        let sysex_buffer = cx.local.sysex_buffer;

        loop {
            // Handle button events
            if let Some(e) = cx.local.button_event_c.dequeue() {
                defmt::info!("Got button event: {}", e);
                let on = match e.event {
                    ButtonEventEdge::NegEdge => false,
                    ButtonEventEdge::PosEdge => true,
                };
                if sequencer_active {
                    if on {
                        let sequencer = &mut cx.shared.sequencer;

                        sequencer.lock(|sequencer| {
                            sequencer.handle_button_press(e.btn);
                            sequencer.draw();
                        });
                    }
                } else {
                    pad_handler.handle_button_event(e);
                }
            }

            // Handle encoder events
            if let Some(e) = cx.local.encoder_event_c.dequeue() {
                if sequencer_active {
                    // TODO
                } else {
                    pad_handler.handle_encoder_event(e);
                }
            }

            // Handle midi messages
            let (message, has_space_for_packet) = cx
                .shared
                .midi
                .lock(|m| (m.dequeue(), m.read_queue_has_space_for_packet()));

            if let Some(b) = message {
                defmt::info!("Got midi message: {:x}", b);

                let midi = match sysex_buffer.handle_packet(b) {
                    Ok(SysExBufferResponse::Other(packet)) => Midi::from_bytes(packet),
                    Ok(SysExBufferResponse::InSysEx) => {
                        defmt::info!("In SysEx message");
                        continue;
                    }
                    Ok(SysExBufferResponse::SysEx(sysex)) => {
                        // defmt::info!("Got SysEx message");
                        defmt::info!("Got SysEx message: {}", sysex);
                        Midi::SysEx(sysex)
                    }
                    Err(e) => {
                        defmt::warn!("SysEx error: {}", e);
                        continue;
                    }
                };

                if sequencer_active {
                    // TODO
                } else {
                    pad_handler.handle_midi(midi);
                }

                if has_space_for_packet {
                    // There is enough capacity in the queue again.
                    unsafe {
                        NVIC::unmask(stm32f1xx_hal::pac::Interrupt::USB_LP_CAN_RX0);
                        NVIC::unmask(stm32f1xx_hal::pac::Interrupt::USB_HP_CAN_TX);
                    }
                }
            }
        }
    }

    #[task(priority = 1, shared = [leds_state])]
    async fn update_leds_state(mut cx: update_leds_state::Context, leds_state: LedsState) {
        cx.shared.leds_state.lock(|ls| *ls = Some(leds_state));
    }

    #[task(binds = TIM1_UP, priority = 3, shared = [leds_state], local = [leds, led_timer_handle, count: u32 = 0, current_mod: u32 = 1, last_time_slot: usize = 0])]
    fn led_bank(mut cx: led_bank::Context) {
        if (*cx.local.count).is_multiple_of(*cx.local.current_mod) {
            let current_time_slot: usize = cx.local.leds.write_next_bank();

            if current_time_slot != *cx.local.last_time_slot {
                // Reset count and set mod
                *cx.local.count = 0;
                *cx.local.current_mod = GAMMA_FACTORS[current_time_slot];

                // Check for new state
                if let Some(leds_state) = cx.shared.leds_state.lock(|ls| ls.take()) {
                    cx.local.leds.set_leds_state(leds_state);
                }
            }

            *cx.local.last_time_slot = current_time_slot;
        }

        *cx.local.count += 1;
        cx.local.led_timer_handle.clear_interrupt(Event::Update);
    }

    #[task(priority = 1, shared = [sequencer, sequencer_timer_handle])]
    async fn sequencer_stop(mut cx: sequencer_stop::Context) {
        cx.shared
            .sequencer_timer_handle
            .lock(|th| th.unlisten(Event::Update));
        cx.shared.sequencer.lock(|s| s.reset_position());
    }

    #[task(priority = 1, shared = [sequencer, sequencer_timer_handle])]
    async fn sequencer_start(mut cx: sequencer_start::Context, bpm: u16) {
        cx.shared.sequencer.lock(|s| s.reset_position());
        cx.shared.sequencer_timer_handle.lock(|th| {
            th.start((bpm as u32 * 24 / 60).Hz()).unwrap(); // bpm * 24ppgn / 60 seconds/minute
            th.listen(Event::Update);
        });
    }

    #[task(priority = 1, shared = [sequencer])]
    async fn sequencer_step(mut cx: sequencer_step::Context) {
        cx.shared.sequencer.lock(|s| s.step());
    }

    #[task(binds = TIM2, priority = 1, shared = [midi, sequencer, sequencer_timer_handle])]
    fn sequencer_step_internal(mut cx: sequencer_step_internal::Context) {
        cx.shared
            .sequencer_timer_handle
            .lock(|th| th.clear_interrupt(Event::Update));
        sequencer_step::spawn().ok();
    }

    #[task(priority = 1)]
    async fn text_scroll(_cx: text_scroll::Context) {
        use embedded_graphics::mono_font::MonoTextStyle;
        use embedded_graphics::mono_font::ascii::FONT_5X7;
        use embedded_graphics::prelude::*;
        use embedded_graphics::text::Text;

        let text_style = MonoTextStyle::new(&FONT_5X7, COLOR_RED);
        // let text_style = MonoTextStyle::new(&PROFONT_7_POINT, COLOR_RED);
        let text = "Hallo, Maus! Liebe, Liebe! <3";

        let text_width = Text::new(text, Point::new(0, 6), text_style)
            .bounding_box()
            .size
            .width;

        loop {
            let mut pad_state = PadLedsState::default();
            let mut offset = 8;
            while offset > -(text_width as i32) {
                let text = Text::new(text, Point::new(offset, 6), text_style);
                pad_state.clear(COLOR_BLACK).unwrap();
                text.draw(&mut pad_state).unwrap();
                update_leds_state::spawn(LedsState {
                    pad_state,
                    control_button_state: Default::default(),
                })
                .ok();
                offset -= 1;

                Mono::delay(200_u64.millis()).await;
            }
        }
    }

    #[task(priority = 2, local = [encoders], shared = [encoder_positions])]
    async fn enc(mut cx: enc::Context) -> ! {
        loop {
            let baseline = Mono::now();
            let change: bool = cx.local.encoders.read().await;

            if change {
                cx.shared
                    .encoder_positions
                    .lock(|pos| *pos = cx.local.encoders.get_positions());
            }

            Mono::delay_until(baseline + ENC_CAPTURE_PERIOD.micros()).await;
        }
    }

    #[task(priority = 1, local = [prev_encoder_positions, encoder_event_p], shared = [encoder_positions, midi])]
    async fn enc_eval(mut cx: enc_eval::Context) -> ! {
        loop {
            let baseline = Mono::now();
            let new_encoder_positions = cx.shared.encoder_positions.lock(|&mut p| p);
            let encoder_positions: [i32; 8] = *cx.local.prev_encoder_positions;

            // Handle encoder changes
            if new_encoder_positions != encoder_positions {
                let mut encoder_event = EncoderEvent::default();
                for i in 0..8 {
                    let diff: i32 = new_encoder_positions[i] - encoder_positions[i];
                    if diff != 0 {
                        encoder_event.0[i] = diff;
                    }
                }
                cx.local.encoder_event_p.enqueue(encoder_event).ok();
                *cx.local.prev_encoder_positions = new_encoder_positions;
            }

            Mono::delay_until(baseline + ENC_EVAL_PERIOD.micros()).await;
        }
    }

    #[task(priority = 2, local = [button_matrix, prev_button_state], shared = [button_event_p])]
    async fn button(mut cx: button::Context) -> ! {
        loop {
            cx.local.button_matrix.read().await;

            let deb_rows: [u8; 11] = cx.local.button_matrix.get_debounced_rows();

            if deb_rows != *cx.local.prev_button_state {
                for (col, deb_row) in deb_rows.iter().enumerate() {
                    for row in 0..8 {
                        let edge = match (
                            ((cx.local.prev_button_state[col] >> row) & 1),
                            ((deb_row >> row) & 1),
                        ) {
                            (0, 1) => Some(PosEdge),
                            (1, 0) => Some(NegEdge),
                            _ => None,
                        };

                        if let Some(e) = edge {
                            cx.shared.button_event_p.lock(|bep| {
                                bep.enqueue(ButtonEvent::new(row, col as u8, e)).ok();
                            })
                        }
                    }
                }

                *cx.local.prev_button_state = deb_rows;
            }

            Mono::delay(BUTTON_COL_PERIOD.micros()).await;
        }
    }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, midi, serial])]
    fn usb_hp_can_tx(cx: usb_hp_can_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut midi = cx.shared.midi;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut midi, &mut serial).lock(|usb_dev, midi, serial| {
            usb_poll(usb_dev, midi, serial);
        });
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, midi, serial])]
    fn usb_lp_can_rx0(cx: usb_lp_can_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut midi = cx.shared.midi;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut midi, &mut serial).lock(|usb_dev, midi, serial| {
            usb_poll(usb_dev, midi, serial);
        });
    }

    #[task(priority = 1,  shared = [usb_dev, midi, serial])]
    async fn usb_poll_task(cx: usb_poll_task::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut midi = cx.shared.midi;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut midi, &mut serial).lock(|usb_dev, midi, serial| {
            usb_poll(usb_dev, midi, serial);
        });
    }

    fn usb_poll<B: bus::UsbBus>(
        usb_dev: &mut UsbDevice<'static, B>,
        midi: &mut usb_midi::MidiClass<'static, B>,
        serial: &mut SerialPort<'static, B>,
    ) -> bool {
        if !midi.write_queue_is_empty() {
            match midi.write_queue_to_host() {
                Ok(_) => {}
                Err(UsbError::WouldBlock) => {
                    defmt::debug!("Retry counter max reached");
                } // If nothing is connected
                Err(_) => panic!(),
            }
        }

        while let Some(buf) = log::get_queue().dequeue() {
            let mut retry_counter = 0;
            let mut start = 0;
            loop {
                match serial.write(&buf[start..]) {
                    Ok(bytes_written) if buf[start..].len() == bytes_written => break,
                    Ok(bytes_written) => {
                        start += bytes_written;
                        retry_counter = 0;
                    }
                    Err(UsbError::WouldBlock) => {
                        if retry_counter >= 10 {
                            break;
                        } else {
                            retry_counter += 1;
                        }
                    }
                    Err(_) => break,
                }
            }
        }

        if !usb_dev.poll(&mut [serial, midi]) {
            return false;
        }

        let mut serial_buf = [0; 64];
        while serial.read(&mut serial_buf).is_ok() {}

        if !midi.read_queue_has_space_for_packet() {
            // Disable USB interrupts to make clearing of the queue possible.
            NVIC::mask(stm32f1xx_hal::pac::Interrupt::USB_LP_CAN_RX0);
            NVIC::mask(stm32f1xx_hal::pac::Interrupt::USB_HP_CAN_TX);
            false
        } else {
            matches!(midi.read_to_queue(), Ok(len) if len > 0)
        }
    }
}
