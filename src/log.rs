use crate::app::usb_poll_task;
use core::sync::atomic::{AtomicBool, Ordering};
use critical_section::RestoreState;
use defmt::{Encoder, Logger, global_logger};
use heapless::{Vec, mpmc::MpMcQueue};

const MAX_BYTES: usize = 128;

static TAKEN: AtomicBool = AtomicBool::new(false);
static OVERFLOW: AtomicBool = AtomicBool::new(false);
static BUF: MpMcQueue<Vec<u8, MAX_BYTES>, 64> = MpMcQueue::new();
static mut ENCODER: Encoder = Encoder::new();
static mut CS_RESTORE: RestoreState = RestoreState::invalid();

#[global_logger]
struct BufferLogger;

/// Implementation customized from
/// https://github.com/gauteh/defmt-serial
unsafe impl Logger for BufferLogger {
    fn acquire() {
        let restore = unsafe { critical_section::acquire() };

        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly");
        }

        TAKEN.store(true, Ordering::Relaxed);

        unsafe {
            CS_RESTORE = restore;
        }

        unsafe { ENCODER.start_frame(write_to_queue) }
    }

    unsafe fn release() {
        unsafe {
            ENCODER.end_frame(write_to_queue);
        }
        if OVERFLOW.swap(false, Ordering::Relaxed) {
            while BUF.dequeue().is_some() {}
        }
        TAKEN.store(false, Ordering::Relaxed);

        unsafe {
            let restore = CS_RESTORE;
            critical_section::release(restore)
        };
        usb_poll_task::spawn().ok();
    }

    unsafe fn write(bytes: &[u8]) {
        unsafe {
            ENCODER.write(bytes, write_to_queue);
        }
    }

    unsafe fn flush() {}
}

fn write_to_queue(bytes: &[u8]) {
    if OVERFLOW.load(Ordering::Relaxed) {
        return;
    }

    let res = Vec::from_slice(bytes).map(|v| BUF.enqueue(v));

    if !matches!(res, Ok(Ok(_))) {
        OVERFLOW.store(true, Ordering::Relaxed);
    }
}

pub fn get_queue() -> &'static MpMcQueue<Vec<u8, MAX_BYTES>, 64> {
    &BUF
}
