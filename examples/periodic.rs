//! examples/periodic.rs

#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_semihosting::hprintln;
use rtfm::Instant;

const PERIOD: u32 = 8_000_000;

// NOTE: does NOT work on QEMU!
#[rtfm::app(device = stm32f1xx_hal::pac)]
const APP: () = {
    #[init(schedule = [foo])]
    fn init() {
        schedule.foo(Instant::now() + PERIOD.cycles()).unwrap();
    }

    #[task(schedule = [foo])]
    fn foo() {
        let now = Instant::now();
        hprintln!("foo(scheduled = {:?}, now = {:?})", scheduled, now).unwrap();

        schedule.foo(scheduled + PERIOD.cycles()).unwrap();
    }

    extern "C" {
        fn EXTI1();
    }
};
