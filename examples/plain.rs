//! examples/plain.rs

//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manaual for an explanation. This is not an issue on the blue pill.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[allow(unused_imports)]
use panic_semihosting;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::gpio::State;
use stm32f1xx_hal::gpio::State::High;
use stm32f1xx_hal::qei::Qei;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    // Acquire the GPIO peripherals
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Declare GPIOs
    let mut led_hs_en_l = gpioa
        .pa2
        .into_push_pull_output_with_state(&mut gpioa.crl, High);
    let mut led_hs_a0 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let mut led_hs_a1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let mut led_hs_a2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

    let mut led_ls_en_l = gpiob
        .pb12
        .into_push_pull_output_with_state(&mut gpiob.crh, High);
    let mut led_ls_dai = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
    let mut led_ls_dck = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
    let mut led_ls_lat = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);

    let mut enc_a0 = gpioc.pc0.into_push_pull_output(&mut gpioc.crl);
    let mut enc_a1 = gpioc.pc1.into_push_pull_output(&mut gpioc.crl);
    let mut enc_a2 = gpioc.pc2.into_push_pull_output(&mut gpioc.crl);

    let mut enc_out_a = gpioa.pa1;
    let mut enc_out_b = gpioa.pa0;

    // Delay
    let mut delay = Delay::new(cp.SYST, clocks);

    // Encoder
    let qei = Qei::tim2(
        dp.TIM2,
        (enc_out_b, enc_out_a),
        &mut afio.mapr,
        &mut rcc.apb1,
    );

    loop {
        let before = qei.count();
        delay.delay_ms(1_000_u16);
        let after = qei.count();

        let elapsed = after.wrapping_sub(before) as i16;

        hprintln!("{}", elapsed).unwrap();
    }

    // LEDs
    /*
    let d: u8 = 1_u8;

    //let mut shift: i32 = 1;
    //let mut bank: i8 = 0;

    delay.delay_us(d);

    // reset DMCs
    for j in 0..31 {
        led_ls_dai.set_low();
        delay.delay_us(d);
        led_ls_dck.set_high();
        delay.delay_us(d);
        led_ls_dck.set_low();
    }
    delay.delay_us(d);
    led_ls_lat.set_high();
    delay.delay_us(d);
    led_ls_lat.set_low();

    let mut bank: i8 = 0;
    let mut shift: u32 = 0;

    //parallel
    loop {
        led_hs_en_l.set_high();
        delay.delay_us(d);

        shift = match bank {
            0 => 0xaaaaaaaa,
            1 => 0x55555555,
            2 => 0xaa55aa55,
            3 => 0x55aa55aa,
            4 => 0x5a5a5a5a,
            5 => 0xa5a5a5a5,
            6 => 0x55555555,
            7 => 0xaaaaaaaa,
            _ => 0x0,
        };

        if bank & 1 << 0 == 0 {
            led_hs_a0.set_low();
        } else {
            led_hs_a0.set_high();
        }
        if bank & 1 << 1 == 0 {
            led_hs_a1.set_low();
        } else {
            led_hs_a1.set_high();
        }
        if bank & 1 << 2 == 0 {
            led_hs_a2.set_low();
        } else {
            led_hs_a2.set_high();
        }

        for i in 0..32 {
            if shift & (1 << i) == 0 {
                led_ls_dai.set_low();
            } else {
                led_ls_dai.set_high();
            }
            delay.delay_us(d);
            led_ls_dck.set_high();
            delay.delay_us(d);
            led_ls_dck.set_low();
        }
        delay.delay_us(d);
        led_ls_lat.set_high();
        delay.delay_us(d);
        led_ls_lat.set_low();
        delay.delay_us(d);
        led_ls_en_l.set_low();

        led_hs_en_l.set_low();
        delay.delay_us(1_u8);
        led_hs_en_l.set_low();
        delay.delay_us(100_u16);

        bank += 1;
        if bank == 8 {
            bank = 0;
        }
    }
    */
    /*
    // sequentiell
    loop {
        led_hs_en_l.set_high();
        delay.delay_us(d);
        if bank & 1 << 0 == 0 {
            led_hs_a0.set_low();
        }
        else {
            led_hs_a0.set_high();
        }
        if bank & 1 << 1 == 0 {
            led_hs_a1.set_low();
        }
        else {
            led_hs_a1.set_high();
        }
        if bank & 1 << 2 == 0 {
            led_hs_a2.set_low();
        }
        else {
            led_hs_a2.set_high();
        }

        for i in 0..32 {
            if shift & (1 << i) == 0 {
                led_ls_dai.set_low();
            }
            else {
                led_ls_dai.set_high();
            }
            delay.delay_us(d);
            led_ls_dck.set_high();
            delay.delay_us(d);
            led_ls_dck.set_low();
        }
        delay.delay_us(d);
        led_ls_lat.set_high();
        delay.delay_us(d);
        led_ls_lat.set_low();
        delay.delay_us(d);
        led_ls_en_l.set_low();

        led_hs_en_l.set_low();
        delay.delay_ms(1_u8);
        for i in 0..25 {
            led_hs_en_l.set_low();
            delay.delay_ms(2_u8);
            led_hs_en_l.set_high();
            delay.delay_ms(2_u8);
        }

        shift = shift << 1;
        if shift == 0 {
            shift = 1;
            bank += 1;
            if bank == 8 {
                bank = 0;
            }
        }

    }
    */
}
