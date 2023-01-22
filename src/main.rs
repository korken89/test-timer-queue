//! Crate

#![no_std]
#![no_main]
#![deny(missing_docs)]
#![allow(incomplete_features)]
#![feature(async_fn_in_trait)]
#![feature(type_alias_impl_trait)]

pub mod rtic_monotonic;
pub mod systick_monotonic;
pub mod timer_queue;

// -------- Test program ---------

use defmt_rtt as _;
use nrf52832_hal as _;
use panic_probe as _;
use systick_monotonic::{Systick, TimerQueue};

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

make_systick_timer_queue!(MONO, Systick<1_000>);

#[rtic::app(
    device = nrf52832_hal::pac,
    dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3, SWI4_EGU4, SWI5_EGU5],
)]
mod app {
    use super::{Systick, MONO};
    use fugit::ExtU32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::println!("init");

        let systick = Systick::start(cx.core.SYST, 64_000_000);
        MONO.initialize(systick);

        async_task::spawn().ok();

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::println!("idle");

        loop {
            core::hint::spin_loop();
        }
    }

    #[task]
    async fn async_task(_: async_task::Context) {
        loop {
            defmt::println!("async task");
            MONO.delay(1.secs()).await;
        }
    }
}
