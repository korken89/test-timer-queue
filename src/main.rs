//! Crate

#![no_std]
#![no_main]
#![deny(missing_docs)]
#![feature(async_fn_in_trait)]

use core::cell::UnsafeCell;
use core::future::{poll_fn, Future};
use core::marker::PhantomPinned;
use core::sync::atomic::{AtomicPtr, AtomicU32, AtomicUsize, Ordering};
use core::task::{Poll, Waker};
use cortex_m::peripheral::SYST;
use cortex_m_rt::entry;
use fugit::ExtU32;
use futures_util::{
    future::{select, Either},
    pin_mut,
};
use nrf52832_hal as _;
use nrf52832_hal::pac::interrupt;
use panic_halt as _;

use critical_section as cs;

use embedded_hal_async::delay::DelayUs;

// --------- SysTick example impl ---------

/// Systick implementing `rtic_monotonic::Monotonic` which runs at a
/// settable rate using the `TIMER_HZ` parameter.
pub struct Systick<const TIMER_HZ: u32> {}

impl<const TIMER_HZ: u32> Systick<TIMER_HZ> {
    /// Provide a new `Monotonic` based on SysTick.
    ///
    /// The `sysclk` parameter is the speed at which SysTick runs at. This value should come from
    /// the clock generation function of the used HAL.
    ///
    /// Notice that the actual rate of the timer is a best approximation based on the given
    /// `sysclk` and `TIMER_HZ`.
    pub fn new(mut systick: cortex_m::peripheral::SYST, sysclk: u32) -> Self {
        // + TIMER_HZ / 2 provides round to nearest instead of round to 0.
        // - 1 as the counter range is inclusive [0, reload]
        let reload = (sysclk + TIMER_HZ / 2) / TIMER_HZ - 1;

        assert!(reload <= 0x00ff_ffff);
        assert!(reload > 0);

        systick.disable_counter();
        systick.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        systick.set_reload(reload);
        systick.enable_interrupt();
        systick.enable_counter();

        Systick {}
    }

    fn systick() -> SYST {
        unsafe { core::mem::transmute::<(), SYST>(()) }
    }
}

static SYSTICK_CNT: AtomicU32 = AtomicU32::new(0);

impl<const TIMER_HZ: u32> Monotonic for Systick<TIMER_HZ> {
    type Instant = fugit::TimerInstantU32<TIMER_HZ>;
    type Duration = fugit::TimerDurationU32<TIMER_HZ>;

    const ZERO: Self::Instant = Self::Instant::from_ticks(0);

    fn now() -> Self::Instant {
        Self::Instant::from_ticks(SYSTICK_CNT.load(Ordering::Relaxed))
    }

    fn set_compare(_: Self::Instant) {
        // No need to do something here, we get interrupts anyway.
    }

    fn clear_compare_flag() {
        // NOOP with SysTick interrupt
    }

    fn pend_interrupt() {
        cortex_m::peripheral::SCB::set_pendst();
    }

    fn on_interrupt() {
        if Self::systick().has_wrapped() {
            SYSTICK_CNT.fetch_add(1, Ordering::AcqRel);
        }
    }

    fn enable_timer() {}

    fn disable_timer() {}
}

impl<const TIMER_HZ: u32> DelayUs for TimerQueue<Systick<TIMER_HZ>> {
    type Error = core::convert::Infallible;

    async fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        Ok(self.delay(us.micros()).await)
    }

    async fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        Ok(self.delay(ms.millis()).await)
    }
}

/// Register the Systick interrupt and crate a timer queue with a specific name.
#[macro_export]
macro_rules! make_systick_timer_queue {
    ($timer_queue_name:ident, $systick:ident) => {
        static $timer_queue_name: TimerQueue<$systick> = TimerQueue::new();

        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn SysTick() {
            $timer_queue_name.on_monotonic_interrupt();
        }
    };
}

// --------- Monotonic trait ---------

/// # A monotonic clock / counter definition.
///
/// ## Correctness
///
/// The trait enforces that proper time-math is implemented between `Instant` and `Duration`. This
/// is a requirement on the time library that the user chooses to use.
pub trait Monotonic {
    /// The time at time zero.
    const ZERO: Self::Instant;

    /// The type for instant, defining an instant in time.
    ///
    /// **Note:** In all APIs in RTIC that use instants from this monotonic, this type will be used.
    type Instant: Ord
        + Copy
        + core::ops::Add<Self::Duration, Output = Self::Instant>
        + core::ops::Sub<Self::Duration, Output = Self::Instant>
        + core::ops::Sub<Self::Instant, Output = Self::Duration>;

    /// The type for duration, defining an duration of time.
    ///
    /// **Note:** In all APIs in RTIC that use duration from this monotonic, this type will be used.
    type Duration;

    /// Get the current time.
    fn now() -> Self::Instant;

    /// Set the compare value of the timer interrupt.
    ///
    /// **Note:** This method does not need to handle race conditions of the monotonic, the timer
    /// queue in RTIC checks this.
    fn set_compare(instant: Self::Instant);

    /// Clear the compare interrupt flag.
    fn clear_compare_flag();

    /// Pend the timer's interrupt.
    fn pend_interrupt();

    /// Optional. Runs on interrupt before any timer queue handling.
    fn on_interrupt() {}

    /// Optional. This is used to save power, this is called when the timer queue is not empty.
    fn enable_timer() {}

    /// Optional. This is used to save power, this is called when the timer queue is empty.
    fn disable_timer() {}
}

// --------- Timer Queue --------

/// Holds a waker and at which time instant this waker shall be awoken.
pub struct WaitingWaker<Mono: Monotonic> {
    waker: Waker,
    release_at: Mono::Instant,
}

impl<Mono: Monotonic> Clone for WaitingWaker<Mono> {
    fn clone(&self) -> Self {
        Self {
            waker: self.waker.clone(),
            release_at: self.release_at,
        }
    }
}

impl<Mono: Monotonic> PartialEq for WaitingWaker<Mono> {
    fn eq(&self, other: &Self) -> bool {
        self.release_at == other.release_at
    }
}

impl<Mono: Monotonic> PartialOrd for WaitingWaker<Mono> {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        self.release_at.partial_cmp(&other.release_at)
    }
}

/// A generic timer queue for async executors.
pub struct TimerQueue<Mono: Monotonic> {
    queue: LinkedList<WaitingWaker<Mono>>,
}

/// .
pub struct TimeoutError;

impl<Mono: Monotonic> TimerQueue<Mono> {
    /// Make a new queue.
    pub const fn new() -> Self {
        Self {
            queue: LinkedList::new(),
        }
    }

    /// Call this in the interrupt handler of the hardware timer supporting the `Monotonic`
    ///
    /// Safety: It's always safe to call, but it must only be called from the interrupt of the
    /// monotonic timer for correct operation.
    pub unsafe fn on_monotonic_interrupt(&self) {
        Mono::clear_compare_flag();
        Mono::on_interrupt();

        loop {
            let now = Mono::now();

            let mut release_at = None;
            let head = self.queue.pop_if(|head| {
                release_at = Some(head.release_at);

                now >= head.release_at
            });

            match (head, release_at) {
                (Some(link), _) => {
                    link.waker.wake();
                }
                (None, Some(instant)) => {
                    Mono::enable_timer();
                    Mono::set_compare(instant);

                    if Mono::now() >= instant {
                        // The time for the next instant passed while handling it,
                        // continue dequeueing
                        continue;
                    }

                    break;
                }
                (None, None) => {
                    // Queue is empty
                    Mono::disable_timer();

                    break;
                }
            }
        }
    }

    /// Timeout at a specific time.
    pub async fn timeout_at<F: Future>(
        &self,
        instant: Mono::Instant,
        future: F,
    ) -> Result<F::Output, TimeoutError> {
        let delay = self.delay_until(instant);

        pin_mut!(future);
        pin_mut!(delay);

        match select(future, delay).await {
            Either::Left((r, _)) => Ok(r),
            Either::Right(_) => Err(TimeoutError),
        }
    }

    /// Timeout after a specific duration.
    pub async fn timeout_after<F: Future>(
        &self,
        duration: Mono::Duration,
        future: F,
    ) -> Result<F::Output, TimeoutError> {
        self.timeout_at(Mono::now() + duration, future).await
    }

    /// Delay for some duration of time.
    pub async fn delay(&self, duration: Mono::Duration) {
        let now = Mono::now();

        self.delay_until(now + duration).await;
    }

    /// Delay to some specific time instant.
    pub async fn delay_until(&self, instant: Mono::Instant) {
        let mut first_run = true;
        let queue = &self.queue;
        let mut link = Link::new(WaitingWaker {
            waker: poll_fn(|cx| Poll::Ready(cx.waker().clone())).await,
            release_at: instant,
        });

        let marker = &AtomicUsize::new(0);

        let dropper = OnDrop::new(|| {
            queue.delete(marker.load(Ordering::Relaxed));
        });

        poll_fn(|_| {
            if Mono::now() >= instant {
                return Poll::Ready(());
            }

            if first_run {
                first_run = false;
                let (was_empty, addr) = queue.insert(&mut link);
                marker.store(addr, Ordering::Relaxed);

                if was_empty {
                    // Pend the monotonic handler if the queue was empty to setup the timer.
                    Mono::pend_interrupt();
                }
            }

            Poll::Pending
        })
        .await;

        // Make sure that our link is deleted from the list before we drop this stack
        drop(dropper);
    }
}

struct OnDrop<F: FnOnce()> {
    f: core::mem::MaybeUninit<F>,
}

impl<F: FnOnce()> OnDrop<F> {
    pub fn new(f: F) -> Self {
        Self {
            f: core::mem::MaybeUninit::new(f),
        }
    }

    #[allow(unused)]
    pub fn defuse(self) {
        core::mem::forget(self)
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        unsafe { self.f.as_ptr().read()() }
    }
}

// -------- LinkedList ----------

/// A sorted linked list for the timer queue.
pub struct LinkedList<T> {
    head: AtomicPtr<Link<T>>,
}

impl<T> LinkedList<T> {
    /// Create a new linked list.
    pub const fn new() -> Self {
        Self {
            head: AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

impl<T: PartialOrd + Clone> LinkedList<T> {
    #[inline(never)]
    /// Peek at the first element in the queue.
    pub fn peek<R, F: FnOnce(Option<&T>) -> R>(&self, f: F) -> R {
        cs::with(|_| {
            // Make sure all previous writes are visible
            core::sync::atomic::fence(Ordering::SeqCst);

            let head = self.head.load(Ordering::Relaxed);

            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a link
            f(unsafe { head.as_ref() }.map(|link| &link.val))
        })
    }

    #[inline(never)]
    /// Pop the first element in the queue if the closure returns true.
    pub fn pop_if<F: FnOnce(&T) -> bool>(&self, f: F) -> Option<T> {
        cs::with(|_| {
            // Make sure all previous writes are visible
            core::sync::atomic::fence(Ordering::SeqCst);

            let head = self.head.load(Ordering::Relaxed);

            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a link
            if let Some(head) = unsafe { head.as_ref() } {
                if f(&head.val) {
                    // Move head to the next element
                    self.head
                        .store(head.next.load(Ordering::Relaxed), Ordering::Relaxed);

                    // We read the value at head
                    let head_val = head.val.clone();

                    return Some(head_val);
                }
            }
            None
        })
    }

    #[inline(never)]
    /// Delete a link at an address.
    pub fn delete(&self, addr: usize) {
        cs::with(|_| {
            // Make sure all previous writes are visible
            core::sync::atomic::fence(Ordering::SeqCst);

            let head = self.head.load(Ordering::Relaxed);

            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a link
            let head_ref = if let Some(head_ref) = unsafe { head.as_ref() } {
                head_ref
            } else {
                // 1. List is empty, do nothing
                return;
            };

            if head as *const _ as usize == addr {
                // 2. Replace head with head.next
                self.head
                    .store(head_ref.next.load(Ordering::Relaxed), Ordering::Relaxed);

                return;
            }

            // 3. search list for correct node
            let mut curr = head_ref;
            let mut next = head_ref.next.load(Ordering::Relaxed);

            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a link
            while let Some(next_link) = unsafe { next.as_ref() } {
                // Next is not null

                if next as *const _ as usize == addr {
                    curr.next
                        .store(next_link.next.load(Ordering::Relaxed), Ordering::Relaxed);

                    return;
                }

                // Continue searching
                curr = next_link;
                next = next_link.next.load(Ordering::Relaxed);
            }
        })
    }

    #[inline(never)]
    /// Insert a new link into the linked list.
    /// The return is (was_empty, address), where the address of the link is for use with `delete`.
    pub fn insert(&self, val: &mut Link<T>) -> (bool, usize) {
        cs::with(|_| {
            let addr = val as *const _ as usize;

            // Make sure all previous writes are visible
            core::sync::atomic::fence(Ordering::SeqCst);

            let head = self.head.load(Ordering::Relaxed);

            // 3 cases to handle

            // 1. List is empty, write to head
            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a link
            let head_ref = if let Some(head_ref) = unsafe { head.as_ref() } {
                head_ref
            } else {
                self.head.store(val, Ordering::Relaxed);
                return (true, addr);
            };

            // 2. val needs to go in first
            if val.val < head_ref.val {
                // Set current head as next of `val`
                val.next.store(head, Ordering::Relaxed);

                // `val` is now first in the queue
                self.head.store(val, Ordering::Relaxed);

                return (false, addr);
            }

            // 3. search list for correct place
            let mut curr = head_ref;
            let mut next = head_ref.next.load(Ordering::Relaxed);

            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a link
            while let Some(next_link) = unsafe { next.as_ref() } {
                // Next is not null

                if val.val < next_link.val {
                    // Replace next with `val`
                    val.next.store(next, Ordering::Relaxed);

                    // Insert `val`
                    curr.next.store(val, Ordering::Relaxed);

                    return (false, addr);
                }

                // Continue searching
                curr = next_link;
                next = next_link.next.load(Ordering::Relaxed);
            }

            // No next, write link to last position in list
            curr.next.store(val, Ordering::Relaxed);

            (false, addr)
        })
    }
}

/// A link in the linked list.
pub struct Link<T> {
    val: T,
    next: AtomicPtr<Link<T>>,
    _up: PhantomPinned,
}

impl<T> Link<T> {
    /// Create a new link.
    pub const fn new(val: T) -> Self {
        Self {
            val,
            next: AtomicPtr::new(core::ptr::null_mut()),
            _up: PhantomPinned,
        }
    }
}

static LL: LinkedList<u32> = LinkedList::new();

#[entry]
fn main() -> ! {
    cs::with(|_| {
        cortex_m::asm::nop();
        cortex_m::asm::nop();
        cortex_m::asm::nop();
    });

    loop {}
}

#[allow(non_snake_case)]
#[interrupt]
fn RADIO() {
    let mut link = Link::new(8);
    LL.insert(&mut link);

    LL.pop_if(|_| true);
}
