#![no_std]
#![no_main]

use core::future::poll_fn;
use core::sync::atomic::{AtomicPtr, AtomicUsize, Ordering};
use core::task::{Poll, Waker};
use cortex_m_rt::entry;
use nrf52832_hal as _;
use nrf52832_hal::pac::interrupt;
use panic_halt as _;

use critical_section as cs;

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

    /// Optional. This is used to save power, this is called when the Monotonic interrupt is
    /// enabled.
    fn enable_timer() {}

    /// Optional. This is used to save power, this is called when the Monotonic interrupt is
    /// disabled.
    fn disable_timer() {}
}

// --------- Timer Queue --------

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

impl<Mono: Monotonic> TimerQueue<Mono> {
    /// Make a new queue.
    pub const fn new() -> Self {
        Self {
            queue: LinkedList::new(),
        }
    }

    /// Call this in the interrupt handler of the hardware timer supporting the `Monotonic`
    ///
    /// Safety: It's always safe to call, but it should only be called from the interrupt of the
    /// monotonic timer.
    pub unsafe fn on_mono_interrupt(&self) {
        let now = Mono::now();

        Mono::clear_compare_flag();

        loop {
            let head = self.queue.peek(|head_value| {
                head_value
                    .map(|val| Some((now >= val.release_at, val.release_at)))
                    .unwrap_or(None)
            });

            if let Some(v) = head {
                match v {
                    (false, instant) => {
                        Mono::enable_timer();
                        Mono::set_compare(instant);

                        // The time for the next instant passed while handling it,
                        // continue dequeueing
                        if Mono::now() >= instant {
                            continue;
                        }

                        break;
                    }
                    (true, _) => {
                        if let Some(waker) = self.queue.pop() {
                            waker.waker.wake();
                        }
                    }
                }
            } else {
                // Queue is empty
                Mono::disable_timer();
            }
        }
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
                marker.store(queue.insert(&mut link), Ordering::Relaxed);
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

pub struct LinkedList<T> {
    head: AtomicPtr<Link<T>>,
}

impl<T> LinkedList<T> {
    pub const fn new() -> Self {
        Self {
            head: AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

pub struct Link<T> {
    val: T,
    next: AtomicPtr<Link<T>>,
}

impl<T: PartialOrd + Clone> LinkedList<T> {
    #[inline(never)]
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
    pub fn pop(&self) -> Option<T> {
        cs::with(|_| {
            // Make sure all previous writes are visible
            core::sync::atomic::fence(Ordering::SeqCst);

            let head = self.head.load(Ordering::Relaxed);

            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a link
            if let Some(head) = unsafe { head.as_ref() } {
                // Move head to the next element
                self.head
                    .store(head.next.load(Ordering::Relaxed), Ordering::Relaxed);

                // We read the value at head
                let head_val = head.val.clone();

                Some(head_val)
            } else {
                None
            }
        })
    }

    #[inline(never)]
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
    pub fn insert(&self, val: &mut Link<T>) -> usize {
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
                return addr;
            };

            // 2. val needs to go in first
            if val.val < head_ref.val {
                // Set current head as next of `val`
                val.next.store(head, Ordering::Relaxed);

                // `val` is now first in the queue
                self.head.store(val, Ordering::Relaxed);

                return addr;
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

                    return addr;
                }

                // Continue searching
                curr = next_link;
                next = next_link.next.load(Ordering::Relaxed);
            }

            // No next, write link to last position in list
            curr.next.store(val, Ordering::Relaxed);

            addr
        })
    }
}

unsafe impl<T> Sync for LinkedList<T> {}

impl<T> Link<T> {
    pub const fn new(val: T) -> Self {
        Self {
            val,
            next: AtomicPtr::new(core::ptr::null_mut()),
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

    LL.pop();
}
