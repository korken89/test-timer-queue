#![no_std]
#![no_main]

use core::{cell::UnsafeCell, ptr::NonNull, sync::atomic::AtomicPtr, task::Waker};
use cortex_m_rt::entry;
use nrf52832_hal as _;
use nrf52832_hal::pac::interrupt;
use panic_halt as _;

use critical_section as cs;

pub struct LinkedList<T> {
    head: UnsafeCell<Option<NonNull<Node<T>>>>,
}

impl<T> LinkedList<T> {
    pub const fn new() -> Self {
        Self {
            head: UnsafeCell::new(None),
        }
    }
}

pub struct Node<T> {
    val: UnsafeCell<T>,
    next: UnsafeCell<Option<NonNull<Node<T>>>>,
}

impl<T: PartialOrd + PartialEq> LinkedList<T> {
    #[inline(never)]
    pub fn pop(&self) -> Option<&mut T> {
        cs::with(|_| {
            // Make sure all previous writes are visible
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            // 1. List is empty, write to head
            let head = unsafe { *self.head.get() };

            if let Some(head) = head {
                let head_val = unsafe { &mut *(head.as_ref()).val.get() };
                unsafe {
                    *self.head.get() = *head.as_ref().next.get();
                };
                Some(head_val)
            } else {
                None
            }
        })
    }

    #[inline(never)]
    pub fn insert(&self, val: &mut Node<T>) {
        // 3 cases to handle

        cs::with(|_| {
            // Make sure all previous writes are visible
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            // 1. List is empty, write to head
            let head = unsafe { *self.head.get() };

            if head.is_none() {
                unsafe { *self.head.get() = Some(val.into()) };
                return;
            }

            let head = unsafe { head.unwrap_unchecked() };

            // 2. val needs to go in first
            let head_val = unsafe { &*(head.as_ref()).val.get() };
            let new_val = unsafe { &*val.val.get() };
            if new_val < head_val {
                // Set current head as next of `val`
                val.next = UnsafeCell::new(unsafe { *self.head.get() });

                unsafe { *self.head.get() = Some(val.into()) };

                return;
            }

            // 3. search list for correct place
            let mut curr = head;
            let mut next = unsafe { *head.as_ref().next.get() };

            loop {
                // ..
                if let Some(next_node) = next {
                    let next_val = unsafe { &*(next_node.as_ref()).val.get() };

                    if new_val < next_val {
                        // Replace
                        val.next = UnsafeCell::new(unsafe { *curr.as_ref().next.get() });
                        unsafe { curr.as_mut().next = UnsafeCell::new(Some(val.into())) };

                        return;
                    } else {
                        curr = next_node;
                        next = unsafe { *next_node.as_ref().next.get() };
                    }
                } else {
                    // No next, write node to last position in list
                    unsafe { curr.as_mut().next = UnsafeCell::new(Some(val.into())) };

                    return;
                }
            }
        });
    }
}

unsafe impl<T> Sync for LinkedList<T> {}

impl<T> Node<T> {
    pub const fn new(val: T) -> Self {
        Self {
            val: UnsafeCell::new(val),
            next: UnsafeCell::new(None),
        }
    }
}

static LL: LinkedList<u32> = LinkedList::new();

use cortex_m::peripheral::{NVIC, SCB};

#[entry]
fn main() -> ! {
    let p = unsafe { nrf52832_hal::pac::CorePeripherals::steal() };

    cs::with(|_| {
        cortex_m::asm::nop();
        cortex_m::asm::nop();
        cortex_m::asm::nop();
    });

    const N: usize = 1 << nrf52832_hal::pac::NVIC_PRIO_BITS + 1;
    let a: [u8; N] = [0; N];

    loop {}
}

pub struct WakerNode {
    waker: Waker,
    instant: u64,
    next: AtomicPtr<WakerNode>,
}

// TODO: More with features
const N_HEADS: usize = 17;

pub struct TimerQueue<const NVIC_NUM_PRIO_BITS: usize> {
    heads: [AtomicPtr<WakerNode>; N_HEADS], // TODO: This should be a list of linked lists
}

impl<const NVIC_NUM_PRIO_BITS: usize> TimerQueue<NVIC_NUM_PRIO_BITS> {
    const _CHECK_BITS: () = assert!(
        (1 << NVIC_NUM_PRIO_BITS) + 1 <= 17,
        "This MCU has too many priority bits"
    );

    #[inline(always)]
    #[must_use]
    fn hw2logical(hardware: u8) -> u8 {
        (1 << NVIC_NUM_PRIO_BITS) - (hardware >> (8 - NVIC_NUM_PRIO_BITS))
    }

    fn get_running_prio() -> u8 {
        let vec_active = unsafe { &*SCB::PTR }.icsr.read() as u32;

        match vec_active {
            0 => {
                // Thread mode
                0
            }
            1..=15 => {
                // Exceptions
                let index = vec_active as usize;
                let prio;

                #[cfg(not(armv6m))]
                {
                    debug_assert!(index >= 4); // Only exceptions with prio are allowed
                    prio = unsafe { (*SCB::PTR).shpr.get_unchecked(index - 4) }.read();
                }

                #[cfg(armv6m)]
                {
                    debug_assert!(index >= 8); // Only exceptions with prio are allowed
                    let shpr = unsafe { (*SCB::PTR).shpr.get_unchecked((index - 8) / 4) }.read();
                    prio = (shpr >> (8 * (index % 4))) & 0x0000_00ff as u8;
                }

                hw2logical(prio, 3)
            }
            16.. => {
                // Irqs
                let irqn = vec_active as usize - 16;

                let prio;

                #[cfg(not(armv6m))]
                {
                    // NOTE(unsafe) atomic read with no side effects
                    prio = unsafe { (*NVIC::PTR).ipr.get_unchecked(irqn) }.read();
                }

                #[cfg(armv6m)]
                {
                    // NOTE(unsafe) atomic read with no side effects
                    let ipr_n = unsafe { (*Self::PTR).ipr.get_unchecked(irqn / 4) }.read();
                    prio = (ipr_n >> ((irqn % 4) * 8)) & 0x0000_00ff as u8;
                }

                hw2logical(prio, 3)
            }
        }
    }
}

#[inline(never)]
fn get_running_prio() -> u8 {
    let vec_active = unsafe { &*SCB::PTR }.icsr.read() as u32;

    match vec_active {
        0 => {
            // Thread mode
            0
        }
        1..=15 => {
            // Exceptions
            let index = vec_active as usize;
            let prio;

            #[cfg(not(armv6m))]
            {
                debug_assert!(index >= 4); // Only exceptions with prio are allowed
                prio = unsafe { (*SCB::PTR).shpr.get_unchecked(index - 4) }.read();
            }

            #[cfg(armv6m)]
            {
                debug_assert!(index >= 8); // Only exceptions with prio are allowed
                let shpr = unsafe { (*SCB::PTR).shpr.get_unchecked((index - 8) / 4) }.read();
                prio = (shpr >> (8 * (index % 4))) & 0x0000_00ff as u8;
            }

            hw2logical(prio, 3)
        }
        16.. => {
            // Irqs
            let irqn = vec_active as usize - 16;

            let prio;

            #[cfg(not(armv6m))]
            {
                // NOTE(unsafe) atomic read with no side effects
                prio = unsafe { (*NVIC::PTR).ipr.get_unchecked(irqn) }.read();
            }

            #[cfg(armv6m)]
            {
                // NOTE(unsafe) atomic read with no side effects
                let ipr_n = unsafe { (*Self::PTR).ipr.get_unchecked(irqn / 4) }.read();
                prio = (ipr_n >> ((irqn % 4) * 8)) & 0x0000_00ff as u8;
            }

            hw2logical(prio, 3)
        }
    }
}

// #[inline(never)]
#[must_use]
pub fn hw2logical(hardware: u8, nvic_prio_bits: usize) -> u8 {
    // hardware = ((1 << nvic_prio_bits) - logical) << (8 - nvic_prio_bits)
    // hardware >> (8 - nvic_prio_bits) = (1 << nvic_prio_bits) - logical
    // logical + hardware >> (8 - nvic_prio_bits) = (1 << nvic_prio_bits)
    // logical = (1 << nvic_prio_bits) - hardware >> (8 - nvic_prio_bits)
    (1 << nvic_prio_bits) - (hardware >> (8 - nvic_prio_bits))
}

static mut X: u8 = 0;
static mut Y: u8 = 0;

#[allow(non_snake_case)]
#[interrupt]
fn TIMER0() {
    unsafe {
        X = get_running_prio();
        core::ptr::read_volatile(&X);
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn TIMER1() {
    unsafe {
        Y = hw2logical(X, 3);

        core::ptr::read_volatile(&X);
        core::ptr::read_volatile(&Y);
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn RADIO() {
    let mut node = Node::new(8);
    LL.insert(&mut node);

    LL.pop();
}
