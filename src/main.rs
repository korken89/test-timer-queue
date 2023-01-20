#![no_std]
#![no_main]

use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m_rt::entry;
use nrf52832_hal as _;
use nrf52832_hal::pac::interrupt;
use panic_halt as _;

use critical_section as cs;

pub struct LinkedList<T> {
    head: AtomicPtr<Node<T>>,
}

impl<T> LinkedList<T> {
    pub const fn new() -> Self {
        Self {
            head: AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

pub struct Node<T> {
    val: T,
    next: AtomicPtr<Node<T>>,
}

impl<T: PartialOrd + PartialEq + Clone> LinkedList<T> {
    #[inline(never)]
    pub fn pop(&self) -> Option<T> {
        cs::with(|_| {
            // Make sure all previous writes are visible
            core::sync::atomic::fence(Ordering::SeqCst);

            let head = self.head.load(Ordering::Relaxed);

            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a node
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
    pub fn insert(&self, val: &mut Node<T>) {
        cs::with(|_| {
            // Make sure all previous writes are visible
            core::sync::atomic::fence(Ordering::SeqCst);

            let head = self.head.load(Ordering::Relaxed);

            // 3 cases to handle

            // 1. List is empty, write to head
            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a node
            let head_ref = if let Some(head_ref) = unsafe { head.as_ref() } {
                head_ref
            } else {
                self.head.store(val, Ordering::Relaxed);
                return;
            };

            // 2. val needs to go in first
            if val.val < head_ref.val {
                // Set current head as next of `val`
                val.next.store(head, Ordering::Relaxed);

                // `val` is now first in the queue
                self.head.store(val, Ordering::Relaxed);

                return;
            }

            // 3. search list for correct place
            let mut curr = head_ref;
            let mut next = head_ref.next.load(Ordering::Relaxed);

            // SAFETY: `as_ref` is safe as `insert` requires a valid reference to a node
            while let Some(next_node) = unsafe { next.as_ref() } {
                // Next is not null

                if val.val < next_node.val {
                    // Replace next with `val`
                    val.next.store(next, Ordering::Relaxed);

                    // Insert `val`
                    curr.next.store(val, Ordering::Relaxed);

                    return;
                }

                // Continue searching
                curr = next_node;
                next = next_node.next.load(Ordering::Relaxed);
            }

            // No next, write node to last position in list
            curr.next.store(val, Ordering::Relaxed);
        });
    }
}

unsafe impl<T> Sync for LinkedList<T> {}

impl<T> Node<T> {
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
    let mut node = Node::new(8);
    LL.insert(&mut node);

    LL.pop();
}
