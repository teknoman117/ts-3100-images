#![no_std]
#![no_main]

#![feature(asm)]

use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[no_mangle]
pub extern "C" fn main() -> ! {
    unsafe {
        asm!(
            "in al, dx",
            "or al, 0x40",
            "out dx, al",
            in("dx") 0xF862,
            out("al") _,
            options(nomem, nostack)
        );
    }

    loop {}
}