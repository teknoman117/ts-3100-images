#![no_std]
#![no_main]

#![feature(asm)]

use core::panic::PanicInfo;
use core::fmt::{self, Write};
use core::str::FromStr;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

fn ioread8(addr : u16) -> u8 {
    unsafe {
        let data;
        asm!("in al, dx", in ("dx") addr, out("al") data, options(nomem, nostack));
        data
    }
}

fn iowrite8(addr : u16, data : u8) {
    unsafe {
        asm!("out dx, al", in ("dx") addr, in("al") data, options(nomem, nostack));
    }
}

fn iosetbits8(addr : u16, bits : u8) {
    unsafe {
        asm!(
            "in al, dx",
            "or al, {bits}",
            "out dx, al",
            bits = in (reg_byte) bits,
            in ("dx") addr,
            out ("al") _,
            options(nomem, nostack)
        );
    }
}

fn ioclearbits8(addr : u16, bits : u8) {
    unsafe {
        asm!(
            "in al, dx",
            "and al, {bits}",
            "out dx, al",
            bits = in (reg_byte) !bits,
            in ("dx") addr,
            out ("al") _,
            options(nomem, nostack)
        );
    }
}

fn iowrite16(addr : u16, data : u16) {
    unsafe {
        asm!("out dx, ax", in ("dx") addr, in("ax") data, options(nomem, nostack));
    }
}

/*fn ioread16(addr : u16) -> u16 {
    unsafe {
        let data;
        asm!("in ax, dx", in ("dx") addr, out("ax") data, options(nomem, nostack));
        data
    }
}*/

fn set_led(on : bool) {
    match on {
        false => iosetbits8(0xF862, 0x40u8),
        true => ioclearbits8(0xF862, 0x40u8)
    };
}

fn com2_setbaud(baudrate: u32) {
    let divisor = 50000000u32 / (baudrate * 16u32 * 27u32);

    iosetbits8(0x2FB, 0x80);
    iowrite8(0x2FB, 0x80);
    iowrite8(0x2F8, (divisor & 0xff) as u8);
    iowrite8(0x2F9, ((divisor >> 8) & 0xff) as u8);
    ioclearbits8(0x2FB, 0x80);
}

fn com2_writechar(c: u8) {
    iowrite8(0x2F8, c as u8);
    while (ioread8(0x2FD) & 0x20) == 0 { }
}

fn com2_readchar() -> Result<u8, u8> {
    // wait for something on the input
    let mut status = 0;
    while status == 0 {
        status = ioread8(0x2FD) & 0x8B;
    }

    // return character if it wasn't an error
    match status & 0x8A {
        0 => Ok(ioread8(0x2F8)),
        _ => Err(status)
    }
}

fn com2_readu8() -> Option<u8> {
    let mut buf : [u8; 3] = [0; 3];
    let mut length = 0;

    // fetch a string
    let mut character = Some(0);
    while let Some(_) = character {
        character = match com2_readchar() {
            Ok(c) => match c {
                13 => None,
                _ => {
                    buf[length] = c;
                    length = length + 1;
                    if length < buf.len() {
                        Some(c)
                    } else {
                        None
                    }
                }
            }
            Err(_) => None
        }
    }

    // convert to a u8
    match core::str::from_utf8(&buf[0..length]) {
        Ok(s) => {
            match u8::from_str(s) {
                Ok(v) => Some(v),
                Err(_) => None
            }
        }
        Err(_) => None
    }
}

struct Console {}

impl fmt::Write for Console {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            com2_writechar(c as u8);
        }
        Ok(())
    }
}

// todo, use locking when we fix it
static mut CONSOLE: Console = Console {};

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => ($crate::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! println {
    () => ($crate::print!("\r\n"));
    ($($arg:tt)*) => ($crate::print!("{}\r\n", format_args!($($arg)*)));
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    unsafe {
        CONSOLE.write_fmt(args).unwrap();
    }
}

fn read_rtc(address : u8) -> u8 {
    iowrite8(0x70, address);
    ioread8(0x71)
}

fn write_rtc(address : u8, data : u8) {
    iowrite8(0x70, address);
    iowrite8(0x71, data);
}

fn setbits_rtc(address : u8, bits : u8) {
    iowrite8(0x70, address);
    iosetbits8(0x71, bits);
}

fn clearbits_rtc(address : u8, bits : u8) {
    iowrite8(0x70, address);
    ioclearbits8(0x71, bits);
}

fn map_rtc() {
    /* Chip Select Unit 5 -> RTC -> (IO) 0070 -> 0071  */
    iowrite16(0xF42A, 0x0001);  // CS5ADH
    iowrite16(0xF428, 0xC480);  // CS5ADL
    iowrite16(0xF42E, 0x0000);  // CS5MSKH
    iowrite16(0xF42C, 0x0401);  // CS5MSKL

    // set 24 hour mode and binary mode
    setbits_rtc(0xB, 0b0000_0110);
}

fn print_rtc() {
    // battery status string
    let battery_msg = match read_rtc(0xD) & 0x80 {
        0 => "DEAD",
        _ => "GOOD"
    };

    // lock the clock so it doesn't update
    setbits_rtc(0xB, 0b1000_0000);

    println!("RTC (battery: {}): {}-{}-{} {}:{}:{}", battery_msg,
        read_rtc(0x9), read_rtc(0x8), read_rtc(0x7),
        read_rtc(0x4), read_rtc(0x2), read_rtc(0x0));

    // unlock the clock
    clearbits_rtc(0xB, 0b1000_0000);
}

fn settime_rtc() {
    // lock the clock so it doesn't update
    clearbits_rtc(0xA, 0b0111_0000);
    setbits_rtc(0xB, 0b1000_0000);

    // update clock
    print!("year? ");
    if let Some(year) = com2_readu8() {
        write_rtc(0x9, year);
    }

    print!("month? ");
    if let Some(month) = com2_readu8() {
        write_rtc(0x8, month);
    }

    print!("day? ");
    if let Some(day) = com2_readu8() {
        write_rtc(0x7, day);
    }

    print!("hour? ");
    if let Some(hour) = com2_readu8() {
        write_rtc(0x4, hour);
    }

    print!("minute? ");
    if let Some(minute) = com2_readu8() {
        write_rtc(0x2, minute);
    }

    print!("second? ");
    if let Some(second) = com2_readu8() {
        write_rtc(0x0, second);
    }

    // unlock clock
    clearbits_rtc(0xB, 0b1000_0000);
    setbits_rtc(0xA, 0b0010_0000);
}

#[no_mangle]
pub extern "C" fn main() -> ! {
    // turn off on board led
    set_led(false);

    // map the onboard real time clock
    map_rtc();

    // setup 115200 8n1 with no interrupts
    iowrite8(0x2F9, 0x00);
    com2_setbaud(115200);
    iowrite8(0x2FB, 0x03);
    iowrite8(0x2FC, 0x08);

    // print a string maybe?
    println!("Rust on i386EX Demo!");

    // allow programming the rtc
    println!("set data and time? [y/n]");
    if let Ok(c) = com2_readchar() {
        match c as char {
            'Y' | 'y' => settime_rtc(),
            _ => {}
        }
    }

    // print out the characters we receive on the serial port!
    loop {
        match com2_readchar() {
            Ok(c) => {
                // TODO: uncommenting goes over ROM budget
                //println!("Got a char: \'{}\', hex: 0x{:x}, value: {}", c as char, c, c);
                match c as char {
                    'T' | 't' => print_rtc(),
                    'Y' | 'y' => set_led(true),
                    'N' | 'n' => set_led(false),
                    _ => {}
                }
            },
            Err(e) => println!("Got an error: 0x{:x}", e)
        }
    }
}