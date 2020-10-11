#![no_std]
#![no_main]

#![feature(asm)]

use core::convert::Infallible;
use core::marker::PhantomData;
use core::panic::PanicInfo;

//use ufmt_write::uWrite;
use ufmt::{uWrite, uwrite, uwriteln};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

// read an 8 bit I/O port
fn ioread8(addr : u16) -> u8 {
    unsafe {
        let data;
        asm!("in al, dx", in ("dx") addr, out("al") data, options(nomem, nostack));
        data
    }
}

// write to an 8 bit I/O port
fn iowrite8(addr : u16, data : u8) {
    unsafe {
        asm!("out dx, al", in ("dx") addr, in("al") data, options(nomem, nostack));
    }
}

// set bits in an 8 bit I/O port
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

// clear bits in an 8 bit I/O port
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

// write to a 16 bit I/O port
fn iowrite16(addr : u16, data : u16) {
    unsafe {
        asm!("out dx, ax", in ("dx") addr, in("ax") data, options(nomem, nostack));
    }
}

// read from a 16 bit I/O port
/*fn ioread16(addr : u16) -> u16 {
    unsafe {
        let data;
        asm!("in ax, dx", in ("dx") addr, out("ax") data, options(nomem, nostack));
        data
    }
}*/

// set the state of the onboard led
fn set_led(on : bool) {
    match on {
        false => iosetbits8(0xF862, 0x40u8),
        true => ioclearbits8(0xF862, 0x40u8)
    };
}

// basic types which represent the base addresses of 16450 serial ports
struct COM1 {}
struct COM2 {}

trait COMBaseAddress {
    fn register(offset: u16) -> u16;
}

impl COMBaseAddress for COM1 {
    fn register(offset : u16) -> u16 {
        0x3f8 + offset
    }
}

impl COMBaseAddress for COM2 {
    fn register(offset : u16) -> u16 {
        0x2f8 + offset
    }
}

// wraps some core functions of a com port
struct COM<Port> {
    phantom: PhantomData<Port>
}

// char iterator over COM port ends when carriage return received.
struct COMPortIterator<'port, Port> {
    port : &'port COM<Port>
}

impl<Port> COM<Port> where Port : COMBaseAddress {
    fn setup(baudrate: u32) -> COM<Port> {
        let divisor = 50000000u32 / (baudrate * 16u32 * 27u32);

        iowrite8(/*0x2F9*/ Port::register(1), 0x00);
        iowrite8(/*0x2FB*/ Port::register(3), 0x80);
        iowrite8(/*0x2F8*/ Port::register(0), (divisor & 0xff) as u8);
        iowrite8(/*0x2F9*/ Port::register(1), ((divisor >> 8) & 0xff) as u8);
        iowrite8(/*0x2FB*/ Port::register(3), 0x03);
        iowrite8(/*0x2FC*/ Port::register(4), 0x08);

        COM::<Port> {
            phantom: PhantomData
        }
    }

    fn writechar(&self, c: u8) {
        iowrite8(/*0x2F8*/ Port::register(0), c as u8);
        while (ioread8(/*0x2FD*/ Port::register(5)) & 0x20) == 0 { }
    }

    fn readchar(&self) -> Result<u8, u8> {
        // wait for something on the input
        let mut status = 0;
        while status == 0 {
            status = ioread8(/*0x2FD*/ Port::register(5)) & 0x8B;
        }

        // return character if it wasn't an error
        match status & 0x8A {
            0 => Ok(ioread8(/*0x2F8*/ Port::register(0))),
            _ => Err(status)
        }
    }

    fn iter(&self) -> COMPortIterator::<Port> {
        COMPortIterator::<Port> {
            port: self
        }
    }

    fn read_numeric(&self, radix: u32) -> Option<u32> {
        self.iter().filter(|c| c.is_digit(radix)).fold(None, |state, c| {
            match c.to_digit(radix) {
                Some(digit) => Some((state.unwrap_or(0u32) * radix) + digit),
                None => state
            }
        })
    }
}

impl<'port, Port> Iterator for COMPortIterator<'port, Port> where Port : COMBaseAddress {
    type Item = char;

    fn next(&mut self) -> Option<char> {
        match self.port.readchar() {
            Ok(c) => {
                match c {
                    13 => None,
                    _ => Some(c as char)
                }
            },
            Err(_) => None
        }
    }
}

impl<Port> uWrite for COM<Port> where Port : COMBaseAddress {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        s.as_bytes().iter().for_each(|b| self.writechar(*b));
        Ok(())
    }
}

// tools to manipulate the DS1687 RTC
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

fn print_rtc<Port>(uart: &mut COM<Port>) where Port : COMBaseAddress {
    // battery status string
    let battery_msg = match read_rtc(0xD) & 0x80 {
        0 => "DEAD",
        _ => "GOOD"
    };

    // lock the clock so it doesn't update
    setbits_rtc(0xB, 0b1000_0000);

    uwriteln!(uart, "RTC (battery: {}): {}-{}-{} {}:{}:{}", battery_msg,
        read_rtc(0x9), read_rtc(0x8), read_rtc(0x7),
        read_rtc(0x4), read_rtc(0x2), read_rtc(0x0)).ok();

    // unlock the clock
    clearbits_rtc(0xB, 0b1000_0000);
}

fn settime_rtc<Port>(uart: &mut COM<Port>) where Port : COMBaseAddress {
    // lock the clock so it doesn't update
    clearbits_rtc(0xA, 0b0111_0000);
    setbits_rtc(0xB, 0b1000_0000);

    // update clock
    uwrite!(uart, "year? ").ok();
    if let Some(year) = uart.read_numeric(10) {
        write_rtc(0x9, year as u8);
    }

    uwrite!(uart, "month? ").ok();
    if let Some(month) = uart.read_numeric(10) {
        write_rtc(0x8, month as u8);
    }

    uwrite!(uart, "day? ").ok();
    if let Some(day) = uart.read_numeric(10) {
        write_rtc(0x7, day as u8);
    }

    uwrite!(uart, "hour? ").ok();
    if let Some(hour) = uart.read_numeric(10) {
        write_rtc(0x4, hour as u8);
    }

    uwrite!(uart, "minute? ").ok();
    if let Some(minute) = uart.read_numeric(10) {
        write_rtc(0x2, minute as u8);
    }

    uwrite!(uart, "second? ").ok();
    if let Some(second) = uart.read_numeric(10) {
        write_rtc(0x0, second as u8);
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
    let mut uart = COM::<COM2>::setup(115200);
    uwriteln!(uart, "Rust on i386EX Demo!").ok();

    // allow programming the rtc
    uwrite!(uart, "set data and time? [y/n]").ok();
    if let Ok(c) = uart.readchar() {
        match c as char {
            'Y' | 'y' => settime_rtc(&mut uart),
            _ => {}
        }
    }

    // print out the characters we receive on the serial port!
    loop {
        match uart.readchar() {
            Ok(c) => {
                // TODO: uncommenting goes over ROM budget
                //uwriteln!(uart, "Got a char: \'{}\', hex: 0x{:x}, value: {}", c as char, c, c).ok();
                match c as char {
                    'T' | 't' => print_rtc(&mut uart),
                    'Y' | 'y' => set_led(true),
                    'N' | 'n' => set_led(false),
                    _ => {}
                }
            },
            Err(e) => uwriteln!(uart, "Got an error: {}", e).unwrap()
        }
    }
}