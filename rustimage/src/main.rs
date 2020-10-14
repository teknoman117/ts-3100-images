#![no_std]
#![no_main]
#![feature(asm)]
#![feature(alloc_error_handler)]
#![feature(assoc_char_funcs)]

//extern crate alloc;
extern crate num_traits;

//use alloc::alloc::{GlobalAlloc, Layout};
//use alloc::{boxed::Box, rc::Rc, vec, vec::Vec};

use core::convert::Infallible;
use core::marker::PhantomData;
use core::panic::PanicInfo;
//use core::ptr::NonNull;
use core::str::from_utf8;

// iterator imports for in-place collection functions
use core::iter::Map;
//use core::iter::TakeWhile;
//use core::str::SplitAsciiWhitespace;

use num_traits::pow;

//use linked_list_allocator::Heap;

use ufmt::{uWrite, uwrite, uwriteln};

// an allocator
struct HeapWrapper {}

/*#[global_allocator]
static mut ALLOCATOR: HeapWrapper = HeapWrapper {};
static mut HEAP: Heap = Heap::empty();
*/

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // TODO: reset computer
    loop {}
}

/*#[alloc_error_handler]
fn alloc_error_handler(_layout: alloc::alloc::Layout) -> ! {
    // TODO: reset computer
    loop {}
}

unsafe impl GlobalAlloc for HeapWrapper {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        HEAP.allocate_first_fit(layout)
            .ok()
            .map_or(0 as *mut u8, |allocation| allocation.as_ptr())
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        HEAP.deallocate(NonNull::new_unchecked(ptr), layout)
    }
}*/

trait CollectInPlace<'a, T> {
    fn collect_with_slice(&mut self, storage: &'a mut [T]) -> &'a [T];
}

impl<'a, B, I: Iterator, F> CollectInPlace<'a, B> for Map<I, F>
where
    F: FnMut(I::Item) -> B,
{
    fn collect_with_slice(&mut self, storage: &'a mut [B]) -> &'a [B] {
        let mut destination = storage.iter_mut();
        let length = self.fold(0, |length, s| match destination.next() {
            Some(d) => {
                *d = s;
                length + 1
            }
            None => length,
        });
        &storage[0..length]
    }
}

/*impl<'a> CollectInPlace<'a, &str> for SplitAsciiWhitespace<'a>
{
    fn collect_with_slice(&mut self, storage: &'a mut [&str]) -> &'a [&str] {
        let mut destination = storage.iter_mut();
        let length = self.fold(0, |length, s| match destination.next() {
            Some(d) => {
                *d = s;
                length + 1
            }
            None => length,
        });
        &storage[0..length]
    }
}*/

/*impl<'a, I: Iterator, P> CollectInPlace<'a, I::Item> for TakeWhile<I, P>
where
    P: FnMut(&I::Item) -> bool
{
    fn collect_with_slice(&mut self, storage: &'a mut [I::Item]) -> &'a [I::Item] {
        storage
    }
}*/

// source is a standard iterator
// destination is a mutable iterator
// consumes the iterator (will drain)
fn collect_all<'a, I: Iterator>(source: I, storage: &'a mut [I::Item]) -> &'a [I::Item] {
    let mut destination = storage.iter_mut();
    let length = source.fold(0, |length, s| match destination.next() {
        Some(d) => {
            *d = s;
            length + 1
        }
        None => length,
    });

    &storage[0..length]
}

// read an 8 bit I/O port
fn ioread8(addr: u16) -> u8 {
    unsafe {
        let data;
        asm!("in al, dx", in ("dx") addr, out("al") data, options(nomem, nostack));
        data
    }
}

// write to an 8 bit I/O port
fn iowrite8(addr: u16, data: u8) {
    unsafe {
        asm!("out dx, al", in ("dx") addr, in("al") data, options(nomem, nostack));
    }
}

// set bits in an 8 bit I/O port
fn iosetbits8(addr: u16, bits: u8) {
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
fn ioclearbits8(addr: u16, bits: u8) {
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
fn iowrite16(addr: u16, data: u16) {
    unsafe {
        asm!("out dx, ax", in ("dx") addr, in("ax") data, options(nomem, nostack));
    }
}

// read from a 16 bit I/O port
fn ioread16(addr: u16) -> u16 {
    unsafe {
        let data;
        asm!("in ax, dx", in ("dx") addr, out("ax") data, options(nomem, nostack));
        data
    }
}

// write to a 32 bit I/O port
fn iowrite32(addr: u16, data: u32) {
    unsafe {
        asm!("out dx, eax", in ("dx") addr, in("eax") data, options(nomem, nostack));
    }
}

// read from a 32 bit I/O port
fn ioread32(addr: u16) -> u32 {
    unsafe {
        let data;
        asm!("in eax, dx", in ("dx") addr, out("eax") data, options(nomem, nostack));
        data
    }
}

// set the state of the onboard led
fn set_led(on: bool) {
    match on {
        false => iosetbits8(0xF862, 0x40u8),
        true => ioclearbits8(0xF862, 0x40u8),
    };
}

// basic types which represent the base addresses of 16450 serial ports
struct COM1 {}
struct COM2 {}

trait COMBaseAddress {
    fn register(offset: u16) -> u16;
}

impl COMBaseAddress for COM1 {
    fn register(offset: u16) -> u16 {
        0x3f8 + offset
    }
}

impl COMBaseAddress for COM2 {
    fn register(offset: u16) -> u16 {
        0x2f8 + offset
    }
}

// wraps some core functions of a com port
struct COM<Port> {
    phantom: PhantomData<Port>,
}

// byte iterator over COM port ends when carriage return received.
struct COMPortIterator<'port, Port> {
    port: &'port COM<Port>,
}

// char iterator over COM port ends when carriage return received.
struct COMPortCharIterator<'port, Port> {
    port: &'port COM<Port>,
}

impl<Port> COM<Port>
where
    Port: COMBaseAddress,
{
    fn setup(baudrate: u32) -> COM<Port> {
        let divisor = 50000000u32 / (baudrate * 16u32 * 27u32);

        iowrite8(/*0x2F9*/ Port::register(1), 0x00);
        iowrite8(/*0x2FB*/ Port::register(3), 0x80);
        iowrite8(/*0x2F8*/ Port::register(0), (divisor & 0xff) as u8);
        iowrite8(
            /*0x2F9*/ Port::register(1),
            ((divisor >> 8) & 0xff) as u8,
        );
        iowrite8(/*0x2FB*/ Port::register(3), 0x03);
        iowrite8(/*0x2FC*/ Port::register(4), 0x08);

        COM::<Port> {
            phantom: PhantomData,
        }
    }

    fn writechar(&self, c: char) {
        iowrite8(/*0x2F8*/ Port::register(0), c as u8);
        while (ioread8(/*0x2FD*/ Port::register(5)) & 0x20) == 0 {}
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
            _ => Err(status),
        }
    }

    fn iter(&self) -> COMPortIterator<Port> {
        COMPortIterator::<Port> { port: self }
    }

    /*fn char_iter(&self) -> COMPortCharIterator<Port> {
        COMPortCharIterator::<Port> { port: self }
    }

    fn read_numeric(&self, radix: u32) -> Option<u32> {
        self.char_iter()
            .skip_while(|c| !c.is_digit(radix) && *c != '\r' && *c != '\n')
            .take_while(|c| *c != '\r' && *c != '\n')
            .filter(|c| c.is_digit(radix))
            .fold(None, |state, c| match c.to_digit(radix) {
                Some(digit) => Some((state.unwrap_or(0u32) * radix) + digit),
                None => state,
            })
    }*/

    fn read_string<'a>(&self, storage: &'a mut [u8]) -> Option<&'a str> {
        let line = self.iter().take_while(|c| *c != 13);
        let data = collect_all(line, storage);
        if data.len() > 0 {
            match from_utf8(data) {
                Ok(message) => Some(message),
                Err(_) => None,
            }
        } else {
            None
        }
    }
}

impl<'port, Port> Iterator for COMPortIterator<'port, Port>
where
    Port: COMBaseAddress,
{
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        match self.port.readchar() {
            Ok(c) => Some(c),
            Err(_) => self.next(),
        }
    }
}

impl<'port, Port> Iterator for COMPortCharIterator<'port, Port>
where
    Port: COMBaseAddress,
{
    type Item = char;

    fn next(&mut self) -> Option<char> {
        match self.port.readchar() {
            Ok(c) => Some(c as char),
            Err(_) => self.next(),
        }
    }
}

impl<Port> uWrite for COM<Port>
where
    Port: COMBaseAddress,
{
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        s.chars().for_each(|c| self.writechar(c));
        Ok(())
    }
}

// commands that the ROM can perform
enum Command<'a> {
    None,
    Execute {
        address: usize,
    },
    DumpMemory {
        address: usize,
        size: usize,
    },
    WriteMemory {
        address: usize,
        data: &'a [u8],
    },
    IOWrite8 {
        address: u16,
        data: u8,
    },
    IOWrite16 {
        address: u16,
        data: u16,
    },
    IOWrite32 {
        address: u16,
        data: u32,
    },
    IORead8 {
        address: u16,
    },
    IORead16 {
        address: u16,
    },
    IORead32 {
        address: u16,
    },
    RTCRead,
    RTCWrite {
        century: u8,
        year: u8,
        month: u8,
        day: u8,
        hour: u8,
        minute: u8,
        second: u8,
    },
    CMOSRead {
        address: usize,
        size: usize,
    },
    CMOSWrite {
        address: usize,
        data: &'a [u8],
    },
    FlashErase {
        sector: u8,
    },
    FlashWrite {
        flash_address: usize,
        data_address: usize,
        size: usize,
    },
}

struct CommandParser<Port> {
    buffer: [u8; 256],
    port: COM<Port>,
}

impl<Port> CommandParser<Port>
where
    Port: COMBaseAddress,
{
    fn new(port: COM<Port>) -> CommandParser<Port> {
        CommandParser {
            buffer: [0u8; 256],
            port: port,
        }
    }

    fn next_command(&mut self) -> Command {
        // place the prompt
        uwrite!(self.port, "none:/> ").ok();

        // read a string from the uart
        let mut storage = [0u8; 256];
        if let Some(message) = self.port.read_string(&mut storage) {
            // collect all of the command components
            let mut slices = [""; 18];
            let t = collect_all(message.split_whitespace(), &mut slices);

            match t[0] {
                "ex" => {
                    if t.len() >= 2 {
                        Command::Execute {
                            address: usize::from_str_radix(t[1], 16).unwrap_or(0),
                        }
                    } else {
                        Command::None
                    }
                }
                "dm" => {
                    if t.len() >= 3 {
                        Command::DumpMemory {
                            address: usize::from_str_radix(t[1], 16).unwrap_or(0),
                            size: usize::from_str_radix(t[2], 16).unwrap_or(0),
                        }
                    } else {
                        Command::None
                    }
                }
                "wm" => {
                    if t.len() >= 2 {
                        let data = &t[2..t.len()]
                            .iter()
                            .map(|s| u8::from_str_radix(s, 16).unwrap_or(0u8))
                            .collect_with_slice(&mut self.buffer);
                        Command::WriteMemory {
                            address: usize::from_str_radix(t[1], 16).unwrap_or(0),
                            data: data,
                        }
                    } else {
                        Command::None
                    }
                }
                "iw" => {
                    if t.len() >= 4 {
                        let address = u16::from_str_radix(t[2], 16).unwrap_or(0);
                        match t[1] {
                            "b" => Command::IOWrite8 {
                                address: address,
                                data: u8::from_str_radix(t[3], 16).unwrap_or(0),
                            },
                            "w" => Command::IOWrite16 {
                                address: address,
                                data: u16::from_str_radix(t[3], 16).unwrap_or(0),
                            },
                            "l" => Command::IOWrite32 {
                                address: address,
                                data: u32::from_str_radix(t[3], 16).unwrap_or(0),
                            },
                            _ => Command::None,
                        }
                    } else {
                        Command::None
                    }
                }
                "ir" => {
                    if t.len() >= 3 {
                        let address = u16::from_str_radix(t[2], 16).unwrap_or(0);
                        match t[1] {
                            "b" => Command::IORead8 { address: address },
                            "w" => Command::IORead16 { address: address },
                            "l" => Command::IORead32 { address: address },
                            _ => Command::None,
                        }
                    } else {
                        Command::None
                    }
                }
                "tg" => Command::RTCRead,
                "ts" => {
                    if t.len() >= 7 {
                        let year = u16::from_str_radix(t[1], 10).unwrap_or(0);
                        Command::RTCWrite {
                            century: (year / 100) as u8,
                            year: (year % 100) as u8,
                            month: u8::from_str_radix(t[2], 10).unwrap_or(0),
                            day: u8::from_str_radix(t[3], 10).unwrap_or(0),
                            hour: u8::from_str_radix(t[4], 10).unwrap_or(0),
                            minute: u8::from_str_radix(t[5], 10).unwrap_or(0),
                            second: u8::from_str_radix(t[6], 10).unwrap_or(0),
                        }
                    } else {
                        Command::None
                    }
                }
                "cr" => {
                    if t.len() >= 3 {
                        Command::CMOSRead {
                            address: usize::from_str_radix(t[1], 16).unwrap_or(0),
                            size: usize::from_str_radix(t[2], 16).unwrap_or(0),
                        }
                    } else {
                        Command::None
                    }
                }
                "cw" => {
                    if t.len() >= 2 {
                        let data = &t[2..t.len()]
                            .iter()
                            .map(|s| u8::from_str_radix(s, 16).unwrap_or(0u8))
                            .collect_with_slice(&mut self.buffer);
                        Command::CMOSWrite {
                            address: usize::from_str_radix(t[1], 16).unwrap_or(0),
                            data: data,
                        }
                    } else {
                        Command::None
                    }
                }
                "fe" => {
                    if t.len() >= 2 {
                        Command::FlashErase {
                            sector: u8::from_str_radix(t[1], 16).unwrap_or(0),
                        }
                    } else {
                        Command::None
                    }
                }
                "fw" => {
                    if t.len() >= 4 {
                        Command::FlashWrite {
                            flash_address: usize::from_str_radix(t[1], 16).unwrap_or(0),
                            data_address: usize::from_str_radix(t[2], 16).unwrap_or(0),
                            size: usize::from_str_radix(t[3], 16).unwrap_or(0),
                        }
                    } else {
                        Command::None
                    }
                }
                _ => Command::None,
            }
        } else {
            Command::None
        }
    }
}

// tools to manipulate the DS1687 RTC
fn read_rtc(address: u8) -> u8 {
    iowrite8(0x70, address);
    ioread8(0x71)
}

fn write_rtc(address: u8, data: u8) {
    iowrite8(0x70, address);
    iowrite8(0x71, data);
}

fn setbits_rtc(address: u8, bits: u8) {
    iowrite8(0x70, address);
    iosetbits8(0x71, bits);
}

fn clearbits_rtc(address: u8, bits: u8) {
    iowrite8(0x70, address);
    ioclearbits8(0x71, bits);
}

fn map_rtc() {
    /* Chip Select Unit 5 -> RTC -> (IO) 0070 -> 0071  */
    iowrite16(0xF42A, 0x0001); // CS5ADH
    iowrite16(0xF428, 0xC480); // CS5ADL
    iowrite16(0xF42E, 0x0000); // CS5MSKH
    iowrite16(0xF42C, 0x0401); // CS5MSKL

    // set 24 hour mode and binary mode
    setbits_rtc(0xB, 0b0000_0110);
}

fn display_hex<T: uWrite>(port: &mut T, value: usize, digits: usize) {
    (0usize..digits).rev().fold(value, |rem, digit| {
        let place = pow(16usize, digit);
        port.write_char(char::from_digit((rem / place) as u32, 16).unwrap_or(' '))
            .ok();
        rem % place
    });
}

fn display_slice_as_hex<T: uWrite>(port: &mut T, data: &[u8], address: usize) {
    // display each row of the hex dump
    (0..data.len()).step_by(16).for_each(|offset| {
        let size = core::cmp::min(16, data.len() - offset);
        let row = &data[offset..(offset + size)];

        // display the base address of the row
        // display_hex could be solved by implementing it in ufmt
        display_hex(port, address + offset, 8);
        uwrite!(port, ":  ").ok();

        // display each hex character on the row
        (0..16).for_each(|i| {
            if i < row.len() {
                display_hex(port, row[i] as usize, 2);
                uwrite!(port, " ").ok();
            } else {
                uwrite!(port, "   ").ok();
            }
        });

        // display the ascii conversion
        let mut ascii = [0u8; 16];
        row.iter()
            .map(|b| {
                let c = *b as char;
                if !c.is_ascii_control()
                    && (c.is_ascii_alphanumeric()
                        || c.is_ascii_whitespace()
                        || c.is_ascii_punctuation())
                {
                    *b
                } else {
                    '.' as u8
                }
            })
            .collect_with_slice(&mut ascii);
        uwriteln!(port, " {}", from_utf8(&ascii).unwrap_or("")).ok();
    });
}

#[no_mangle]
pub extern "C" fn main() -> ! {
    // initialize the heap
    /*unsafe {
        HEAP.init(0x10000, 0x10000);
    }*/

    // turn off on board led
    set_led(false);

    // map the onboard real time clock
    map_rtc();

    // setup 115200 8n1 with no interrupts
    let mut uart = COM::<COM2>::setup(115200);
    uwriteln!(uart, "386EX Monitor").ok();
    uwriteln!(
        uart,
        "Copyright (C) 2020 Nathaniel R. Lewis <linux.robotdude@gmail.com>"
    )
    .ok();
    uwriteln!(uart, "").ok();

    // command parser utility
    let mut parser = CommandParser::new(uart);
    loop {
        match parser.next_command() {
            Command::Execute { address } => {
                uwriteln!(parser.port, "Execute is not implemented yet").ok();
            }

            Command::DumpMemory { address, size } => {
                let slice = unsafe { core::slice::from_raw_parts(address as *const u8, size) };
                display_slice_as_hex(&mut parser.port, slice, address);
            }

            Command::WriteMemory { address, data } => {
                let slice =
                    unsafe { core::slice::from_raw_parts_mut(address as *mut u8, data.len()) };
                slice.clone_from_slice(&data);
            }

            Command::IOWrite8 { address, data } => {
                iowrite8(address, data);
            }

            Command::IOWrite16 { address, data } => {
                iowrite16(address, data);
            }

            Command::IOWrite32 { address, data } => {
                iowrite32(address, data);
            }

            Command::IORead8 { address } => {
                uwrite!(parser.port, "inb (").ok();
                display_hex(&mut parser.port, address as usize, 4);
                uwrite!(parser.port, ") = ").ok();
                display_hex(&mut parser.port, ioread8(address) as usize, 2);
                uwriteln!(parser.port, "").ok();
            }

            Command::IORead16 { address } => {
                uwrite!(parser.port, "inw (").ok();
                display_hex(&mut parser.port, address as usize, 4);
                uwrite!(parser.port, ") = ").ok();
                display_hex(&mut parser.port, ioread16(address) as usize, 2);
                uwriteln!(parser.port, "").ok();
            }

            Command::IORead32 { address } => {
                uwrite!(parser.port, "inl (").ok();
                display_hex(&mut parser.port, address as usize, 4);
                uwrite!(parser.port, ") = ").ok();
                display_hex(&mut parser.port, ioread32(address) as usize, 2);
                uwriteln!(parser.port, "").ok();
            }

            Command::RTCRead => {
                // battery status string
                let battery_msg = match read_rtc(0xD) & 0x80 {
                    0 => "DEAD",
                    _ => "GOOD",
                };

                // lock the clock so it doesn't update
                setbits_rtc(0xB, 0b1000_0000);

                uwriteln!(
                    parser.port,
                    "Date: {}{}-{}-{} {}:{}:{} (CMOS Battery {})",
                    "", // TODO: century
                    read_rtc(0x9),
                    read_rtc(0x8),
                    read_rtc(0x7),
                    read_rtc(0x4),
                    read_rtc(0x2),
                    read_rtc(0x0),
                    battery_msg
                )
                .ok();

                // unlock the clock
                clearbits_rtc(0xB, 0b1000_0000);
            }

            Command::RTCWrite {
                century,
                year,
                month,
                day,
                hour,
                minute,
                second,
            } => {
                // lock the clock so it doesn't update
                clearbits_rtc(0xA, 0b0111_0000);
                setbits_rtc(0xB, 0b1000_0000);

                // update clock
                // TODO: century
                write_rtc(0x9, year);
                write_rtc(0x8, month);
                write_rtc(0x7, day);
                write_rtc(0x4, hour);
                write_rtc(0x2, minute);
                write_rtc(0x0, second);

                // unlock clock
                clearbits_rtc(0xB, 0b1000_0000);
                setbits_rtc(0xA, 0b0010_0000);
            }

            Command::CMOSRead { address, size } => {
                let last_address = core::cmp::min(address + size, 114);
                let mut data = [0u8; 114];
                (address..last_address).for_each(|x| {
                    data[x] = read_rtc((x as u8) + 14);
                });
                display_slice_as_hex(&mut parser.port, &data[address..last_address], address);
            }

            Command::CMOSWrite { address, data } => {
                let last_address = core::cmp::min(address + data.len(), 114);
                (address..last_address).for_each(|x| {
                    write_rtc((x as u8) + 14, data[x]);
                });
            }

            Command::FlashErase { sector } => {
                uwriteln!(parser.port, "Flash Erase is not implemented yet").unwrap();
            }

            Command::FlashWrite {
                flash_address,
                data_address,
                size,
            } => {
                uwriteln!(parser.port, "Flash Write is not implemented yet").unwrap();
            }

            _ => uwriteln!(parser.port, "Unknown command").unwrap(),
        }
    }
}
