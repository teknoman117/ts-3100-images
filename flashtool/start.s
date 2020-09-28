# Copyright 2020 Nathaniel R. Lewis
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

.section .text_realmode, "ax"
.intel_syntax noprefix
.code16

start:
    # Ensure Interrupts are disabled
    cli
    cld

    # Setup segment registers
    mov ax, cs
    mov ds, ax
    mov es, ax

    # Setup a stack (1 KiB, after IVT)
    xor ax, ax
    mov ss, ax
    mov sp, 0x1400
    mov bp, sp

    # Enable "Extended I/O Space" -> in short, the 386EX internal peripherals
    mov ax, 0x8000
    out 0x23, al
    xchg al, ah
    out 0x22, al
    out 0x22, ax

    # Map all I/O in both DOS and Extended regions
    xor ax, ax
    out 0x22, al

    # Disable watchdog timer - otherwise, the CPU will reset
    # TODO: is there an interrupt when this is about to expire?
    #       that would allow us to use it correctly.
    # NOTE: the watchdog pin is connected to one of the chipset
    #       CPLDs. It's probably issuing the reset. Can we use
    #       the software watchdog without triggering that pin?
    mov dx, 0xF4CA
    mov al, 0x01
    out dx, al

    # ------------------ SETUP I/O PORTS ------------------
    
    # set known state for all pins
    # outputs: bit is strongly driven onto bus
    # inputs / open drain output:
    #     0 -> pull to ground
    #     1 -> high impedance

    # P1LTC
    mov dx, 0xF862
    mov al, 0x00
    out dx, al

    # P2LTC
    mov dx, 0xF86A
    mov al, 0x1F
    out dx, al

    # P3LTC
    mov dx, 0xF872
    mov al, 0x44
    out dx, al

    # set direction
    # per bit:
    #     0 -> output
    #     1 -> input / open drain output
    
    # P1DIR
    mov dx, 0xF864
    mov al, 0xBD
    out dx, al

    # P2DIR
    mov dx, 0xF86C
    mov al, 0x10
    out dx, al

    # P3DIR
    mov dx, 0xF874
    mov al, 0x44
    out dx, al

    # set configuration
    # per bit: 0 - I/O mode, 1 - peripheral mode

    # P1CFG
    # All PORT1 pins are I/O mode
    # P1.6 -> LED (inverted)
    # P1.4 -> JP2
    # P1.1 -> COM1 RTS (on I/O header)
    mov dx, 0xF820
    mov al, 0x00
    # connect RTS to COM1
    #mov al, 0x02
    out dx, al

    # P2CFG
    # All PORT2 pins are Peripheral mode
    # P2.0 -> P2.4 are Chip Select Units 0 -> 4
    # P2.5 -> P2.6 are COM1 RX and TX
    # P2.7 -> COM1 CTS1, grounded inside PCB
    mov dx, 0xF822
    mov al, 0xFF
    out dx, al

    # P3CFG
    # P3.0 -> peripheral, IRQ4
    # P3.1 -> peripheral, IRQ3
    # P3.2 -> I/0 (IRQ1 if peripheral)
    # P3.3 -> peripheral, IRQ5
    # P3.4 -> peripheral, IRQ6
    # P3.5 -> peripheral, IRQ7
    # P3.6 -> I/O
    # P3.7 -> peripheral, UART common clock
    mov dx, 0xF824
    mov al, 0xBB
    out dx, al

    # PINCFG
    # page 306 (11-17) of 386 manual (select serial ports)
    mov dx, 0xF826
    mov al, 0x3f
    out dx, al

    # Enable A20
    in al, 0x92
    or al, 0x2
    out 0x92, al

    # ------------------ SETUP CLOCKING ------------------

    # divide clock by 0x19 (20) to produce PSCLK
    mov dx, 0xF804
    mov ax, 0x0012
    out dx, ax

    # ------------------ SETUP TIMER ------------------

    # TODO TS-3100 never sets this

    # 386EX specific register (defaults to 0x00)
    # TMRCFG.0 -> 0 (CLKIN0 mux select PSCLK)
    # TMRCFG.1 -> 0 (TMRGATE0 low)
    # TMRCFG.2 -> 0 (CLKIN1 mux select PSCLK)
    # TMRCFG.3 -> 0 (TMRGATE1 low)
    # TMRCFG.4 -> 1 (CLKIN2 mux select TMRCLK2)
    # TMRCFG.5 -> 0 (TMRGATE2 low)
    # TMRCFG.6 -> 1 (manual TMRGATEx control)
    # TMRCFG.7 -> 1 (disable clock input)
    # mov dx, 0xF834
    # mov al, 0xD0
    # out dx, al

    # ------------------ SERIAL PORT SETUP ------------------

    # 386EX specific register
    # SIOCFG.0 -> 0 (COM1 baud generator uses COMCLK)
    # SIOCFG.1 -> 0 (COM2 baud generator uses COMCLK)
    # SIOCFG.2 -> 0 (SSIO baud generator uses PSCLK)
    # SIOCFG.3,4,5 -> 0
    # SIOCFG.6 -> 1 (COM1 modem pins connected internally)
    # SIOCFG.7 -> 0 (COM2 modem pins connected externally)
    mov dx, 0xF836
    mov al, 0x40
    # connect COM1 modem pins externally
    # mov al, 0x00
    out dx, al

    # ------------------ SETUP COM1 ------------------

    # base = 0x3f8
    # disable interrupts
    mov dx, 0x3f9
    mov al, 0x00
    out dx, al

    # enable DLAB
    mov dx, 0x3fb
    mov al, 0x80
    out dx, al

    # set divisor = 12 (9600 baud) (have to do 2 8-bit cycles)
    mov dx, 0x3f8
    mov al, 0x0c
    out dx, al
    mov dx, 0x3f9
    mov al, 0x00
    out dx, al

    # disable DLAB, set 8n1
    mov dx, 0x3fb
    mov al, 0x03
    out dx, al

    # select interrupts
    mov dx, 0x3fc
    mov al, 0x08
    out dx, al

    # ------------------ SETUP COM2 ------------------

    # base = 0x2f8
    # disable interrupts
    mov dx, 0x2f9
    mov al, 0x00
    out dx, al

    # enable DLAB
    mov dx, 0x2fb
    mov al, 0x80
    out dx, al

    # set divisor = 1 (115200 baud) (have to do 2 8-bit cycles)
    mov dx, 0x2f8
    mov al, 0x01
    out dx, al
    mov dx, 0x2f9
    mov al, 0x00
    out dx, al

    # disable DLAB, set 8n1
    mov dx, 0x2fb
    mov al, 0x03
    out dx, al

    # select interrupts
    mov dx, 0x2fc
    mov al, 0x08
    out dx, al

    # ------------------ SETUP INTERRUPT CONTROLLER ------------------

    # 386EX specific register
    # INTCFG.0 -> 1 (connect external IRQ8)
    # INTCFG.1 -> 1 (connect external IRQ9)
    # INTCFG.2 -> 0 (disconnect external IRQ12/13, disable IRQ13 internally)
    # INTCFG.3 -> 0 (disconnect external IRQ14, disable internally)
    # INTCFG.4 -> 0 (connect DMAINT to IRQ12)
    # INTCFG.5 -> 1 (connect IRQ4 to MCR0 mux)
    # INTCFG.6 -> 1 (connect IRQ3 to MCR1 mux)
    # INTCFG.7 -> 0 (disable external interrupt controllers)
    mov dx, 0xF832
    # mov al, 0x63
    mov al, 0x7f
    out dx, al

    # ICW1 (init mode, edge triggered interrupts) for both PICs
    mov al, 0x11
    out 0x20, al
    out 0xA0, al

    # ICW2 (base vector -> 0x20) for master PIC (IRQ0->7)
    mov al, 0x20
    out 0x21, al

    # ICW2 (base vector -> 0x28) for slave PIC (IRQ8->15)
    mov al, 0x28
    out 0xA1, al

    # ICW3 (slave connections -> 0x4 (IR2 is slave)) for master PIC
    mov al, 0x04
    out 0x21, al

    # ICW3 (slave identity -> 0x2) for slave PIC
    mov al, 0x02
    out 0xA1, al

    # ICW4 (x86 mode -> 0x01) for both PICs
    mov al, 0x01
    out 0x21, al
    out 0xA1, al

    # mask all interrupts for now
    mov al, 0xff
    out 0x21, al
    out 0xA1, al

    # write 0x20 to 0xA0 then 0xA0 for EOI

    # ------------------ SETUP CHIP SELECT UNITS ------------------

    # Chip Select Unit 4 -> Flash Memory -> 0010_0000 -> 0017_FFFF
    # FLASH_CS jumpered to DOC socket pin 22
    # (0000_00) 00_0001_0000_0000_0[000_0000_0000]
    # mask
    # (0000_00) 00_0000_0111_1111_1[111_1111_1111]

    mov dx, 0xF422 # CS4ADH
    mov ax, 0x0010
    out dx, ax
    mov dx, 0xF420 # CS4ADL
    mov ax, 0x0505
    out dx, ax
    mov dx, 0xF426 # CS4MSKH
    mov ax, 0x0007
    out dx, ax
    mov dx, 0xF424 # CS4MSKL
    mov ax, 0xFC01
    out dx, ax

    # Chip Select Unit 5 -> RTC -> (IO) 0070 -> 0071
    # 0000_0000_01 11_000[0]
    # mask
    # 0000_0000_00 00_000[1]

    mov dx, 0xF42A # CS5ADH
    mov ax, 0x0001
    out dx, ax
    mov dx, 0xF428 # CS5ADL
    mov ax, 0xC480
    out dx, ax
    mov dx, 0xF42E # CS5MSKH
    mov ax, 0x0000
    out dx, ax
    mov dx, 0xF42C # CS5MSKL
    mov ax, 0x0401
    out dx, ax

    # ------------------ SWITCH TO PROTECTED MODE -----------------

    lgdt [gdtinfo]
    mov eax, cr0
    or eax, 0x01
    mov cr0, eax

    # install data descriptors
    # mov ax, OFFSET (romdatadesc - gdt)
    mov ax, OFFSET (flatdata - gdt)
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax

    # install code descriptor
    jmp 0x18:protected

.section .text
.code32
.intel_syntax noprefix

protected:
    # ------------------ RELOCATE ROM ------------------
    # Change UCS to 0340_0000 -> 0340_1FFF
    # 11_0100_0000_0000_0[000_0000_0000]
    # mask
    # (0000_00) 00_0000_0000_0001_1[111_1111_1111]

    mov dx, 0xF43A # UCSADH
    mov ax, 0x0340
    out dx, ax
    mov dx, 0xF438 # UCSADL
    mov ax, 0x0505
    out dx, ax
    mov dx, 0xF43E # UCSMSKH
    mov ax, 0x0000
    out dx, ax
    mov dx, 0xF43C # UCSMSKL
    mov ax, 0x1C01
    out dx, ax

    # ------------------ TURN OFF LED ------------------
    mov dx, 0xF862
    in al, dx
    or ax, 0x40
    out dx, al

    # ------------------ INITIALIZE PROTECTED MODE ------------------
    xor eax, eax
    xor ebx, ebx
    xor ecx, ecx
    xor edx, edx
    xor esi, esi
    xor edi, edi
    mov esp, 0x1000
    mov ebp, esp
    jmp main

# real mode data
.section .gdt, "a"
.global gdtinfo
.global gdtinfo_relocated
.code32
gdt:
    nulldesc: .byte 0, 0, 0, 0, 0, 0, 0, 0
    # limits is (size - 1)
    # format (db limit[7:0], limit[15:8], base[7:0], base[15:8], base[23:16], access byte, {flags, limit[19:16]}, base[31:24])
    # 0008: code descriptor (limit = 64 KiB, base = 000F_0000):
    # codedesc: .byte 0x0F, 0x00, 0x00, 0x00, 0x0F, 10011010b, 11000000b, 0x00
    codedesc: .byte 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x9A, 0xC0, 0x00
    # 0010: data descriptor (limit = 64 KiB, base = 000F_0000): 
    # datadesc: .byte 0x0F, 0x00, 0x00, 0x00, 0x0F, 10010010b, 11000000b, 0x00
    datadesc: .byte 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x92, 0xC0, 0x00
    # 0018: rom code descriptor (limit = 8 KiB, base = 0140_0000):
    # romcodedesc: .byte 0x01, 0x00, 0x00, 0x00, 0x40, 10011010b, 11000000b, 0x03
    romcodedesc: .byte 0x01, 0x00, 0x00, 0x00, 0x40, 0x9A, 0xC0, 0x03
    # 0020: rom data descriptor (limit = 8 KiB, base = 0140_0000): 
    # romdatadesc: .byte 0x01, 0x00, 0x00, 0x00, 0x40, 10010010b, 11000000b, 0x03
    romdatadesc: .byte 0x01, 0x00, 0x00, 0x00, 0x40, 0x92, 0xC0, 0x03
    # 0028: code descriptor for whole (flat) memory
    # flatcode: .byte 0xff, 0xff, 0x00, 0x00, 0x00, 10011010b, 11001111b, 0x00
    flatcode: .byte 0xff, 0xff, 0x00, 0x00, 0x00, 0x9A, 0xCF, 0x00
    # 0030: data descriptor for whole (flat) memory
    # flatdata: .byte 0xff, 0xff, 0x00, 0x00, 0x00, 10010010b, 11001111b, 0x00
    flatdata: .byte 0xff, 0xff, 0x00, 0x00, 0x00, 0x92, 0xCF, 0x00
.equ gdt_size, .-gdt

gdtinfo:
    .word gdt_size - 1
    .word gdt
    .word 0x0340
    # align to 8 bytes
    .word 0

# reset vector for x86 processors
.section .reset, "ax"
.intel_syntax noprefix
.code16

reset:
    jmp 0xF000:start
    .byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
