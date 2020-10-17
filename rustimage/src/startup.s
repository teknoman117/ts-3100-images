# reset vector for x86 processors
.section .reset, "ax"
.global _reset
.code16

_reset:
    ljmp $0xF000, $_start
    .byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

# initial real mode section
.section .init16, "ax"
.code16
.align 8
_gdtinfo:
    .word gdt_size - 1
    .long gdt

.align 8
_start:
    # --- Setup initial state (replace eventually) ---

    # Point data segments at code segments
    movw %cs, %ax
    movw %ax, %ds
    movw %ax, %es

    # Enable "Extended I/O Space" -> in short, the 386EX internal peripherals
    movw $0x8000, %ax
    outb %al, $0x23
    xchgb %ah, %al
    outb %al, $0x22
    outw %ax, $0x22

    # Map all I/O in both DOS and Extended regions
    xorw %ax, %ax
    outb %al, $0x22

    # Disable watchdog timer
    movw $0xF4CA, %dx
    movb $0x01, %al
    outb %al, %dx

    # IO port configuration
    movw $0xF862, %dx
    movb $0, %al
    outb %al, (%dx)
    movw $0xF86A, %dx
    movb $0x1F, %al
    outb %al, (%dx)
    movw $0xF872, %dx
    movb $0x44, %al
    outb %al, (%dx)
    movw $0xF864, %dx
    movb $0xBD, %al
    outb %al, (%dx)
    movw $0xF86C, %dx
    movb $0x10, %al
    outb %al, (%dx)
    movw $0xF874, %dx
    movb $0x44, %al
    outb %al, (%dx)
    movw $0xF820, %dx
    movb $0, %al
    outb %al, (%dx)
    movw $0xF822, %dx
    movb $0xFF, %al
    outb %al, (%dx)
    movw $0xF824, %dx
    movb $0xBB, %al
    outb %al, (%dx)

    # setup serial pins
    movw $0xF826, %dx
    inb (%dx), %al
    andb $0x80, %al
    orb $0x3F, %al
    outb %al, (%dx)

    # setup serial clocking
    movw $0xF836, %dx
    movb $0x40, %al
    outb %al, (%dx)

    # setup refresh unit
    # looks like a read in (0x90000 -> 0x93FFF) in 8 bit mode, as BHE and BLE aren't available
    # refresh pin is not connected to the PLD
    movw $0xF4A0, %dx
    movw $0x0024, %ax
    outw %ax, (%dx)

    # how many 25 MHz cycles pass before issuing refresh
    movw $0xF4A2, %dx
    movw $0x030C, %ax
    outw %ax, (%dx)

    # start refresh unit
    movw $0xF4A4, %dx
    movw $0x8000, %ax
    outw %ax, (%dx)

    # Chip Select Unit 4 -> DOC -> 000D_8000 -> 000D_9FFF
    # (0000_00) 00_0000_1101_1000_0[000_0000_0000]
    # mask
    # (0000_00) 00_0000_0000_0001_1[111_1111_1111]

    # CS4ADH
    movw $0xF422, %dx
    movw $0x000D, %ax
    outw %ax, (%dx)
    # CS4ADL
    movw $0xF420, %dx
    movw $0x8503, %ax
    outw %ax, (%dx)
    # CS4MSKH
    movw $0xF426, %dx
    # movw $0x0000, %ax
    xorl %eax, %eax
    outw %ax, (%dx)
    # CS4MSKL
    movw $0xF424, %dx
    movw $0x1C01, %ax
    outw %ax, (%dx)

    # --- ----------------------------------------------

    # load gdt
    lgdtl %ds:(_gdtinfo)

    # switch into protected mode (short jump to flush prefetcher)
    movl %cr0, %eax
    orl $0x01, %eax
    movl %eax, %cr0
    jmp _start.descriptors

_start.descriptors:
    # install data descriptors
    movw $0x10, %ax
    movw %ax, %ds
    movw %ax, %es
    movw %ax, %fs
    movw %ax, %gs
    movw %ax, %ss

    # install code descriptor
    ljmpl $0x08, $_protected

.section .init32, "ax"
.code32
_protected:
    # copy .rodata segment (to RORAM)
    movl $__sirodata, %esi
    movl $__srodata, %edi
    movl $__rodata_size_dwords, %ecx
    rep movsl %ds:(%esi), %es:(%edi)

    # copy .text segment (to RORAM)
    movl $__sitext, %esi
    movl $__stext, %edi
    movl $__text_size_dwords, %ecx
    rep movsl %ds:(%esi), %es:(%edi)

    # relocate rom to normal region
    # will disable access to RORAM region
    # Change UCS to 0340_0000 -> 0340_1FFF
    # 11_0100_0000_0000_0[000_0000_0000]
    # mask
    # (0000_00) 00_0000_0000_0001_1[111_1111_1111]

    # UCSADH
    movw $0xF43A, %dx
    movw $0x0340, %ax
    outw %ax, (%dx)
    # UCSADL
    movw $0xF438, %dx
    movw $0x0505, %ax
    outw %ax, (%dx)
    # UCSMSKH
    movw $0xF43E, %dx
    # movw $0x0000, %ax
    xorl %eax, %eax
    outw %ax, (%dx)
    # UCSMSKL
    movw $0xF43C, %dx
    movw $0x1C01, %ax
    outw %ax, (%dx)

    # zero .bss segment
    movl $__sbss, %edi
    movl $__bss_size_dwords, %ecx
    xorl %eax, %eax
    rep stos %eax, %es:(%edi)

    # copy .data segment
    movl $__sidata, %esi
    movl $__sdata, %edi
    movl $__data_size_dwords, %ecx
    rep movsl %ds:(%esi), %es:(%edi)

    # initialize registers
    xorl %eax, %eax
    xorl %ebx, %ebx
    xorl %ecx, %ecx
    xorl %edx, %edx
    xorl %esi, %esi
    xorl %edi, %edi

    movl $_stack_start, %esp
    movl %esp, %ebp
    jmp main

# global descriptor table
.section .gdt, "a"
.global gdtinfo
.code32
gdt:
    nulldesc: .byte 0, 0, 0, 0, 0, 0, 0, 0
    # limits is (size - 1)
    # format (db limit[7:0], limit[15:8], base[7:0], base[15:8], base[23:16], access byte, {flags, limit[19:16]}, base[31:24])
    # 0008: code descriptor for whole (flat) memory
    # flatcode: .byte 0xff, 0xff, 0x00, 0x00, 0x00, 10011010b, 11001111b, 0x00
    code: .byte 0xff, 0xff, 0x00, 0x00, 0x00, 0x9A, 0xCF, 0x00
    # 0010: data descriptor for whole (flat) memory
    # flatdata: .byte 0xff, 0xff, 0x00, 0x00, 0x00, 10010010b, 11001111b, 0x00
    data: .byte 0xff, 0xff, 0x00, 0x00, 0x00, 0x92, 0xCF, 0x00
.equ gdt_size, .-gdt
