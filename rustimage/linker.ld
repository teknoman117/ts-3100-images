INPUT(startup.o)
ENTRY(_reset)

ROM_BASE   = 0x03400000;
REAL_BASE  = 0x1F60;
RESET_BASE = 0x1FF0;

MEMORY
{
    /* Low memory */
    RAM : ORIGIN = 0x00000000, LENGTH = 448K
    /* BIOS mirror memory (read only after init) */ 
    RORAM : ORIGIN = 0x000F0000, LENGTH = 64K
    /* ROM */
    ROM : ORIGIN = 0x03400000, LENGTH = 8K
}

SECTIONS
{
    PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

    .gdt :
    {
        (*.gdt);
        . = ALIGN(8);
    } > ROM

    .init32 :
    {
        (*.init32 .init32.*);
        . = ALIGN(8);
    } > ROM

    .text :
    {
        *(.text .text.*);
        . = ALIGN(8);
    } > ROM

    .rodata : ALIGN(4)
    {
        . = ALIGN(4);
        __srodata = .;
        *(.rodata .rodata.*);
        . = ALIGN(4);
    } > RORAM AT>ROM
    . = ALIGN(4);
    __erodata = .;
    __rodata_size_dwords = (SIZEOF(.rodata) / 4);
    __sirodata = LOADADDR(.rodata);

    /* data (read/write) segment */
    .data : ALIGN(4)
    {
        . = ALIGN(4);
        __sdata = .;
        *(.data .data.*);
        . = ALIGN(4);
    } > RAM AT>ROM
    . = ALIGN(4);
    __edata = .;
    __data_size_dwords = (SIZEOF(.data) / 4);
    __sidata = LOADADDR(.data);

    /* bss (uninitialized) segment */
    . = ALIGN(4);
    __sbss = .;
    .bss (NOLOAD) : ALIGN(4)
    {
        *(.bss .bss.*);
        *(COMMON);
        . = ALIGN(4);
    } > RAM
    . = ALIGN(4);
    __ebss = .;
    __bss_size_dwords = (SIZEOF(.bss) / 4);

    /* exception handling data */
    .eh_frame_hdr (NOLOAD) :
    {
        (*.eh_frame_hdr);
        . = ALIGN(8);
    } > ROM

    .eh_frame (NOLOAD) :
    {
        (*.eh_frame);
        . = ALIGN(8);
    } > ROM

    /* real mode initialization */
    .init16 REAL_BASE : AT(ORIGIN(ROM) + REAL_BASE) { *(.init16); } >ROM
    .reset RESET_BASE : AT(ORIGIN(ROM) + RESET_BASE) { *(.reset); } >ROM
}

/*SECTIONS
{
    . = 0x0;
    __ram_start = .;
    __edata_start = .;
    . += SIZEOF(.data);
    __edata_end = .;
    . = ALIGN(16);
    __ebss_start = .;
    . += SIZEOF(.bss);
    __ebss_end = .;
    . = 0x70000;
    __ram_end = .;

    . = ROM_BASE;
    .init32 :
    {
        KEEP(*(.gdt));
        . = ALIGN(8);
        *(.init32);
    }

    . = ALIGN(8);
    .text : { *(.text .text.*); }
    . = ALIGN(8);
    .rodata : { *(.rodata .rodata.*); }
    . = ALIGN(8);
    __sdata_start = .;
    .data 0 : AT(0x03401000) { *(.data .data.*); }
    __sdata_end = .;

    .init16 REAL_BASE : AT(ROM_BASE + REAL_BASE) { *(.init16); }

    .reset RESET_BASE : AT(ROM_BASE + RESET_BASE) { *(.reset); }
}*/