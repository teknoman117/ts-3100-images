INPUT(startup.o)
ENTRY(_reset)

ROM_BASE     = 0x03480000;
REAL_BASE    = 0x0000FF10;
RESET_BASE   = 0x0000FFF0;
REAL_OFFSET  = 0x0007FF10;
RESET_OFFSET = 0x0007FFF0;

MEMORY
{
    /* Low memory */
    RAM : ORIGIN = 0x00000000, LENGTH = 448K
    /* BIOS mirror memory (read only after init) */ 
    RORAM : ORIGIN = 0x000F0000, LENGTH = 64K
    /* ROM */
    ROM : ORIGIN = 0x03480000, LENGTH = 512K
}

SECTIONS
{
    PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

    .gdt : ALIGN(8)
    {
        . = ALIGN(8);
        (*.gdt);
        . = ALIGN(8);
    } > ROM
    . = ALIGN(8);

    .init32 : ALIGN(4)
    {
        . = ALIGN(4);
        (*.init32 .init32.*);
        . = ALIGN(4);
    } > ROM
    . = ALIGN(4);

    .text : ALIGN(4)
    {
        . = ALIGN(4);
        __stext = .;
        *(.text .text.*);
        . = ALIGN(4);
    } > RORAM AT>ROM
    . = ALIGN(4);
    __etext = .;
    __text_size_dwords = (SIZEOF(.text) / 4);
    __sitext = LOADADDR(.text);

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
    .init16 REAL_BASE : AT(ORIGIN(ROM) + REAL_OFFSET)
    {
        *(.init16);
    } > ROM

    .reset RESET_BASE : AT(ORIGIN(ROM) + RESET_OFFSET)
    {
        *(.reset);
    } > ROM
}
