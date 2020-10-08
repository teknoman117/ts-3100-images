INPUT(startup.o)
ENTRY(_reset)

ROM_BASE   = 0x03400000;
REAL_BASE  = 0x1F00;
RESET_BASE = 0x1FF0;

SECTIONS
{
    . = 0x0;
    __ram_start = .;
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
    .text :
    {
        *(.rodata .rodata.*);
        . = ALIGN(8);
        *(.data .data.*);
        . = ALIGN(8);
        *(.text .text.*);
    }

    /* Real mode init code */
    .init16 REAL_BASE : AT(ROM_BASE + REAL_BASE) { *(.init16); }

    /* Reset Vector for x86 */
    .reset RESET_BASE : AT(ROM_BASE + RESET_BASE) { *(.reset); }
}
