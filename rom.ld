SECTIONS
{
    .gdt :
    {
        . = ALIGN(8);
        *(.gdt .gdt.*)
    }

    .realmode :
    {
        . = ALIGN(16);
        *(.text_realmode .text_realmode.*)
    }

    .rom :
    {
        . = ALIGN(16);
        *(.data .data.*)
        . = ALIGN(16);
        *(.text .text.*)
    }

    /* reset vector */
    . = 0x1FF0;
    .reset :
    {
        *(.reset)
    }
    . = 0x2000;
    rom_size = .;
}