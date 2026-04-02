ENTRY(rp2040_flash_stub_entry)

SECTIONS
{
    . = 0x20030000;

    .text : ALIGN(4)
    {
        KEEP(*(.text.entry))
        *(.text*)
        *(.rodata*)
        *(.data*)
    }

    /DISCARD/ :
    {
        *(.bss*)
        *(.comment*)
        *(.note*)
        *(.eh_frame*)
        *(.ARM.attributes)
    }
}
