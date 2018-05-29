MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 24K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

__cbtable_start__ = ORIGIN(FLASH) + 0x01C0;
__text_start__    = ORIGIN(FLASH) + 0x0200;

SECTIONS
{
    __STACKSIZE__ = 512;

    startup :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    cbtable __cbtable_start__ :
    {
        KEEP(*(_callback_table))
    } > FLASH

    .text __text_start__ :
    {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data : AT (ADDR(.text) + SIZEOF(.text))
    {
        _data = .;
        *(vtable)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
        _stack = .;
        . = ALIGN(MAX(_stack + __STACKSIZE__ , .), 4);
        _estack = .;
        
    } > SRAM
}
