/* flash.ld — RP2350 Pico2W memory layout for PX4 FMU */

MEMORY
{
    /* RP2350 boots from external QSPI flash (W25Q32 = 4MB on Pico2W)    
    XIP window at 0x10000000 — chip has NO internal flash              
     First 256 bytes = second-stage bootloader (boot2)                  */
    BOOT2  (rx)  : ORIGIN = 0x10000000, LENGTH = 256
    FLASH  (rx)  : ORIGIN = 0x10000100, LENGTH = 4096K - 256

    /* RP2350 SRAM — 520KB across 10 banks                                
     Banks 0-7 : 512KB at 0x20000000 — main heap/stack/data            
     Bank 8    : 4KB scratch X at 0x20080000 — leave for SDK use       
     Bank 9    : 4KB scratch Y at 0x20081000 — Core 1 dedicated stack  */
    SRAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 512K   // fix: was 516K 
    SRAM_C1 (rw)  : ORIGIN = 0x20081000, LENGTH = 4K     // Bank 9 only  
}

SECTIONS
{
    /*  BOOT2 — must be first 256 bytes of flash  */
    .boot2 :
    {
        *(.boot2)
        FILL(0xff)
        . = ORIGIN(BOOT2) + LENGTH(BOOT2);
    } > BOOT2

    /*  TEXT — code and read-only data in flash  */
    .text :
    {
        _stext = ABSOLUTE(.);
        *(.vectors)          // interrupt vector table must be first 
        *(.text)
        *(.text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata)
        *(.rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(4);
        _etext = ABSOLUTE(.);
    } > FLASH

    /*  ARM EXCEPTION INDEX (C++ / RTTI)  */
    .ARM.exidx :
    {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        *(.gnu.linkonce.armexidx.*)
        __exidx_end = ABSOLUTE(.);
    } > FLASH

    /*  DATA LOAD ADDRESS (flash → SRAM copy marker)  */
    /* NuttX arm_start.c uses _eronly as the flash source address for      */
    /* copying .data into SRAM at boot                                     */
    _eronly = ABSOLUTE(.);   /* end of read-only flash content             */

    /*  DATA — initialized variables, loaded from flash, run in SRAM  */
    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > SRAM AT > FLASH

    /*  BSS — zero-initialized variables in SRAM  */
    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > SRAM

    /*  HEAP  */
    /* NuttX mm_initialize() uses these bounds                              */
    .heap :
    {
        _sheap = ABSOLUTE(.);
        . = ORIGIN(SRAM) + LENGTH(SRAM);  /* heap fills remaining SRAM    */
        _eheap = ABSOLUTE(.);
    } > SRAM

    /*  IDLE THREAD STACK (NuttX scheduler requirement)  */
    /* g_idle_topstack must point to top of initial stack                  */
    /* NuttX places idle stack at top of SRAM by convention                */
    g_idle_topstack = ORIGIN(SRAM) + LENGTH(SRAM);

    /*  CORE 1 DEDICATED STACK — Bank 9 scratch Y  */
    .core1_stack (NOLOAD) :
    {
        _score1_stack = ABSOLUTE(.);
        . = . + LENGTH(SRAM_C1);
        _ecore1_stack = ABSOLUTE(.);
    } > SRAM_C1
}