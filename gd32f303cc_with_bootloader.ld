EXTERN(vector_table)
ENTRY(reset_handler)
MEMORY
{
	/* leave 8 bytes at the end of ram for argument passing */
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 49144

	/*
	256KB flash total
	16KB reserved at start for bootloader.
	128KB reserved at end for user data (18K * 7 + 2K).
	*/
	rom (rx) : ORIGIN = 0x08004000, LENGTH = 112K
}
SECTIONS
{
 .text : {
  *(.vectors)
  . = ALIGN(2048);
  *(.start*)
  KEEP (*(.start))
  . = ALIGN(2048);
  *(.text*)
  . = ALIGN(4);
  *(.rodata*)
  . = ALIGN(4);
 } >rom

 .preinit_array : {
  . = ALIGN(4);
  __preinit_array_start = .;
  KEEP (*(.preinit_array))
  __preinit_array_end = .;
 } >rom

 .init_array : {
  . = ALIGN(4);
  __init_array_start = .;
  KEEP (*(SORT(.init_array.*)))
  KEEP (*(.init_array))
  __init_array_end = .;
 } >rom

 .fini_array : {
  . = ALIGN(4);
  __fini_array_start = .;
  KEEP (*(.fini_array))
  KEEP (*(SORT(.fini_array.*)))
  __fini_array_end = .;
 } >rom

 .fini : {
  KEEP (*(.fini))
 } >rom

 .ARM.extab : {
  *(.ARM.extab*)
 } >rom

 .ARM.exidx : {
  __exidx_start = .;
  *(.ARM.exidx*)
  __exidx_end = .;
 } >rom

 . = ALIGN(4);
 _etext = .;

 .data : {
  _data = .;
  *(.data*)
  . = ALIGN(4);
  _edata = .;
 } >ram AT >rom

 _data_loadaddr = LOADADDR(.data);
 .bss : {
  *(.bss*)
  *(COMMON)
  . = ALIGN(4);
  _ebss = .;
 } >ram

 /DISCARD/ : { *(.eh_frame) }

 . = ALIGN(4);
 end = .;
}

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
