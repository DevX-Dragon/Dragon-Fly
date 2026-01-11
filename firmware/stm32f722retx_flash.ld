/* Memory Mapping for STM32F722RET6 */
MEMORY
{
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 256K
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
}

/* Define sections (Code in Flash, Data in RAM) */
SECTIONS
{
  .text : { *(.text) } > FLASH
  .data : { *(.data) } > RAM AT > FLASH
  .bss  : { *(.bss) }  > RAM
}