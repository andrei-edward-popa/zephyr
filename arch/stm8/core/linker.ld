/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/linker/sections.h>
#include <zephyr/linker/iterable_sections.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>

#define ROMABLE_REGION ROM
#define RAMABLE_REGION RAM

/* ROM contains code and constant tables; initialized RAM has a flash LMA. */
PHDRS
{
	rom_constants PT_LOAD FLAGS(5);
	stacks PT_LOAD FLAGS(6);
	ram_data PT_LOAD FLAGS(6);
	rom_text PT_LOAD FLAGS(5);
	ram_bss PT_LOAD FLAGS(6);
	irq_metadata PT_LOAD FLAGS(4);
	stack PT_GNU_STACK FLAGS(6);
}

#undef GROUP_ROM_LINK_IN
#define GROUP_ROM_LINK_IN(vregion, lregion) > lregion :rom_constants
#undef GROUP_DATA_LINK_IN
#define GROUP_DATA_LINK_IN(vregion, lregion) > vregion AT > lregion :ram_data
#undef GROUP_LINK_IN
#define GROUP_LINK_IN(region) > region :PHDR_##region
#define PHDR_ROM rom_constants
#define PHDR_RAM ram_data
#define PHDR_IDT_LIST irq_metadata

ENTRY(__start)
MEMORY
{
	ROM (rx) : ORIGIN = 0x8000, LENGTH = 64K
	RAM (rw) : ORIGIN = 1, LENGTH = 6K - 1
	IDT_LIST : ORIGIN = 0x180000, LENGTH = 2K
}
SECTIONS
{
	.vectors : { KEEP(*(.vectors)) } > ROM :rom_constants
	PLACE_SYMBOL_HERE(__rom_region_start);
	.rodata :
	{
		*(.rodata .rodata.*)
#include <snippets-rodata.ld>
	} > ROM :rom_constants
#if LINKER_ZEPHYR_FINAL
	/DISCARD/ : { *(.gnu.linkonce.sw_isr_table*) }
#endif
#include <zephyr/linker/common-rom.ld>
#include <snippets-rom-sections.ld>
#if LINKER_ZEPHYR_FINAL
	__sw_isr_table = _sw_isr_table;
#endif
	/* RM0016 section 3.1.2: custom stacks must stay below 0x1400. */
	.stm8_stacks (NOLOAD) :
	{
		*(.stm8_stacks .stm8_stacks.*)
	} > RAM :stacks
	ASSERT(. <= 0x1400, "STM8 stacks exceed the hardware stack roll-over limit")
	.datas :
	{
		PLACE_SYMBOL_HERE(__data_region_start);
		PLACE_SYMBOL_HERE(__data_start);
		*(.data .data.* .sdata .sdata.*)
	} > RAM AT > ROM :ram_data
#include <zephyr/linker/common-ram.ld>
#include <snippets-ram-sections.ld>
	PLACE_SYMBOL_HERE(__data_region_end);
	__data_region_load_start = LOADADDR(.datas);
	PROVIDE(___data_region_load_start = __data_region_load_start);
	__data_region_num_bytes = __data_region_end - __data_region_start;
	PROVIDE(___data_region_num_bytes = __data_region_num_bytes);
	__near_flash_end = __data_region_load_start + __data_region_num_bytes;
	.text : { *(.text.startup) *(.text .text.*) } > ROM :rom_text
	PLACE_SYMBOL_HERE(__rom_region_end);
	.bss (NOLOAD) :
	{
		PLACE_SYMBOL_HERE(__bss_start);
		*(.bss .bss.* .sbss .sbss.* COMMON)
		PLACE_SYMBOL_HERE(__bss_end);
	} > RAM :ram_bss
	.noinit (NOLOAD) : { *(.noinit .noinit.*) } > RAM :ram_bss
	PLACE_SYMBOL_HERE(_image_ram_end);
	_image_ram_start = ORIGIN(RAM);
	PROVIDE(__image_ram_start = _image_ram_start);
	_image_ram_size = _image_ram_end - _image_ram_start;
	PROVIDE(__image_ram_size = _image_ram_size);
	__rom_region_size = __rom_region_end - ORIGIN(ROM);
	PROVIDE(___rom_region_size = __rom_region_size);
	ASSERT(__near_flash_end <= 0x10000, "STM8 constants and data load image exceed near flash")
#include <zephyr/linker/debug-sections.ld>
#include <snippets-sections.ld>
	/DISCARD/ : { *(.note.GNU-stack) }
}
