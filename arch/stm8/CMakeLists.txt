# SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(include)
zephyr_library_sources(
  core/reset.S
  core/switch.S
  core/cpu.c
  core/atomic.c
  core/irq.S
  core/irq.c
)
zephyr_library_sources_ifdef(CONFIG_MULTITHREADING core/thread.c)
zephyr_library_sources_ifdef(CONFIG_IRQ_OFFLOAD core/irq_offload.c)
set_property(GLOBAL PROPERTY PROPERTY_OUTPUT_FORMAT "elf32-stm8")

zephyr_linker_sources(RODATA core/swi_tables.ld)
