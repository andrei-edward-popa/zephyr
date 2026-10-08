/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/printk.h>

volatile uint16_t stm8_fatal_reason;
#ifdef CONFIG_STM8_CONSOLE_TRACE
volatile uint16_t stm8_console_count;
volatile char stm8_console_buffer[256];
#endif

int arch_printk_char_out(int c)
{
#ifdef CONFIG_STM8_CONSOLE_TRACE
	stm8_console_buffer[stm8_console_count & 255U] = c;
	stm8_console_count++;
#endif
	return c;
}

void arch_cpu_idle(void)
{
	/* WFI enables IRQs atomically with entry into Wait mode (PM0044). */
	__asm__ volatile("wfi" : : : "cc", "memory");
}

void arch_cpu_atomic_idle(unsigned int key)
{
	__asm__ volatile("wfi" : : : "cc", "memory");
	arch_irq_unlock(key);
}

FUNC_NORETURN void arch_system_halt(unsigned int reason)
{
	stm8_fatal_reason = reason + 1U;
	__asm__ volatile("sim" : : : "memory");
	for (;;) {
		__asm__ volatile("halt");
	}
}

void z_stm8_fatal(unsigned int reason)
{
	z_fatal_error(reason, NULL);
}
