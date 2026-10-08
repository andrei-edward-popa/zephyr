/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/kernel.h>
#include <zephyr/fatal.h>

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
	ARG_UNUSED(reason);
	__asm__ volatile("sim" : : : "memory");
	for (;;) {
		__asm__ volatile("halt");
	}
}

void z_stm8_fatal(unsigned int reason)
{
	z_fatal_error(reason, NULL);
}
