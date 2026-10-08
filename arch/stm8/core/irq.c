/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sw_isr_table.h>
#include <kernel_internal.h>
#include <ksched.h>
#include <kswap.h>

void z_irq_spurious(const void *arg)
{
	ARG_UNUSED(arg);
	z_fatal_error(K_ERR_SPURIOUS_IRQ, NULL);
}

void *z_stm8_irq_dispatch(unsigned int irq, void *interrupted)
{
	struct _cpu *cpu = arch_curr_cpu();

	cpu->nested++;
	if (IS_ENABLED(CONFIG_IRQ_OFFLOAD) && irq == CONFIG_NUM_IRQS) {
		z_stm8_irq_offload_handler();
	} else {
		const struct _isr_table_entry *entry = &_sw_isr_table[irq];

		entry->isr(entry->arg);
	}
	cpu->nested--;
#ifdef CONFIG_MULTITHREADING
	if (IS_ENABLED(CONFIG_STACK_SENTINEL)) {
		z_check_stack_sentinel();
	}
	/* Publish the saved hardware frame and select the thread before leaving the ISR stack. */
	return z_get_next_switch_handle(interrupted);
#else
	return interrupted;
#endif
}
