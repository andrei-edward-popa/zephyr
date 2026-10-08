/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/irq_offload.h>
#include <zephyr/kernel.h>

static irq_offload_routine_t stm8_offload_routine;
static const void *stm8_offload_parameter;

void z_stm8_irq_offload_handler(void)
{
	irq_offload_routine_t routine = stm8_offload_routine;
	const void *parameter = stm8_offload_parameter;

	stm8_offload_routine = NULL;
	__ASSERT_NO_MSG(routine != NULL);
	routine(parameter);
}

void arch_irq_offload(irq_offload_routine_t routine, const void *parameter)
{
	unsigned int key;

	__ASSERT_NO_MSG(!k_is_in_isr());
	key = arch_irq_lock();
	stm8_offload_routine = routine;
	stm8_offload_parameter = parameter;
	/* PM0044: TRAP is synchronous and is not masked by SIM. */
	__asm__ volatile("trap" : : : "memory");
	arch_irq_unlock(key);
}

void arch_irq_offload_init(void)
{
	/* The TRAP vector needs no interrupt-controller configuration. */
}
