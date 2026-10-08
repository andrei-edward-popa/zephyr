/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_ARCH_STM8_KERNEL_ARCH_FUNC_H_
#define ZEPHYR_ARCH_STM8_KERNEL_ARCH_FUNC_H_
#include <kernel_arch_data.h>
#ifndef _ASMLANGUAGE
static inline void arch_kernel_init(void)
{
}

void z_stm8_arch_switch(void *switch_to, void **switched_from);
void z_stm8_irq_offload_handler(void);

static inline void arch_switch(void *switch_to, void **switched_from)
{
	z_stm8_arch_switch(switch_to, switched_from);
}

static inline bool arch_is_in_isr(void)
{
	return arch_curr_cpu()->nested != 0U;
}
#endif
#endif
