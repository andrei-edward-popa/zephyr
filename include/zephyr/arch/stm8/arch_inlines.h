/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_ARCH_STM8_ARCH_INLINES_H_
#define ZEPHYR_INCLUDE_ARCH_STM8_ARCH_INLINES_H_
#ifndef _ASMLANGUAGE
#include <zephyr/kernel_structs.h>

static inline unsigned int arch_num_cpus(void)
{
	return 1U;
}
#endif
#endif
