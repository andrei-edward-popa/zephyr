/*
 * Copyright (c) 2020 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>

/* Freestanding LTO can introduce an implicit abort call during RTL expansion. */
FUNC_NORETURN __used void abort(void)
{
	printk("abort()\n");
	k_panic();
	CODE_UNREACHABLE;
}
