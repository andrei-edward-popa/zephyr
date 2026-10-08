/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_ARCH_STM8_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_STM8_THREAD_H_
#ifndef _ASMLANGUAGE
#include <zephyr/types.h>

struct _callee_saved {
	uint16_t sp;
};
typedef struct _callee_saved _callee_saved_t;

struct _thread_arch {
	void (*entry)(void *p1, void *p2, void *p3);
	void *p1;
	void *p2;
	void *p3;
};
typedef struct _thread_arch _thread_arch_t;
#endif
#endif
