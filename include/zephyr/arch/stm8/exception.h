/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_ARCH_STM8_EXCEPTION_H_
#define ZEPHYR_INCLUDE_ARCH_STM8_EXCEPTION_H_
#ifndef _ASMLANGUAGE
#include <zephyr/types.h>
struct arch_esf {
	uint8_t cc;
	uint8_t a;
	uint16_t x;
	uint16_t y;
	uint8_t pc[3];
};
#endif
#endif
