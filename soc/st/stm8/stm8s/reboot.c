/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/cpu.h>

#define STM8_WWDG_CR      0x50d1U
#define STM8_WWDG_CR_WDGA 0x80U

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);
	sys_write8(STM8_WWDG_CR_WDGA, STM8_WWDG_CR);
	for (;;) {
		arch_nop();
	}
}
