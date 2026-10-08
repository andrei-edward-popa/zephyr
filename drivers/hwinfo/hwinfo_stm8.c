/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/drivers/hwinfo.h>

#define STM8_RST_SR     0x50b3U
#define STM8_RST_WWDGF  0x01U
#define STM8_RST_IWDGF  0x02U
#define STM8_RST_ILLOPF 0x04U
#define STM8_RST_SWIMF  0x08U
#define STM8_RST_EMCF   0x10U
#define STM8_RST_FLAGS  0x1fU
#define STM8_UID_BASE   0x48cdU
#define STM8_UID_SIZE   12U

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	length = MIN(length, STM8_UID_SIZE);
	for (size_t i = 0U; i < length; i++) {
		buffer[i] = sys_read8(STM8_UID_BASE + i);
	}
	return length;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint8_t sr = sys_read8(STM8_RST_SR);

	*cause = 0U;
	if ((sr & (STM8_RST_WWDGF | STM8_RST_IWDGF)) != 0U) {
		/* Software reset through WWDG is indistinguishable from WWDG expiry. */
		*cause |= RESET_WATCHDOG;
	}
	if ((sr & STM8_RST_ILLOPF) != 0U) {
		*cause |= RESET_CPU_LOCKUP;
	}
	if ((sr & STM8_RST_SWIMF) != 0U) {
		*cause |= RESET_DEBUG;
	}
	if ((sr & STM8_RST_EMCF) != 0U) {
		*cause |= RESET_HARDWARE;
	}
	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	sys_write8(STM8_RST_FLAGS, STM8_RST_SR);
	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = RESET_WATCHDOG | RESET_DEBUG | RESET_CPU_LOCKUP | RESET_HARDWARE;
	return 0;
}
