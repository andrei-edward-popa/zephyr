/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>
#include <zephyr/devicetree.h>
#include <zephyr/platform/hooks.h>
#include <zephyr/sys/util.h>

#define STM8_CLK_CKDIVR       (DT_REG_ADDR(DT_NODELABEL(clk)) + 0x06U)
#define STM8_CLK_HSIDIV_SHIFT 3U

void soc_early_init_hook(void)
{
	uint8_t hsi_shift = LOG2(DT_PROP(DT_NODELABEL(clk), st_hsi_divisor));
	uint8_t cpu_shift = LOG2(DT_PROP(DT_NODELABEL(clk), st_cpu_divisor));

	sys_write8((hsi_shift << STM8_CLK_HSIDIV_SHIFT) | cpu_shift, STM8_CLK_CKDIVR);
}
