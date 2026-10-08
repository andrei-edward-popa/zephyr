/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_iwdg

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>

#define STM8_IWDG_KR        0U
#define STM8_IWDG_PR        1U
#define STM8_IWDG_RLR       2U
#define STM8_IWDG_ENABLE    0xccU
#define STM8_IWDG_ACCESS    0x55U
#define STM8_IWDG_REFRESH   0xaaU
#define STM8_IWDG_PR_MAX    6U
#define STM8_IWDG_COUNT_MAX 256U

struct wdt_iwdg_stm8_config {
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
};

struct wdt_iwdg_stm8_data {
	uint32_t rate;
	uint8_t prescaler;
	uint8_t reload;
	bool installed;
	bool running;
};

static int wdt_iwdg_stm8_install_timeout(const struct device *dev,
					 const struct wdt_timeout_cfg *timeout)
{
	struct wdt_iwdg_stm8_data *data = dev->data;
	uint8_t prescaler;
	uint32_t ticks;
	unsigned int key = irq_lock();

	if (data->running) {
		irq_unlock(key);
		return -EBUSY;
	}
	if (data->installed) {
		irq_unlock(key);
		return -ENOMEM;
	}
	if (timeout->callback != NULL || timeout->window.min != 0U ||
	    timeout->flags != WDT_FLAG_RESET_SOC) {
		irq_unlock(key);
		return -ENOTSUP;
	}
	if (timeout->window.max == 0U || timeout->window.max > (1000UL << (STM8_IWDG_PR_MAX + 3U)) *
								       STM8_IWDG_COUNT_MAX /
								       data->rate) {
		irq_unlock(key);
		return -EINVAL;
	}
	for (prescaler = 0U; prescaler <= STM8_IWDG_PR_MAX; prescaler++) {
		uint32_t denominator = 1000UL << (prescaler + 3U);

		ticks = DIV_ROUND_UP(timeout->window.max * data->rate, denominator);
		if (ticks <= STM8_IWDG_COUNT_MAX) {
			break;
		}
	}
	if (prescaler > STM8_IWDG_PR_MAX) {
		irq_unlock(key);
		return -EINVAL;
	}
	data->prescaler = prescaler;
	data->reload = ticks - 1U;
	data->installed = true;
	irq_unlock(key);
	return 0;
}

static int wdt_iwdg_stm8_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_iwdg_stm8_config *cfg = dev->config;
	struct wdt_iwdg_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	if (options != 0U || data->running || !data->installed) {
		int err = options != 0U ? -ENOTSUP : data->running ? -EBUSY : -EINVAL;

		irq_unlock(key);
		return err;
	}
	/* Protected registers become writable only after starting the watchdog. */
	sys_write8(STM8_IWDG_ENABLE, cfg->base + STM8_IWDG_KR);
	sys_write8(STM8_IWDG_ACCESS, cfg->base + STM8_IWDG_KR);
	sys_write8(data->prescaler, cfg->base + STM8_IWDG_PR);
	sys_write8(data->reload, cfg->base + STM8_IWDG_RLR);
	sys_write8(STM8_IWDG_REFRESH, cfg->base + STM8_IWDG_KR);
	data->running = true;
	irq_unlock(key);
	return 0;
}

static int wdt_iwdg_stm8_disable(const struct device *dev)
{
	struct wdt_iwdg_stm8_data *data = dev->data;

	return data->running ? -EPERM : -EFAULT;
}

static int wdt_iwdg_stm8_feed(const struct device *dev, int channel)
{
	const struct wdt_iwdg_stm8_config *cfg = dev->config;
	struct wdt_iwdg_stm8_data *data = dev->data;

	if (channel != 0 || !data->running) {
		return -EINVAL;
	}
	sys_write8(STM8_IWDG_REFRESH, cfg->base + STM8_IWDG_KR);
	return 0;
}

static int wdt_iwdg_stm8_init(const struct device *dev)
{
	const struct wdt_iwdg_stm8_config *cfg = dev->config;
	struct wdt_iwdg_stm8_data *data = dev->data;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}
	return clock_control_get_rate(cfg->clock, cfg->clock_id, &data->rate);
}

static DEVICE_API(wdt, wdt_iwdg_stm8_api) = {
	.setup = wdt_iwdg_stm8_setup,
	.disable = wdt_iwdg_stm8_disable,
	.install_timeout = wdt_iwdg_stm8_install_timeout,
	.feed = wdt_iwdg_stm8_feed,
};

#define WDT_IWDG_STM8_DEFINE(n)                                                                    \
	static struct wdt_iwdg_stm8_data wdt_iwdg_stm8_data_##n;                                   \
	static const struct wdt_iwdg_stm8_config wdt_iwdg_stm8_config_##n = {                      \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, wdt_iwdg_stm8_init, NULL, &wdt_iwdg_stm8_data_##n,                \
			      &wdt_iwdg_stm8_config_##n, PRE_KERNEL_1,                             \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_iwdg_stm8_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_IWDG_STM8_DEFINE)
