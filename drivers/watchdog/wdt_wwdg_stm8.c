/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_wwdg

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>

#define STM8_WWDG_CR        0U
#define STM8_WWDG_WR        1U
#define STM8_WWDG_ENABLE    0x80U
#define STM8_WWDG_COUNT_MIN 0x40U
#define STM8_WWDG_COUNT_MAX 0x7fU
#define STM8_WWDG_DIVISOR   12288UL
#define STM8_WWDG_MS_SCALE  (STM8_WWDG_DIVISOR * 1000UL)

struct wdt_wwdg_stm8_config {
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
};

struct wdt_wwdg_stm8_data {
	uint32_t rate;
	struct wdt_window window;
	uint8_t reload;
	uint8_t limit;
	bool installed;
	bool running;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	struct clock_control_stm8_client clock_client;
#endif
};

static int wdt_wwdg_stm8_limits(uint32_t rate, const struct wdt_window *window, uint8_t *reload,
				uint8_t *limit)
{
	/* Bound the products before multiplying; all accepted values fit in 32 bits. */
	if (window->max == 0U || window->min >= window->max || rate == 0U ||
	    window->max > STM8_WWDG_MS_SCALE * STM8_WWDG_COUNT_MIN / rate) {
		return -EINVAL;
	}
	uint32_t ticks = DIV_ROUND_UP(rate * window->max, STM8_WWDG_MS_SCALE);
	uint32_t early = DIV_ROUND_UP(rate * window->min, STM8_WWDG_MS_SCALE);

	/* The first decrement can happen immediately after loading the counter. */
	if (window->min != 0U) {
		early++;
	}
	if (window->max == 0U || window->min >= window->max || ticks == 0U ||
	    ticks > STM8_WWDG_COUNT_MIN || early >= ticks) {
		return -EINVAL;
	}
	*reload = STM8_WWDG_COUNT_MIN + ticks - 1U;
	*limit = *reload - early;
	return 0;
}

static int wdt_wwdg_stm8_install_timeout(const struct device *dev,
					 const struct wdt_timeout_cfg *timeout)
{
	struct wdt_wwdg_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	int err;

	if (data->running || data->installed) {
		err = data->running ? -EBUSY : -ENOMEM;
	} else if (timeout->callback != NULL || timeout->flags != WDT_FLAG_RESET_SOC) {
		err = -ENOTSUP;
	} else {
		err = wdt_wwdg_stm8_limits(data->rate, &timeout->window, &data->reload,
					   &data->limit);
		if (err == 0) {
			data->window = timeout->window;
			data->installed = true;
		}
	}
	irq_unlock(key);
	return err;
}

static int wdt_wwdg_stm8_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_wwdg_stm8_config *cfg = dev->config;
	struct wdt_wwdg_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	if (options != 0U || data->running || !data->installed) {
		int err = options != 0U ? -ENOTSUP : data->running ? -EBUSY : -EINVAL;

		irq_unlock(key);
		return err;
	}
	/* Open the initial window before loading a free-running counter. */
	sys_write8(STM8_WWDG_COUNT_MAX, cfg->base + STM8_WWDG_WR);
	sys_write8(STM8_WWDG_ENABLE | data->reload, cfg->base + STM8_WWDG_CR);
	sys_write8(data->limit, cfg->base + STM8_WWDG_WR);
	data->running = true;
	irq_unlock(key);
	return 0;
}

static int wdt_wwdg_stm8_disable(const struct device *dev)
{
	struct wdt_wwdg_stm8_data *data = dev->data;

	return data->running ? -EPERM : -EFAULT;
}

static int wdt_wwdg_stm8_feed(const struct device *dev, int channel)
{
	const struct wdt_wwdg_stm8_config *cfg = dev->config;
	struct wdt_wwdg_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	uint8_t counter = sys_read8(cfg->base + STM8_WWDG_CR) & STM8_WWDG_COUNT_MAX;
	int err = 0;

	if (channel != 0 || !data->running) {
		err = -EINVAL;
	} else if (counter > data->limit) {
		err = -EAGAIN;
	} else if (counter < STM8_WWDG_COUNT_MIN) {
		err = -EIO;
	} else {
		sys_write8(STM8_WWDG_ENABLE | data->reload, cfg->base + STM8_WWDG_CR);
	}
	irq_unlock(key);
	return err;
}

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static int wdt_wwdg_stm8_clock_prepare(const struct device *dev, uint32_t rate)
{
	struct wdt_wwdg_stm8_data *data = dev->data;
	uint8_t reload;
	uint8_t limit;

	if (data->running) {
		return -EBUSY;
	}
	return data->installed ? wdt_wwdg_stm8_limits(rate, &data->window, &reload, &limit) : 0;
}

static void wdt_wwdg_stm8_clock_changed(const struct device *dev, uint32_t rate)
{
	struct wdt_wwdg_stm8_data *data = dev->data;

	data->rate = rate;
	if (data->installed) {
		(void)wdt_wwdg_stm8_limits(rate, &data->window, &data->reload, &data->limit);
	}
}
#endif

static int wdt_wwdg_stm8_init(const struct device *dev)
{
	const struct wdt_wwdg_stm8_config *cfg = dev->config;
	struct wdt_wwdg_stm8_data *data = dev->data;
	int err;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}
	err = clock_control_get_rate(cfg->clock, cfg->clock_id, &data->rate);
	if (err != 0) {
		return err;
	}
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	data->clock_client.clock_id = STM8_CLOCK_CPU;
	data->clock_client.dev = dev;
	data->clock_client.prepare = wdt_wwdg_stm8_clock_prepare;
	data->clock_client.changed = wdt_wwdg_stm8_clock_changed;
	return clock_control_stm8_register_client(&data->clock_client);
#else
	return 0;
#endif
}

static DEVICE_API(wdt, wdt_wwdg_stm8_api) = {
	.setup = wdt_wwdg_stm8_setup,
	.disable = wdt_wwdg_stm8_disable,
	.install_timeout = wdt_wwdg_stm8_install_timeout,
	.feed = wdt_wwdg_stm8_feed,
};

#define WDT_WWDG_STM8_DEFINE(n)                                                                    \
	static struct wdt_wwdg_stm8_data wdt_wwdg_stm8_data_##n;                                   \
	static const struct wdt_wwdg_stm8_config wdt_wwdg_stm8_config_##n = {                      \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, wdt_wwdg_stm8_init, NULL, &wdt_wwdg_stm8_data_##n,                \
			      &wdt_wwdg_stm8_config_##n, PRE_KERNEL_1,                             \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_wwdg_stm8_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_WWDG_STM8_DEFINE)
