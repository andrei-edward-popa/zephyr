/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_beep

#include <zephyr/drivers/buzzer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>

#define STM8_BEEP_ENABLE       0x20U
#define STM8_BEEP_SELECT_SHIFT 6U
#define STM8_BEEP_DIV_MIN      2U
#define STM8_BEEP_DIV_MAX      32U
#define STM8_BEEP_OPT4         0x4807U
#define STM8_BEEP_HSE_SOURCE   0x04U

struct buzzer_stm8_config {
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
	const struct pinctrl_dev_config *pcfg;
	uint32_t frequency;
};

struct buzzer_stm8_data {
	const struct device *dev;
	struct k_work_delayable stop_work;
	struct k_mutex lock;
	bool audible;
};

static void buzzer_stm8_silence(const struct device *dev)
{
	const struct buzzer_stm8_config *cfg = dev->config;

	sys_write8(sys_read8(cfg->base) & ~STM8_BEEP_ENABLE, cfg->base);
}

static void buzzer_stm8_stop_work(struct k_work *work)
{
	struct buzzer_stm8_data *data =
		CONTAINER_OF(k_work_delayable_from_work(work), struct buzzer_stm8_data, stop_work);

	buzzer_stm8_silence(data->dev);
}

static void buzzer_stm8_cancel_stop(struct buzzer_stm8_data *data)
{
	struct k_work_sync sync;

	(void)k_work_cancel_delayable_sync(&data->stop_work, &sync);
}

static int buzzer_stm8_tone(const struct device *dev, uint32_t frequency, uint32_t duration)
{
	const struct buzzer_stm8_config *cfg = dev->config;
	struct buzzer_stm8_data *data = dev->data;
	uint32_t rate;
	uint32_t best = UINT32_MAX;
	uint8_t control = 0U;
	int err;

	k_mutex_lock(&data->lock, K_FOREVER);
	buzzer_stm8_cancel_stop(data);
	buzzer_stm8_silence(dev);
	if (frequency == BUZZER_FREQ_REST || !data->audible || duration == 0U) {
		err = 0;
		goto out;
	}
	err = clock_control_get_rate(cfg->clock, cfg->clock_id, &rate);
	if (err != 0) {
		goto out;
	}
	/* The buzzer API permits rounding to the nearest supported frequency. */
	for (uint8_t select = 0U; select < 3U; select++) {
		for (uint8_t divisor = STM8_BEEP_DIV_MIN; divisor <= STM8_BEEP_DIV_MAX; divisor++) {
			uint32_t actual = rate / ((8U >> select) * divisor);
			uint32_t error =
				actual > frequency ? actual - frequency : frequency - actual;

			if (error < best) {
				best = error;
				control = STM8_BEEP_ENABLE | (select << STM8_BEEP_SELECT_SHIFT) |
					  (divisor - STM8_BEEP_DIV_MIN);
			}
		}
	}
	err = clock_control_on(cfg->clock, cfg->clock_id);
	if (err != 0) {
		goto out;
	}
	sys_write8(control, cfg->base);
	if (duration != BUZZER_DURATION_FOREVER) {
		err = k_work_schedule(&data->stop_work, K_MSEC(duration));
		if (err < 0) {
			buzzer_stm8_silence(dev);
			goto out;
		}
	}
	err = 0;
out:
	k_mutex_unlock(&data->lock);
	return err;
}

static int buzzer_stm8_stop(const struct device *dev)
{
	struct buzzer_stm8_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	buzzer_stm8_cancel_stop(data);
	buzzer_stm8_silence(dev);
	k_mutex_unlock(&data->lock);
	return 0;
}

static int buzzer_stm8_set_volume(const struct device *dev, uint8_t percent)
{
	struct buzzer_stm8_data *data = dev->data;

	if (percent > BUZZER_VOLUME_MAX) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	data->audible = percent != 0U;
	if (!data->audible) {
		buzzer_stm8_cancel_stop(data);
		buzzer_stm8_silence(dev);
	}
	k_mutex_unlock(&data->lock);
	return 0;
}

static int buzzer_stm8_beep(const struct device *dev, uint32_t duration)
{
	const struct buzzer_stm8_config *cfg = dev->config;

	return buzzer_stm8_tone(dev, cfg->frequency, duration);
}

static int buzzer_stm8_init(const struct device *dev)
{
	const struct buzzer_stm8_config *cfg = dev->config;
	struct buzzer_stm8_data *data = dev->data;
	int err;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}
	/* Keep the DT oscillator consistent with the shared AWU option-byte source. */
	if ((sys_read8(STM8_BEEP_OPT4) & STM8_BEEP_HSE_SOURCE) != 0U) {
		return -ENOTSUP;
	}
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}
	data->dev = dev;
	data->audible = true;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->stop_work, buzzer_stm8_stop_work);
	buzzer_stm8_silence(dev);
	return 0;
}

static DEVICE_API(buzzer, buzzer_stm8_api) = {
	.tone = buzzer_stm8_tone,
	.set_volume = buzzer_stm8_set_volume,
	.beep = buzzer_stm8_beep,
	.stop = buzzer_stm8_stop,
};

#define BUZZER_STM8_DEFINE(n)                                                                      \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct buzzer_stm8_data buzzer_stm8_data_##n;                                       \
	static const struct buzzer_stm8_config buzzer_stm8_config_##n = {                          \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.frequency = DT_INST_PROP(n, beep_frequency),                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, buzzer_stm8_init, NULL, &buzzer_stm8_data_##n,                    \
			      &buzzer_stm8_config_##n, POST_KERNEL, CONFIG_BUZZER_INIT_PRIORITY,   \
			      &buzzer_stm8_api);

DT_INST_FOREACH_STATUS_OKAY(BUZZER_STM8_DEFINE)
