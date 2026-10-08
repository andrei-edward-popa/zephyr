/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_adc

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

#define STM8_ADC_CSR            0U
#define STM8_ADC_CR1            1U
#define STM8_ADC_CR2            2U
#define STM8_ADC_DRH            4U
#define STM8_ADC_DRL            5U
#define STM8_ADC_TDRH           6U
#define STM8_ADC_TDRL           7U
#define STM8_ADC_EOC            0x80U
#define STM8_ADC_EOCIE          0x20U
#define STM8_ADC_ADON           0x01U
#define STM8_ADC_ALIGN_RIGHT    0x08U
#define STM8_ADC_PRESCALE_SHIFT 4U
#define STM8_ADC_CLOCK_MIN      1000000UL
#define STM8_ADC_CLOCK_MAX      4000000UL
#define STM8_ADC_RESOLUTION     10U
#define STM8_ADC_OVERSAMPLE_MAX 8U
#define STM8_ADC_CHANNEL_MAX    15U
#define STM8_ADC_STARTUP_US     20U

struct adc_stm8_config {
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
	const struct pinctrl_dev_config *pcfg;
	uint16_t channel_mask;
	void (*irq_config)(const struct device *dev);
};

struct adc_stm8_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint16_t configured;
	uint16_t pending;
	uint16_t remaining;
	uint16_t samples;
	uint32_t sum;
	uint8_t channel;
	uint8_t control;
	bool vector_enabled;
	bool active;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	struct clock_control_stm8_client clock_client;
#endif
};

static int adc_stm8_prescaler(uint32_t rate)
{
	static const uint8_t divisors[] = {2U, 3U, 4U, 6U, 8U, 10U, 12U, 18U};

	for (uint8_t i = 0U; i < ARRAY_SIZE(divisors); i++) {
		uint32_t frequency = rate / divisors[i];

		if (frequency >= STM8_ADC_CLOCK_MIN && frequency <= STM8_ADC_CLOCK_MAX) {
			return i << STM8_ADC_PRESCALE_SHIFT;
		}
	}
	return -EINVAL;
}

static void adc_stm8_irq_set_enabled(const struct device *dev, bool enabled)
{
	const struct adc_stm8_config *cfg = dev->config;
	struct adc_stm8_data *data = dev->data;
	uint8_t eoc = sys_read8(cfg->base + STM8_ADC_CSR) & STM8_ADC_EOC;

	data->vector_enabled = enabled;
	sys_write8(eoc | data->channel | (enabled && data->active ? STM8_ADC_EOCIE : 0U),
		   cfg->base + STM8_ADC_CSR);
}

static void adc_stm8_convert(const struct device *dev)
{
	const struct adc_stm8_config *cfg = dev->config;
	struct adc_stm8_data *data = dev->data;

	/* EOC is cleared explicitly; do not read-modify-write the channel bits. */
	sys_write8(data->channel | (data->vector_enabled ? STM8_ADC_EOCIE : 0U),
		   cfg->base + STM8_ADC_CSR);
	if ((sys_read8(cfg->base + STM8_ADC_CR1) & STM8_ADC_ADON) == 0U) {
		sys_write8(data->control | STM8_ADC_ADON, cfg->base + STM8_ADC_CR1);
		k_busy_wait(STM8_ADC_STARTUP_US);
	}
	/* A second write of ADON starts a single conversion. */
	sys_write8(data->control | STM8_ADC_ADON, cfg->base + STM8_ADC_CR1);
}

static void adc_stm8_next_channel(struct adc_stm8_data *data)
{
	data->channel = find_lsb_set(data->pending) - 1U;
	data->pending &= ~BIT(data->channel);
	data->remaining = data->samples;
	data->sum = 0U;
	adc_stm8_convert(data->dev);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_stm8_data *data = CONTAINER_OF(ctx, struct adc_stm8_data, ctx);

	data->repeat_buffer = data->buffer;
	data->pending = ctx->sequence.channels;
	adc_stm8_next_channel(data);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat)
{
	struct adc_stm8_data *data = CONTAINER_OF(ctx, struct adc_stm8_data, ctx);

	if (repeat) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_stm8_data *data = CONTAINER_OF(ctx, struct adc_stm8_data, ctx);
	const struct adc_stm8_config *cfg = data->dev->config;

	ARG_UNUSED(status);
	data->active = false;
	sys_write8(data->channel, cfg->base + STM8_ADC_CSR);
	sys_write8(data->control, cfg->base + STM8_ADC_CR1);
}

static void adc_stm8_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct adc_stm8_config *cfg = dev->config;
	struct adc_stm8_data *data = dev->data;

	if ((sys_read8(cfg->base + STM8_ADC_CSR) & STM8_ADC_EOC) == 0U) {
		return;
	}
	/* Right alignment requires reading the low byte first to latch the high byte. */
	uint16_t result = sys_read8(cfg->base + STM8_ADC_DRL);

	result |= (uint16_t)sys_read8(cfg->base + STM8_ADC_DRH) << 8;
	sys_write8(data->channel, cfg->base + STM8_ADC_CSR);
	if (!data->active) {
		return;
	}
	data->sum += result;
	if (--data->remaining != 0U) {
		adc_stm8_convert(dev);
		return;
	}
	*data->buffer++ = data->sum >> data->ctx.sequence.oversampling;
	if (data->pending != 0U) {
		adc_stm8_next_channel(data);
	} else {
		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

static int adc_stm8_channel_setup(const struct device *dev, const struct adc_channel_cfg *channel)
{
	const struct adc_stm8_config *cfg = dev->config;
	struct adc_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	int err = 0;

	if (data->active) {
		err = -EBUSY;
	} else if (channel->channel_id > STM8_ADC_CHANNEL_MAX ||
		   (cfg->channel_mask & BIT(channel->channel_id)) == 0U) {
		err = -EINVAL;
	} else if (channel->gain != ADC_GAIN_1 || channel->reference != ADC_REF_VDD_1 ||
		   channel->differential || channel->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		err = -ENOTSUP;
	} else {
		data->configured |= BIT(channel->channel_id);
		sys_write8(data->configured >> 8, cfg->base + STM8_ADC_TDRH);
		sys_write8(data->configured, cfg->base + STM8_ADC_TDRL);
	}
	irq_unlock(key);
	return err;
}

static int adc_stm8_start_read(const struct device *dev, const struct adc_sequence *sequence,
			       bool asynchronous, struct k_poll_signal *signal)
{
	struct adc_stm8_data *data = dev->data;
	uint32_t channels = sequence->channels;
	uint32_t count = 0U;
	int err;

	adc_context_lock(&data->ctx, asynchronous, signal);
	if (channels == 0U || (channels & ~((uint32_t)data->configured)) != 0U) {
		err = -EINVAL;
		goto out;
	}
	if (sequence->resolution != STM8_ADC_RESOLUTION ||
	    sequence->oversampling > STM8_ADC_OVERSAMPLE_MAX || sequence->calibrate) {
		err = -ENOTSUP;
		goto out;
	}
	while (channels != 0U) {
		count++;
		channels &= channels - 1U;
	}
	if (sequence->options != NULL) {
		count *= 1UL + sequence->options->extra_samplings;
	}
	if (sequence->buffer == NULL || count * sizeof(uint16_t) > sequence->buffer_size) {
		err = -ENOMEM;
		goto out;
	}
	unsigned int key = irq_lock();

	data->buffer = sequence->buffer;
	data->samples = 1U << sequence->oversampling;
	data->active = true;
	adc_context_start_read(&data->ctx, sequence);
	irq_unlock(key);
	err = adc_context_wait_for_completion(&data->ctx);
out:
	adc_context_release(&data->ctx, err);
	return err;
}

static int adc_stm8_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return adc_stm8_start_read(dev, sequence, false, NULL);
}

#ifdef CONFIG_ADC_ASYNC
static int adc_stm8_read_async(const struct device *dev, const struct adc_sequence *sequence,
			       struct k_poll_signal *signal)
{
	return adc_stm8_start_read(dev, sequence, true, signal);
}
#endif

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static int adc_stm8_clock_prepare(const struct device *dev, uint32_t rate)
{
	struct adc_stm8_data *data = dev->data;

	return data->active ? -EBUSY : adc_stm8_prescaler(rate) < 0 ? -EINVAL : 0;
}

static void adc_stm8_clock_changed(const struct device *dev, uint32_t rate)
{
	const struct adc_stm8_config *cfg = dev->config;
	struct adc_stm8_data *data = dev->data;

	data->control = adc_stm8_prescaler(rate);
	sys_write8(data->control, cfg->base + STM8_ADC_CR1);
}
#endif

static int adc_stm8_init(const struct device *dev)
{
	const struct adc_stm8_config *cfg = dev->config;
	struct adc_stm8_data *data = dev->data;
	uint32_t rate;
	int err;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}
	err = clock_control_get_rate(cfg->clock, cfg->clock_id, &rate);
	if (err != 0) {
		return err;
	}
	err = adc_stm8_prescaler(rate);
	if (err < 0) {
		return err;
	}
	data->control = err;
	err = clock_control_on(cfg->clock, cfg->clock_id);
	if (err != 0) {
		return err;
	}
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}
	data->dev = dev;
	adc_context_init(&data->ctx);
	sys_write8(0U, cfg->base + STM8_ADC_CSR);
	sys_write8(data->control, cfg->base + STM8_ADC_CR1);
	sys_write8(STM8_ADC_ALIGN_RIGHT, cfg->base + STM8_ADC_CR2);
	cfg->irq_config(dev);
	adc_context_unlock_unconditionally(&data->ctx);
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	data->clock_client.clock_id = STM8_CLOCK_MASTER;
	data->clock_client.dev = dev;
	data->clock_client.prepare = adc_stm8_clock_prepare;
	data->clock_client.changed = adc_stm8_clock_changed;
	return clock_control_stm8_register_client(&data->clock_client);
#else
	return 0;
#endif
}

static DEVICE_API(adc, adc_stm8_api) = {
	.channel_setup = adc_stm8_channel_setup,
	.read = adc_stm8_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_stm8_read_async,
#endif
};

#define ADC_STM8_DEFINE(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void adc_stm8_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		intc_stm8_irq_register(DT_INST_IRQN(n), dev, adc_stm8_irq_set_enabled);            \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), adc_stm8_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static struct adc_stm8_data adc_stm8_data_##n;                                             \
	static const struct adc_stm8_config adc_stm8_config_##n = {                                \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.channel_mask = DT_INST_PROP(n, st_channel_mask),                                  \
		.irq_config = adc_stm8_irq_config_##n,                                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, adc_stm8_init, NULL, &adc_stm8_data_##n, &adc_stm8_config_##n,    \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &adc_stm8_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_STM8_DEFINE)
