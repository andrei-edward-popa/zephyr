/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_counter

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/drivers/pinctrl.h>

#define STM8_TIM_CR1            0U
#define STM8_TIM_CR1_CEN        0x01U
#define STM8_TIM_IER_UIE        0x01U
#define STM8_TIM_SR_UIF         0x01U
#define STM8_TIM_EGR_UG         0x01U
#define STM8_TIM1_IER           4U
#define STM8_TIM1_SR            5U
#define STM8_TIM1_EGR           7U
#define STM8_TIM1_CNTR          14U
#define STM8_TIM1_PSCR          16U
#define STM8_TIM1_ARR           18U
#define STM8_TIM2_IER           1U
#define STM8_TIM2_SR            2U
#define STM8_TIM2_EGR           4U
#define STM8_TIM2_CNTR          10U
#define STM8_TIM2_PSCR          12U
#define STM8_TIM2_ARR           13U
#define STM8_TIM3_IER           1U
#define STM8_TIM3_SR            2U
#define STM8_TIM3_EGR           4U
#define STM8_TIM3_CNTR          8U
#define STM8_TIM3_PSCR          10U
#define STM8_TIM3_ARR           11U
#define STM8_TIM4_IER           1U
#define STM8_TIM4_SR            2U
#define STM8_TIM4_EGR           3U
#define STM8_TIM4_CNTR          4U
#define STM8_TIM4_PSCR          5U
#define STM8_TIM4_ARR           6U
#define STM8_TIM1_CCMR          8U
#define STM8_TIM2_CCMR          5U
#define STM8_TIM3_CCMR          5U
#define STM8_TIM4_CCMR          0U
#define STM8_TIM1_CCER          12U
#define STM8_TIM2_CCER          8U
#define STM8_TIM3_CCER          7U
#define STM8_TIM4_CCER          0U
#define STM8_TIM1_CCR           21U
#define STM8_TIM2_CCR           15U
#define STM8_TIM3_CCR           13U
#define STM8_TIM4_CCR           0U
#define STM8_TIM_CC_MASK        0x1eU
#define STM8_TIM_CC_ENABLE      0x01U
#define STM8_TIM_CC_POLARITY    0x02U
#define STM8_TIM_CC_SHIFT       4U
#define STM8_TIM_CCMR_DIRECT    0x01U
#define STM8_TIM_CHANNELS_MAX   4U
#define STM8_TIM4_PRESCALER_MAX 128U
#define STM8_TIM_OFFSET(id, reg)                                                                   \
	((id) == 1U   ? STM8_TIM1_##reg                                                            \
	 : (id) == 2U ? STM8_TIM2_##reg                                                            \
	 : (id) == 3U ? STM8_TIM3_##reg                                                            \
		      : STM8_TIM4_##reg)

static inline uint16_t counter_stm8_read(uintptr_t base, uint8_t offset, bool wide)
{
	uint16_t value = sys_read8(base + offset);

	if (wide) {
		/* Reading the high byte latches the low byte of the 16-bit counter. */
		value = (value << 8) | sys_read8(base + offset + 1U);
	}
	return value;
}

static inline void counter_stm8_write(uintptr_t base, uint8_t offset, uint16_t value, bool wide)
{
	if (wide) {
		/* The low-byte write latches the complete 16-bit value. */
		sys_write8(value >> 8, base + offset);
		offset++;
	}
	sys_write8(value, base + offset);
}

struct counter_stm8_config {
	struct counter_config_info info;
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
	uint32_t prescaler;
	uint8_t timer_id;
	uint8_t ier;
	uint8_t sr;
	uint8_t egr;
	uint8_t cntr;
	uint8_t psc;
	uint8_t arr;
	uint8_t ccmr;
	uint8_t ccer;
	uint8_t ccr;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
};

struct counter_stm8_channel {
	counter_alarm_callback_t alarm;
#ifdef CONFIG_COUNTER_CAPTURE
	counter_capture_cb_t capture;
	counter_capture_flags_t flags;
#endif
	void *user_data;
};

struct counter_stm8_data {
	uint8_t interrupt_mask;
	bool vector_enabled;
	bool cc_enabled;
	uint32_t guard;
	struct counter_stm8_channel channels[STM8_TIM_CHANNELS_MAX];
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	struct clock_control_stm8_client clock_client;
#endif
	counter_top_callback_t callback;
	void *user_data;
	uint32_t top;
	uint32_t frequency;
};

static void counter_stm8_interrupt_write(const struct device *dev, uint8_t value)
{
	const struct counter_stm8_config *cfg = dev->config;
	struct counter_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	data->interrupt_mask = value;
	uint8_t enabled = (data->vector_enabled ? STM8_TIM_IER_UIE : 0U) |
			  (data->cc_enabled ? STM8_TIM_CC_MASK : 0U);

	sys_write8(value & enabled, cfg->base + cfg->ier);
	irq_unlock(key);
}

static void counter_stm8_irq_set_enabled(const struct device *dev, bool enabled)
{
	struct counter_stm8_data *data = dev->data;

	data->vector_enabled = enabled;
	counter_stm8_interrupt_write(dev, data->interrupt_mask);
}

static void counter_stm8_cc_irq_set_enabled(const struct device *dev, bool enabled)
{
	struct counter_stm8_data *data = dev->data;

	data->cc_enabled = enabled;
	counter_stm8_interrupt_write(dev, data->interrupt_mask);
}

static void counter_stm8_cc_enable(const struct device *dev, uint8_t channel, bool enable,
				   bool falling)
{
	const struct counter_stm8_config *cfg = dev->config;
	uintptr_t address = cfg->base + cfg->ccer + channel / 2U;
	uint8_t shift = (channel & 1U) * STM8_TIM_CC_SHIFT;
	uint8_t value = sys_read8(address);

	value &= ~((STM8_TIM_CC_ENABLE | STM8_TIM_CC_POLARITY) << shift);
	value |= ((enable ? STM8_TIM_CC_ENABLE : 0U) | (falling ? STM8_TIM_CC_POLARITY : 0U))
		 << shift;
	sys_write8(value, address);
}

static int counter_stm8_cancel_alarm(const struct device *dev, uint8_t channel)
{
	struct counter_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	if (data->channels[channel].alarm == NULL) {
		irq_unlock(key);
		return 0;
	}
	data->channels[channel].alarm = NULL;
	counter_stm8_interrupt_write(dev, data->interrupt_mask & ~BIT(channel + 1U));
	irq_unlock(key);
	return 0;
}

static int counter_stm8_set_alarm(const struct device *dev, uint8_t channel,
				  const struct counter_alarm_cfg *alarm)
{
	const struct counter_stm8_config *cfg = dev->config;
	struct counter_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	uint8_t mask = BIT(channel + 1U);
	uint32_t now;
	uint32_t target;
	uint32_t distance;
	uint32_t after;
	uint32_t elapsed;
	int err = 0;

	if (alarm->ticks > data->top || alarm->callback == NULL ||
	    (alarm->flags & ~(COUNTER_ALARM_CFG_ABSOLUTE | COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE)) !=
		    0U) {
		err = -EINVAL;
		goto out;
	}
	if ((data->interrupt_mask & mask) != 0U) {
		err = -EBUSY;
		goto out;
	}
	now = counter_stm8_read(cfg->base, cfg->cntr, true);
	if ((alarm->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0U) {
		target = alarm->ticks;
		distance = target >= now ? target - now : data->top + 1U - now + target;
	} else {
		distance = alarm->ticks;
		target = now + distance;
		if (target > data->top) {
			target -= data->top + 1U;
		}
	}
	counter_stm8_cc_enable(dev, channel, false, false);
	sys_write8(0U, cfg->base + cfg->ccmr + channel);
	sys_write8((uint8_t)~mask, cfg->base + cfg->sr);
	counter_stm8_write(cfg->base, cfg->ccr + 2U * channel, target, true);
#ifdef CONFIG_COUNTER_CAPTURE
	data->channels[channel].capture = NULL;
#endif
	data->channels[channel].alarm = alarm->callback;
	data->channels[channel].user_data = alarm->user_data;
	counter_stm8_interrupt_write(dev, data->interrupt_mask | mask);
	after = counter_stm8_read(cfg->base, cfg->cntr, true);
	elapsed = after >= now ? after - now : data->top + 1U - now + after;
	if ((distance <= elapsed ||
	     ((alarm->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0U &&
	      distance > data->top - data->guard)) &&
	    (sys_read8(cfg->base + cfg->sr) & mask) == 0U) {
		if ((alarm->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0U) {
			err = -ETIME;
		}
		if ((alarm->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0U ||
		    (alarm->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) != 0U) {
			sys_write8(mask, cfg->base + cfg->egr);
		} else {
			counter_stm8_cancel_alarm(dev, channel);
		}
	}
out:
	irq_unlock(key);
	return err;
}

static uint32_t counter_stm8_get_guard(const struct device *dev, uint32_t flags)
{
	const struct counter_stm8_data *data = dev->data;

	ARG_UNUSED(flags);
	unsigned int key = irq_lock();
	uint32_t value = data->guard;

	irq_unlock(key);
	return value;
}

static int counter_stm8_set_guard(const struct device *dev, uint32_t ticks, uint32_t flags)
{
	struct counter_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	if (flags != COUNTER_GUARD_PERIOD_LATE_TO_SET || ticks > data->top) {
		irq_unlock(key);
		return -EINVAL;
	}

	data->guard = ticks;
	irq_unlock(key);
	return 0;
}

#ifdef CONFIG_COUNTER_CAPTURE
static int counter_stm8_capture_configure(const struct device *dev, uint8_t channel,
					  counter_capture_flags_t flags,
					  counter_capture_cb_t callback, void *user_data)
{
	const struct counter_stm8_config *cfg = dev->config;
	struct counter_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	uint32_t edge = flags & COUNTER_CAPTURE_BOTH_EDGES;
	int err = 0;

	if (edge == 0U ||
	    (flags & ~(COUNTER_CAPTURE_BOTH_EDGES | COUNTER_CAPTURE_SINGLE_SHOT)) != 0U) {
		err = -EINVAL;
	} else if (edge == COUNTER_CAPTURE_BOTH_EDGES) {
		/* Each channel has one edge selector; use two channels for both edges. */
		err = -ENOTSUP;
	} else if ((data->interrupt_mask & BIT(channel + 1U)) != 0U) {
		err = -EBUSY;
	} else {
		counter_stm8_cc_enable(dev, channel, false, edge == COUNTER_CAPTURE_FALLING_EDGE);
		sys_write8(STM8_TIM_CCMR_DIRECT, cfg->base + cfg->ccmr + channel);
		data->channels[channel].alarm = NULL;
		data->channels[channel].capture = callback;
		data->channels[channel].flags = flags;
		data->channels[channel].user_data = user_data;
	}
	irq_unlock(key);
	return err;
}

static int counter_stm8_capture_enable(const struct device *dev, uint8_t channel)
{
	const struct counter_stm8_config *cfg = dev->config;
	struct counter_stm8_data *data = dev->data;
	struct counter_stm8_channel *state = &data->channels[channel];
	uint8_t mask = BIT(channel + 1U);
	unsigned int key = irq_lock();
	int err = 0;

	if ((data->interrupt_mask & mask) != 0U) {
		err = -EBUSY;
	} else if (state->capture == NULL) {
		err = -EINVAL;
	} else {
		sys_write8((uint8_t)~mask, cfg->base + cfg->sr);
		sys_write8((uint8_t)~mask, cfg->base + cfg->sr + 1U);
		counter_stm8_interrupt_write(dev, data->interrupt_mask | mask);
		counter_stm8_cc_enable(dev, channel, true,
				       (state->flags & COUNTER_CAPTURE_FALLING_EDGE) != 0U);
	}
	irq_unlock(key);
	return err;
}

static int counter_stm8_capture_disable(const struct device *dev, uint8_t channel)
{
	struct counter_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	if (data->channels[channel].alarm != NULL) {
		irq_unlock(key);
		return -EBUSY;
	}
	counter_stm8_cc_enable(dev, channel, false, false);
	counter_stm8_interrupt_write(dev, data->interrupt_mask & ~BIT(channel + 1U));
	irq_unlock(key);
	return 0;
}
#endif

static int counter_stm8_start(const struct device *dev)
{
	const struct counter_stm8_config *cfg = dev->config;

	sys_write8(sys_read8(cfg->base + STM8_TIM_CR1) | STM8_TIM_CR1_CEN, cfg->base);
	return 0;
}

static int counter_stm8_stop(const struct device *dev)
{
	const struct counter_stm8_config *cfg = dev->config;

	sys_write8(sys_read8(cfg->base + STM8_TIM_CR1) & ~STM8_TIM_CR1_CEN, cfg->base);
	return 0;
}

static int counter_stm8_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_stm8_config *cfg = dev->config;
	unsigned int key = irq_lock();

	*ticks = counter_stm8_read(cfg->base, cfg->cntr, cfg->timer_id != 4U);
	irq_unlock(key);
	return 0;
}

static int counter_stm8_set_top_value(const struct device *dev, const struct counter_top_cfg *top)
{
	const struct counter_stm8_config *cfg = dev->config;
	struct counter_stm8_data *data = dev->data;

	if (top->ticks == 0U || top->ticks > cfg->info.max_top_value) {
		return -EINVAL;
	}
	/* Live top updates cannot guarantee that the new limit has not passed. */
	if ((top->flags & COUNTER_TOP_CFG_DONT_RESET) != 0U) {
		return -ENOTSUP;
	}
	unsigned int key = irq_lock();

	if ((data->interrupt_mask & STM8_TIM_CC_MASK) != 0U) {
		irq_unlock(key);
		return -EBUSY;
	}
	uint8_t cr1 = sys_read8(cfg->base);

	sys_write8(cr1 & ~STM8_TIM_CR1_CEN, cfg->base);
	counter_stm8_interrupt_write(dev, 0);
	counter_stm8_write(cfg->base, cfg->arr, top->ticks, cfg->timer_id != 4U);
	counter_stm8_write(cfg->base, cfg->cntr, 0, cfg->timer_id != 4U);
	sys_write8(STM8_TIM_EGR_UG, cfg->base + cfg->egr);
	sys_write8(0, cfg->base + cfg->sr);
	data->callback = top->callback;
	data->user_data = top->user_data;
	data->top = top->ticks;
	data->guard = MIN(data->guard, data->top);
	counter_stm8_interrupt_write(dev, top->callback != NULL);
	sys_write8(cr1, cfg->base);
	irq_unlock(key);
	return 0;
}

static uint32_t counter_stm8_get_pending_int(const struct device *dev)
{
	const struct counter_stm8_config *cfg = dev->config;

	uint8_t mask = STM8_TIM_SR_UIF | (cfg->info.channels > 0U ? STM8_TIM_CC_MASK : 0U);

	return sys_read8(cfg->base + cfg->sr) & mask;
}

static uint32_t counter_stm8_get_top_value(const struct device *dev)
{
	struct counter_stm8_data *data = dev->data;

	unsigned int key = irq_lock();
	uint32_t value = data->top;

	irq_unlock(key);
	return value;
}

static void counter_stm8_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct counter_stm8_config *cfg = dev->config;
	struct counter_stm8_data *data = dev->data;
	uint8_t pending = sys_read8(cfg->base + cfg->sr) & sys_read8(cfg->base + cfg->ier);

	if ((pending & STM8_TIM_SR_UIF) != 0U) {
		sys_write8((uint8_t)~STM8_TIM_SR_UIF, cfg->base + cfg->sr);
		if (data->callback != NULL) {
			data->callback(dev, data->user_data);
		}
	}
	for (uint8_t i = 0U; i < cfg->info.channels; i++) {
		uint8_t mask = BIT(i + 1U);
		struct counter_stm8_channel *state = &data->channels[i];

		if ((pending & mask) == 0U) {
			continue;
		}
		uint32_t ticks = counter_stm8_read(cfg->base, cfg->ccr + 2U * i, true);

		if (state->alarm != NULL) {
			counter_alarm_callback_t callback = state->alarm;
			void *user_data = state->user_data;

			sys_write8((uint8_t)~mask, cfg->base + cfg->sr);
			counter_stm8_cancel_alarm(dev, i);
			callback(dev, i, ticks, user_data);
#ifdef CONFIG_COUNTER_CAPTURE
		} else if (state->capture != NULL) {
			counter_capture_flags_t flags = state->flags;

			/* Reading CCR clears CCIF; clear any overcapture indication separately. */
			sys_write8((uint8_t)~mask, cfg->base + cfg->sr + 1U);
			if ((flags & COUNTER_CAPTURE_SINGLE_SHOT) != 0U) {
				counter_stm8_capture_disable(dev, i);
			}
			state->capture(dev, i, flags & COUNTER_CAPTURE_BOTH_EDGES, ticks,
				       state->user_data);
#endif
		}
	}
}

static uint32_t counter_stm8_get_freq(const struct device *dev)
{
	const struct counter_stm8_data *data = dev->data;

	unsigned int key = irq_lock();
	uint32_t value = data->frequency;

	irq_unlock(key);
	return value;
}

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static int counter_stm8_clock_prepare(const struct device *dev, uint32_t rate)
{
	const struct counter_stm8_config *cfg = dev->config;

	ARG_UNUSED(rate);
	return (sys_read8(cfg->base + STM8_TIM_CR1) & STM8_TIM_CR1_CEN) != 0U ? -EBUSY : 0;
}

static void counter_stm8_clock_changed(const struct device *dev, uint32_t rate)
{
	const struct counter_stm8_config *cfg = dev->config;
	struct counter_stm8_data *data = dev->data;

	data->frequency = rate / cfg->prescaler;
}
#endif

static int counter_stm8_init(const struct device *dev)
{
	const struct counter_stm8_config *cfg = dev->config;
	struct counter_stm8_data *data = dev->data;
	uint32_t divisor = cfg->prescaler;
	uint32_t rate;
	int err;

	if (divisor == 0U || divisor > 65536UL ||
	    (cfg->timer_id != 1U && ((divisor & (divisor - 1U)) != 0U || divisor > 32768UL)) ||
	    (cfg->timer_id == 4U && divisor > STM8_TIM4_PRESCALER_MAX)) {
		return -EINVAL;
	}
	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}
	err = clock_control_get_rate(cfg->clock, cfg->clock_id, &rate);
	if (err != 0) {
		return err;
	}
	err = clock_control_on(cfg->clock, cfg->clock_id);
	if (err != 0) {
		return err;
	}
	if (cfg->pcfg != NULL) {
		err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (err != 0) {
			return err;
		}
	}
	data->frequency = rate / divisor;
	sys_write8(0, cfg->base);
	counter_stm8_interrupt_write(dev, 0);
	if (cfg->timer_id == 1U) {
		counter_stm8_write(cfg->base, cfg->psc, divisor - 1U, cfg->timer_id != 4U);
	} else {
		sys_write8(u32_count_trailing_zeros(divisor), cfg->base + cfg->psc);
	}
	counter_stm8_write(cfg->base, cfg->arr, cfg->info.max_top_value, cfg->timer_id != 4U);
	counter_stm8_write(cfg->base, cfg->cntr, 0, cfg->timer_id != 4U);
	sys_write8(STM8_TIM_EGR_UG, cfg->base + cfg->egr);
	sys_write8(0, cfg->base + cfg->sr);
	data->top = cfg->info.max_top_value;
	cfg->irq_config(dev);
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	data->clock_client.clock_id = STM8_CLOCK_MASTER;
	data->clock_client.dev = dev;
	data->clock_client.prepare = counter_stm8_clock_prepare;
	data->clock_client.changed = counter_stm8_clock_changed;
	return clock_control_stm8_register_client(&data->clock_client);
#else
	return 0;
#endif
}

static DEVICE_API(counter, counter_stm8_driver_api) = {
	.start = counter_stm8_start,
	.set_alarm = counter_stm8_set_alarm,
	.cancel_alarm = counter_stm8_cancel_alarm,
	.get_guard_period = counter_stm8_get_guard,
	.set_guard_period = counter_stm8_set_guard,
#ifdef CONFIG_COUNTER_CAPTURE
	.capture_configure = counter_stm8_capture_configure,
	.enable_capture = counter_stm8_capture_enable,
	.disable_capture = counter_stm8_capture_disable,
#endif
	.stop = counter_stm8_stop,
	.get_value = counter_stm8_get_value,
	.set_top_value = counter_stm8_set_top_value,
	.get_pending_int = counter_stm8_get_pending_int,
	.get_top_value = counter_stm8_get_top_value,
	.get_freq = counter_stm8_get_freq,
};

#define COUNTER_STM8_TIMER_ID(n) DT_PROP(DT_PARENT(DT_DRV_INST(n)), st_timer_id)

#define COUNTER_STM8_PARENT(n) DT_PARENT(DT_DRV_INST(n))
#define COUNTER_STM8_CC_IRQ(n) DT_IRQ_BY_NAME(COUNTER_STM8_PARENT(n), cc, irq)
#define COUNTER_STM8_CC_CONNECT(n)                                                                 \
	intc_stm8_irq_register(COUNTER_STM8_CC_IRQ(n), DEVICE_DT_INST_GET(n),                      \
			       counter_stm8_cc_irq_set_enabled);                                   \
	IRQ_CONNECT(COUNTER_STM8_CC_IRQ(n), DT_IRQ_BY_NAME(COUNTER_STM8_PARENT(n), cc, priority),  \
		    counter_stm8_isr, DEVICE_DT_INST_GET(n), 0);                                   \
	irq_enable(COUNTER_STM8_CC_IRQ(n));

#define COUNTER_STM8_CC_CONFIG(n)                                                                  \
	COND_CODE_1(IS_EQ(COUNTER_STM8_TIMER_ID(n), 4), (), (COUNTER_STM8_CC_CONNECT(n)))

#define COUNTER_STM8_PINCTRL(n)                                                                    \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0), \
		    (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL))

#define COUNTER_STM8_DEFINE(n)                                                                     \
	BUILD_ASSERT(!DT_PROP(COUNTER_STM8_PARENT(n), st_system_timer),                            \
		     "System timer and counter cannot own the same timer");                        \
	BUILD_ASSERT(!DT_NODE_HAS_STATUS_OKAY(DT_CHILD(COUNTER_STM8_PARENT(n), pwm)),              \
		     "Counter and PWM cannot own the same timer");                                 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),                                           \
		    (PINCTRL_DT_INST_DEFINE(n);), ())                                              \
	static void counter_stm8_irq_config_##n(const struct device *dev)                          \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		intc_stm8_irq_register(DT_IRQ_BY_NAME(DT_PARENT(DT_DRV_INST(n)), update, irq),     \
				       DEVICE_DT_INST_GET(n), counter_stm8_irq_set_enabled);       \
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_PARENT(DT_DRV_INST(n)), update, irq),                \
			    DT_IRQ_BY_NAME(DT_PARENT(DT_DRV_INST(n)), update, priority),           \
			    counter_stm8_isr, DEVICE_DT_INST_GET(n), 0);                           \
		COUNTER_STM8_CC_CONFIG(n)                                                          \
		irq_enable(DT_IRQ_BY_NAME(DT_PARENT(DT_DRV_INST(n)), update, irq));                \
	}                                                                                          \
	static struct counter_stm8_data counter_stm8_data_##n;                                     \
	static const struct counter_stm8_config counter_stm8_config_##n = {                        \
		.info =                                                                            \
			{                                                                          \
				.freq = 0U,                                                        \
				.max_top_value = COUNTER_STM8_TIMER_ID(n) == 4 ? 255UL : 65535UL,  \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = COUNTER_STM8_TIMER_ID(n) == 4U                         \
						    ? 0U                                           \
						    : 5U - COUNTER_STM8_TIMER_ID(n),               \
			},                                                                         \
		.base = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(n))),                                    \
		.clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_PARENT(DT_DRV_INST(n)))),                 \
		.clock_id = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_PARENT(DT_DRV_INST(n)), id), \
		.prescaler = DT_PROP(DT_PARENT(DT_DRV_INST(n)), st_prescaler),                     \
		.timer_id = COUNTER_STM8_TIMER_ID(n),                                              \
		.ier = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), IER),                             \
		.sr = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), SR),                               \
		.egr = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), EGR),                             \
		.cntr = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), CNTR),                           \
		.psc = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), PSCR),                            \
		.arr = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), ARR),                             \
		.ccmr = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), CCMR),                           \
		.ccer = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), CCER),                           \
		.ccr = STM8_TIM_OFFSET(COUNTER_STM8_TIMER_ID(n), CCR),                             \
		.pcfg = COUNTER_STM8_PINCTRL(n),                                                   \
		.irq_config = counter_stm8_irq_config_##n,                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, counter_stm8_init, NULL, &counter_stm8_data_##n,                  \
			      &counter_stm8_config_##n, PRE_KERNEL_1,                              \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_stm8_driver_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_STM8_DEFINE)
