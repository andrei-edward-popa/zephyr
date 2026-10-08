/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_pwm

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/irq.h>
#include <zephyr/sys/math_extras.h>

#define STM8_PWM_CR1        0U
#define STM8_PWM_CEN        0x01U
#define STM8_PWM_ARPE       0x80U
#define STM8_PWM_UG         0x01U
#define STM8_PWM_MODE_LOW   0x40U
#define STM8_PWM_MODE_HIGH  0x50U
#define STM8_PWM_MODE_PWM1  0x60U
#define STM8_PWM_PRELOAD    0x08U
#define STM8_PWM_CC_ENABLE  0x01U
#define STM8_PWM_CC_INVERT  0x02U
#define STM8_PWM_CC_SHIFT   4U
#define STM8_PWM_TIM1_BKR   29U
#define STM8_PWM_TIM1_MOE   0x80U
#define STM8_PWM_PERIOD_MAX 65536UL

#define STM8_PWM_TIM1_EGR 7U
#define STM8_PWM_TIM2_EGR 4U
#define STM8_PWM_TIM3_EGR 4U

#define STM8_PWM_TIM1_CCMR 8U
#define STM8_PWM_TIM2_CCMR 5U
#define STM8_PWM_TIM3_CCMR 5U

#define STM8_PWM_TIM1_CCER 12U
#define STM8_PWM_TIM2_CCER 8U
#define STM8_PWM_TIM3_CCER 7U

#define STM8_PWM_TIM1_PSCR 16U
#define STM8_PWM_TIM2_PSCR 12U
#define STM8_PWM_TIM3_PSCR 10U

#define STM8_PWM_TIM1_ARR 18U
#define STM8_PWM_TIM2_ARR 13U
#define STM8_PWM_TIM3_ARR 11U

#define STM8_PWM_TIM1_CCR 21U
#define STM8_PWM_TIM2_CCR 15U
#define STM8_PWM_TIM3_CCR 13U

#define STM8_PWM_TIM1_CHANNELS 4U
#define STM8_PWM_TIM2_CHANNELS 3U
#define STM8_PWM_TIM3_CHANNELS 2U

struct pwm_stm8_config {
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
	const struct pinctrl_dev_config *pcfg;
	uint32_t prescaler;
	uint8_t timer_id;
	uint8_t channels;
	uint8_t egr;
	uint8_t ccmr;
	uint8_t ccer;
	uint8_t psc;
	uint8_t arr;
	uint8_t ccr;
};

struct pwm_stm8_data {
	uint32_t frequency;
	uint32_t period;
	uint8_t active;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	struct clock_control_stm8_client clock_client;
#endif
};

static void pwm_stm8_write16(uintptr_t address, uint16_t value)
{
	sys_write8(value >> 8, address);
	sys_write8(value, address + 1U);
}

static int pwm_stm8_set_cycles(const struct device *dev, uint32_t channel, uint32_t period,
			       uint32_t pulse, pwm_flags_t flags)
{
	const struct pwm_stm8_config *cfg = dev->config;
	struct pwm_stm8_data *data = dev->data;
	unsigned int key;
	uint8_t mode;
	uint8_t shift;
	uint8_t ccer;
	uintptr_t enable_reg;

	/* Timer channel numbers in DT are one-based, matching the reference manual. */
	if (channel == 0U || channel > cfg->channels || pulse > period ||
	    period > STM8_PWM_PERIOD_MAX) {
		return -EINVAL;
	}
	if ((flags & ~PWM_POLARITY_INVERTED) != 0U) {
		return -ENOTSUP;
	}
	channel--;
	key = irq_lock();
	if (period != 0U && data->period != period && (data->active & ~BIT(channel)) != 0U) {
		irq_unlock(key);
		return -EBUSY;
	}
	shift = (channel & 1U) * STM8_PWM_CC_SHIFT;
	enable_reg = cfg->base + cfg->ccer + channel / 2U;
	ccer = sys_read8(enable_reg) & ~(STM8_PWM_CC_ENABLE << shift);
	sys_write8(ccer, enable_reg);
	if (period == 0U) {
		data->active &= ~BIT(channel);
		if (data->active == 0U) {
			sys_write8(STM8_PWM_ARPE, cfg->base + STM8_PWM_CR1);
		}
		irq_unlock(key);
		return 0;
	}
	bool reset = data->period != period || data->active == 0U;

	if (reset) {
		sys_write8(STM8_PWM_ARPE, cfg->base + STM8_PWM_CR1);
		pwm_stm8_write16(cfg->base + cfg->arr, period - 1U);
		data->period = period;
	}
	mode = pulse == 0U       ? STM8_PWM_MODE_LOW
	       : pulse == period ? STM8_PWM_MODE_HIGH
				 : STM8_PWM_MODE_PWM1;
	/* Forced levels also represent 100 percent duty for a 65536-cycle period. */
	pwm_stm8_write16(cfg->base + cfg->ccr + 2U * channel, pulse);
	sys_write8(mode | STM8_PWM_PRELOAD, cfg->base + cfg->ccmr + channel);
	if (reset) {
		/* UG transfers buffered ARR/CCR and initializes the prescaler. */
		sys_write8(STM8_PWM_UG, cfg->base + cfg->egr);
	}
	ccer &= ~(STM8_PWM_CC_INVERT << shift);
	ccer |= (STM8_PWM_CC_ENABLE |
		 ((flags & PWM_POLARITY_INVERTED) != 0U ? STM8_PWM_CC_INVERT : 0U))
		<< shift;
	sys_write8(ccer, enable_reg);
	data->active |= BIT(channel);
	sys_write8(STM8_PWM_ARPE | STM8_PWM_CEN, cfg->base + STM8_PWM_CR1);
	irq_unlock(key);
	return 0;
}

static int pwm_stm8_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
	const struct pwm_stm8_config *cfg = dev->config;
	struct pwm_stm8_data *data = dev->data;

	if (channel == 0U || channel > cfg->channels) {
		return -EINVAL;
	}
	unsigned int key = irq_lock();

	*cycles = data->frequency;
	irq_unlock(key);
	return 0;
}

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static int pwm_stm8_clock_prepare(const struct device *dev, uint32_t rate)
{
	struct pwm_stm8_data *data = dev->data;

	ARG_UNUSED(rate);
	return data->active != 0U ? -EBUSY : 0;
}

static void pwm_stm8_clock_changed(const struct device *dev, uint32_t rate)
{
	const struct pwm_stm8_config *cfg = dev->config;
	struct pwm_stm8_data *data = dev->data;

	data->frequency = rate / cfg->prescaler;
}
#endif

static int pwm_stm8_init(const struct device *dev)
{
	const struct pwm_stm8_config *cfg = dev->config;
	struct pwm_stm8_data *data = dev->data;
	uint32_t rate;
	int err;

	if (cfg->timer_id == 4U || cfg->prescaler == 0U || cfg->prescaler > STM8_PWM_PERIOD_MAX ||
	    (cfg->timer_id != 1U &&
	     (!is_power_of_two(cfg->prescaler) || cfg->prescaler > 32768UL))) {
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
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}
	data->frequency = rate / cfg->prescaler;
	sys_write8(STM8_PWM_ARPE, cfg->base + STM8_PWM_CR1);
	if (cfg->timer_id == 1U) {
		pwm_stm8_write16(cfg->base + cfg->psc, cfg->prescaler - 1U);
		sys_write8(STM8_PWM_TIM1_MOE, cfg->base + STM8_PWM_TIM1_BKR);
	} else {
		sys_write8(u32_count_trailing_zeros(cfg->prescaler), cfg->base + cfg->psc);
	}
	sys_write8(0U, cfg->base + cfg->ccer);
	if (cfg->channels > 2U) {
		sys_write8(0U, cfg->base + cfg->ccer + 1U);
	}
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	data->clock_client.clock_id = STM8_CLOCK_MASTER;
	data->clock_client.dev = dev;
	data->clock_client.prepare = pwm_stm8_clock_prepare;
	data->clock_client.changed = pwm_stm8_clock_changed;
	return clock_control_stm8_register_client(&data->clock_client);
#else
	return 0;
#endif
}

static DEVICE_API(pwm, pwm_stm8_api) = {
	.set_cycles = pwm_stm8_set_cycles,
	.get_cycles_per_sec = pwm_stm8_get_cycles_per_sec,
};

#define PWM_STM8_PARENT(n) DT_PARENT(DT_DRV_INST(n))
#define PWM_STM8_ID(n)     DT_PROP(PWM_STM8_PARENT(n), st_timer_id)
#define PWM_STM8_OFFSET(n, tim1, tim2, tim3)                                                       \
	(PWM_STM8_ID(n) == 1U ? (tim1) : PWM_STM8_ID(n) == 2U ? (tim2) : (tim3))

#define PWM_STM8_DEFINE(n)                                                                         \
	BUILD_ASSERT(PWM_STM8_ID(n) < 4U, "TIM4 has no capture/compare outputs");                  \
	BUILD_ASSERT(!DT_NODE_HAS_STATUS_OKAY(DT_CHILD(PWM_STM8_PARENT(n), counter)),              \
		     "Counter and PWM cannot own the same timer");                                 \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct pwm_stm8_data pwm_stm8_data_##n;                                             \
	static const struct pwm_stm8_config pwm_stm8_config_##n = {                                \
		.base = DT_REG_ADDR(PWM_STM8_PARENT(n)),                                           \
		.clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(PWM_STM8_PARENT(n))),                        \
		.clock_id = (clock_control_subsys_t)DT_CLOCKS_CELL(PWM_STM8_PARENT(n), id),        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.prescaler = DT_PROP(PWM_STM8_PARENT(n), st_prescaler),                            \
		.timer_id = PWM_STM8_ID(n),                                                        \
		.channels = PWM_STM8_OFFSET(n, STM8_PWM_TIM1_CHANNELS, STM8_PWM_TIM2_CHANNELS,     \
					    STM8_PWM_TIM3_CHANNELS),                               \
		.egr = PWM_STM8_OFFSET(n, STM8_PWM_TIM1_EGR, STM8_PWM_TIM2_EGR,                    \
				       STM8_PWM_TIM3_EGR),                                         \
		.ccmr = PWM_STM8_OFFSET(n, STM8_PWM_TIM1_CCMR, STM8_PWM_TIM2_CCMR,                 \
					STM8_PWM_TIM3_CCMR),                                       \
		.ccer = PWM_STM8_OFFSET(n, STM8_PWM_TIM1_CCER, STM8_PWM_TIM2_CCER,                 \
					STM8_PWM_TIM3_CCER),                                       \
		.psc = PWM_STM8_OFFSET(n, STM8_PWM_TIM1_PSCR, STM8_PWM_TIM2_PSCR,                  \
				       STM8_PWM_TIM3_PSCR),                                        \
		.arr = PWM_STM8_OFFSET(n, STM8_PWM_TIM1_ARR, STM8_PWM_TIM2_ARR,                    \
				       STM8_PWM_TIM3_ARR),                                         \
		.ccr = PWM_STM8_OFFSET(n, STM8_PWM_TIM1_CCR, STM8_PWM_TIM2_CCR,                    \
				       STM8_PWM_TIM3_CCR),                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, pwm_stm8_init, NULL, &pwm_stm8_data_##n, &pwm_stm8_config_##n,    \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &pwm_stm8_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_STM8_DEFINE)
