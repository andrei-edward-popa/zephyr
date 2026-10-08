/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>

#define STM8_TIM_CR1            0U
#define STM8_TIM_CR1_CEN        0x01U
#define STM8_TIM_SR_UIF         0x01U
#define STM8_TIM_EGR_UG         0x01U
#define STM8_TIM4_IER           1U
#define STM8_TIM4_SR            2U
#define STM8_TIM4_EGR           3U
#define STM8_TIM4_CNTR          4U
#define STM8_TIM4_PSCR          5U
#define STM8_TIM4_ARR           6U
#define STM8_TIM4_PRESCALER_MAX 128U

#define STM8_TIM4_TIMER_NODE DT_NODELABEL(tim4)
#define STM8_TIM4_TIMER_BASE DT_REG_ADDR(STM8_TIM4_TIMER_NODE)
#define STM8_TIM4_CR1_REG    (STM8_TIM4_TIMER_BASE + STM8_TIM_CR1)
#define STM8_TIM4_IER_REG    (STM8_TIM4_TIMER_BASE + STM8_TIM4_IER)
#define STM8_TIM4_SR1_REG    (STM8_TIM4_TIMER_BASE + STM8_TIM4_SR)
#define STM8_TIM4_EGR_REG    (STM8_TIM4_TIMER_BASE + STM8_TIM4_EGR)
#define STM8_TIM4_CNTR_REG   (STM8_TIM4_TIMER_BASE + STM8_TIM4_CNTR)
#define STM8_TIM4_PSCR_REG   (STM8_TIM4_TIMER_BASE + STM8_TIM4_PSCR)
#define STM8_TIM4_ARR_REG    (STM8_TIM4_TIMER_BASE + STM8_TIM4_ARR)
#define STM8_TIM4_CYCLES_PER_TICK                                                                  \
	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define STM8_TIM4_PRESCALER_SHIFT LOG2CEIL(DIV_ROUND_UP(STM8_TIM4_CYCLES_PER_TICK, 256UL))
#define STM8_TIM4_PRESCALER       (1UL << STM8_TIM4_PRESCALER_SHIFT)
#define STM8_TIM4_COUNTS_PER_TICK (STM8_TIM4_CYCLES_PER_TICK / STM8_TIM4_PRESCALER)

BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC % CONFIG_SYS_CLOCK_TICKS_PER_SEC == 0U,
	     "Reference cycle frequency must be an integral number of cycles per tick");
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC <= UINT32_MAX / STM8_TIM4_PRESCALER_MAX,
	     "Reference cycle conversion must fit a 32-bit numerator");

#ifdef CONFIG_TICKLESS_KERNEL
#define STM8_TIM4_COUNTER_SPAN 256U
#define STM8_TIM4_PROGRAM_GUARD 2U
#define STM8_TIM4_TICKLESS_SHIFT                                                               \
	MIN(7U, LOG2CEIL(DIV_ROUND_UP(STM8_TIM4_CYCLES_PER_TICK, STM8_TIM4_COUNTER_SPAN / 2U)))
#define STM8_TIM4_TICKLESS_PRESCALER (1UL << STM8_TIM4_TICKLESS_SHIFT)
#define STM8_TIM4_TIME_UNIT_SHIFT                                                               \
	MIN(STM8_TIM4_TICKLESS_SHIFT, __builtin_ctzl((unsigned long)STM8_TIM4_CYCLES_PER_TICK))
#define STM8_TIM4_UNITS_PER_TICK (STM8_TIM4_CYCLES_PER_TICK >> STM8_TIM4_TIME_UNIT_SHIFT)

static uint32_t stm8_tim4_cycle_base;
static uint32_t stm8_tim4_announced;
static uint32_t stm8_tim4_rate;
static uint32_t stm8_tim4_cycles_per_count;
static uint32_t stm8_tim4_remainder;
static uint32_t stm8_tim4_fraction;
static uint16_t stm8_tim4_current_counts;
static uint16_t stm8_tim4_min_counts;
static uint8_t stm8_tim4_shift;

static uint32_t stm8_tim4_ticks_elapsed(uint32_t cycles)
{
	uint32_t delta = cycles - stm8_tim4_announced;

	if (STM8_TIM4_UNITS_PER_TICK <= UINT16_MAX &&
	    (delta >> STM8_TIM4_TIME_UNIT_SHIFT) <= UINT16_MAX) {
		/* The bounded sub-tick interval fits the CPU's hardware DIVW. */
		return (uint16_t)(delta >> STM8_TIM4_TIME_UNIT_SHIFT) /
		       (uint16_t)STM8_TIM4_UNITS_PER_TICK;
	}
	return delta / STM8_TIM4_CYCLES_PER_TICK;
}

static uint32_t stm8_tim4_tick_phase(uint32_t cycles)
{
	uint32_t delta = cycles - stm8_tim4_announced;

	if (STM8_TIM4_UNITS_PER_TICK <= UINT16_MAX &&
	    (delta >> STM8_TIM4_TIME_UNIT_SHIFT) <= UINT16_MAX) {
		uint16_t phase = (uint16_t)(delta >> STM8_TIM4_TIME_UNIT_SHIFT) %
				 (uint16_t)STM8_TIM4_UNITS_PER_TICK;

		return ((uint32_t)phase << STM8_TIM4_TIME_UNIT_SHIFT) |
		       (delta & ((1UL << STM8_TIM4_TIME_UNIT_SHIFT) - 1U));
	}
	return delta % STM8_TIM4_CYCLES_PER_TICK;
}

static void stm8_tim4_irq_set_enabled(const struct device *dev, bool enabled)
{
	ARG_UNUSED(dev);
	sys_write8(enabled ? STM8_TIM_SR_UIF : 0U, STM8_TIM4_IER_REG);
}

static uint32_t stm8_tim4_count_cycles(uint16_t count, uint32_t *fraction)
{
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	uint32_t cycles = (uint32_t)count * stm8_tim4_cycles_per_count;

	if (stm8_tim4_remainder != 0U) {
		uint64_t remainder = (uint64_t)count * stm8_tim4_remainder + *fraction;

		cycles += remainder / stm8_tim4_rate;
		*fraction = remainder % stm8_tim4_rate;
	}
	return cycles;
#else
	ARG_UNUSED(fraction);
	return (uint32_t)count * STM8_TIM4_TICKLESS_PRESCALER;
#endif
}

static uint16_t stm8_tim4_count_get(void)
{
	uint16_t count = sys_read8(STM8_TIM4_CNTR_REG);

	if ((sys_read8(STM8_TIM4_SR1_REG) & STM8_TIM_SR_UIF) != 0U) {
		/* Re-read after rollover; the pending period still belongs to cycle_base. */
		count = stm8_tim4_current_counts + sys_read8(STM8_TIM4_CNTR_REG);
	}
	return count;
}

static uint32_t stm8_tim4_now(void)
{
	uint32_t fraction = stm8_tim4_fraction;

	return stm8_tim4_cycle_base +
		stm8_tim4_count_cycles(stm8_tim4_count_get(), &fraction);
}

static void stm8_tim4_configure(uint32_t rate)
{
	uint8_t shift = 0U;
	uint32_t counts = DIV_ROUND_UP(rate, CONFIG_SYS_CLOCK_TICKS_PER_SEC);

	while (counts > STM8_TIM4_COUNTER_SPAN / 2U && shift < 7U) {
		shift++;
		counts = DIV_ROUND_UP(counts, 2U);
	}
	stm8_tim4_shift = shift;
	stm8_tim4_rate = rate;
	stm8_tim4_min_counts = MIN(counts, STM8_TIM4_COUNTER_SPAN);
	uint32_t numerator = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC << shift;

	stm8_tim4_cycles_per_count = numerator / rate;
	stm8_tim4_remainder = numerator % rate;
	stm8_tim4_fraction = 0U;
	stm8_tim4_current_counts = STM8_TIM4_COUNTER_SPAN;
	sys_write8(shift, STM8_TIM4_PSCR_REG);
	sys_write8(STM8_TIM4_COUNTER_SPAN - 1U, STM8_TIM4_ARR_REG);
	sys_write8(STM8_TIM_EGR_UG, STM8_TIM4_EGR_REG);
	sys_write8(0U, STM8_TIM4_SR1_REG);
	sys_write8(STM8_TIM_CR1_CEN, STM8_TIM4_CR1_REG);
}

void sys_clock_set_timeout(uint32_t ticks, bool idle)
{
	uint16_t count = stm8_tim4_count_get();
	uint32_t fraction = stm8_tim4_fraction;
	uint32_t now = stm8_tim4_cycle_base + stm8_tim4_count_cycles(count, &fraction);

	fraction = stm8_tim4_fraction;
	uint32_t max_cycles = stm8_tim4_count_cycles(STM8_TIM4_COUNTER_SPAN, &fraction);
	uint32_t max_ticks = stm8_tim4_ticks_elapsed(stm8_tim4_announced + max_cycles);
	uint32_t cycles = (uint16_t)MIN(ticks, max_ticks + 1U) * STM8_TIM4_CYCLES_PER_TICK;

	ARG_UNUSED(idle);
	if (ticks != 0U) {
		cycles -= stm8_tim4_tick_phase(now);
	}
	cycles = MIN(cycles, max_cycles);
	uint16_t delay;

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	if (stm8_tim4_remainder == 0U) {
		delay = DIV_ROUND_UP(cycles, stm8_tim4_cycles_per_count);
	} else {
		delay = DIV_ROUND_UP((uint64_t)cycles * stm8_tim4_rate,
			(uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC << stm8_tim4_shift);
	}
#else
	delay = DIV_ROUND_UP(cycles, STM8_TIM4_TICKLESS_PRESCALER);
#endif
	uint16_t end = count + delay;
	uint32_t rollover_fraction = stm8_tim4_fraction;
	uint32_t rollover_cycles =
		stm8_tim4_count_cycles(stm8_tim4_current_counts, &rollover_fraction);

	/* Pause only the register update; retain CNT and the prescaler phase. */
	sys_write8(0U, STM8_TIM4_CR1_REG);
	count = sys_read8(STM8_TIM4_CNTR_REG);

	if ((sys_read8(STM8_TIM4_SR1_REG) & STM8_TIM_SR_UIF) != 0U) {
		stm8_tim4_cycle_base += rollover_cycles;
		stm8_tim4_fraction = rollover_fraction;
		end = end > stm8_tim4_current_counts ? end - stm8_tim4_current_counts : 0U;
		sys_write8(0U, STM8_TIM4_SR1_REG);
	}
	end = MAX(end, MAX(count + STM8_TIM4_PROGRAM_GUARD, stm8_tim4_min_counts));
	stm8_tim4_current_counts = MIN(end, STM8_TIM4_COUNTER_SPAN);
	sys_write8(stm8_tim4_current_counts - 1U, STM8_TIM4_ARR_REG);
	sys_write8(STM8_TIM_CR1_CEN, STM8_TIM4_CR1_REG);
}

uint32_t sys_clock_elapsed(void)
{
	return stm8_tim4_ticks_elapsed(stm8_tim4_now());
}

uint32_t sys_clock_cycle_get_32(void)
{
	unsigned int key = irq_lock();
	uint32_t cycles = stm8_tim4_now();

	irq_unlock(key);
	return cycles;
}

static void stm8_tim4_isr(const void *arg)
{
	k_spinlock_key_t key = sys_clock_lock();

	ARG_UNUSED(arg);
	sys_write8(0U, STM8_TIM4_SR1_REG);
	stm8_tim4_cycle_base +=
		stm8_tim4_count_cycles(stm8_tim4_current_counts, &stm8_tim4_fraction);
	uint32_t ticks = stm8_tim4_ticks_elapsed(stm8_tim4_cycle_base);

	stm8_tim4_announced += ticks * STM8_TIM4_CYCLES_PER_TICK;
	if (ticks != 0U) {
		sys_clock_announce_locked(ticks, key);
	} else {
		sys_clock_unlock(key);
	}
}

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static struct clock_control_stm8_client stm8_tim4_clock_client;

static int stm8_tim4_clock_prepare(const struct device *dev, uint32_t rate)
{
	ARG_UNUSED(dev);
	return rate < CONFIG_SYS_CLOCK_TICKS_PER_SEC ? -EINVAL : 0;
}

static void stm8_tim4_clock_changed(const struct device *dev, uint32_t rate)
{
	ARG_UNUSED(dev);
	if (stm8_tim4_rate == 0U) {
		return;
	}
	sys_write8(0U, STM8_TIM4_CR1_REG);
	stm8_tim4_cycle_base = stm8_tim4_now();
	stm8_tim4_configure(rate);
}
#endif

static int stm8_tim4_init(void)
{
	const struct device *clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(STM8_TIM4_TIMER_NODE));
	clock_control_subsys_t id =
		(clock_control_subsys_t)DT_CLOCKS_CELL(STM8_TIM4_TIMER_NODE, id);
	uint32_t rate;
	int err;

	if (!device_is_ready(clock)) {
		return -ENODEV;
	}
	err = clock_control_get_rate(clock, id, &rate);
	if (err != 0 || rate < CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
		return err != 0 ? err : -EINVAL;
	}
#ifndef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	if (rate != CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) {
		return -EINVAL;
	}
#endif
	err = clock_control_on(clock, id);
	if (err != 0) {
		return err;
	}
	stm8_tim4_configure(rate);
	intc_stm8_irq_register(DT_IRQN(STM8_TIM4_TIMER_NODE), NULL, stm8_tim4_irq_set_enabled);
	IRQ_CONNECT(DT_IRQN(STM8_TIM4_TIMER_NODE), DT_IRQ(STM8_TIM4_TIMER_NODE, priority),
		    stm8_tim4_isr, NULL, 0);
	irq_enable(DT_IRQN(STM8_TIM4_TIMER_NODE));
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	stm8_tim4_clock_client.clock_id = STM8_CLOCK_MASTER;
	stm8_tim4_clock_client.prepare = stm8_tim4_clock_prepare;
	stm8_tim4_clock_client.changed = stm8_tim4_clock_changed;
	return clock_control_stm8_register_client(&stm8_tim4_clock_client);
#else
	return 0;
#endif
}

#else /* CONFIG_TICKLESS_KERNEL */
#ifndef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
BUILD_ASSERT(STM8_TIM4_COUNTS_PER_TICK > 0 && STM8_TIM4_COUNTS_PER_TICK <= 256,
	     "Tick interval must fit the 8-bit TIM4 counter");
BUILD_ASSERT(STM8_TIM4_PRESCALER <= STM8_TIM4_PRESCALER_MAX,
	     "TIM4 prescaler must fit its 3-bit register");
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC %
			     (STM8_TIM4_PRESCALER * CONFIG_SYS_CLOCK_TICKS_PER_SEC) ==
		     0,
	     "TIM4 requires an integral tick period");
#endif
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC % CONFIG_SYS_CLOCK_TICKS_PER_SEC == 0U,
	     "Reference cycle frequency must be an integral number of cycles per tick");
BUILD_ASSERT(!IS_ENABLED(CONFIG_TICKLESS_KERNEL), "TIM4 provides a periodic tick");

static uint32_t stm8_tim4_cycle_base;
static void stm8_tim4_irq_set_enabled(const struct device *dev, bool enabled)
{
	ARG_UNUSED(dev);
	sys_write8(enabled ? STM8_TIM_SR_UIF : 0U, STM8_TIM4_IER_REG);
}

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
struct stm8_tim4_period {
	uint32_t denominator;
	uint32_t remainder;
	uint16_t counts;
	uint8_t shift;
};

static struct stm8_tim4_period stm8_tim4_period;
static uint32_t stm8_tim4_fraction;
static uint32_t stm8_tim4_phase_bias;
static uint16_t stm8_tim4_current_counts;
static bool stm8_tim4_started;
static uint32_t stm8_tim4_cycles_per_count;
static struct clock_control_stm8_client stm8_tim4_clock_client;

static int stm8_tim4_period_get(uint32_t rate, struct stm8_tim4_period *period)
{
	uint8_t shift = 0U;
	uint32_t denominator = CONFIG_SYS_CLOCK_TICKS_PER_SEC;

	if (rate < denominator) {
		return -EINVAL;
	}
	while (DIV_ROUND_UP(rate, denominator) > 256U && shift < 7U) {
		shift++;
		denominator <<= 1;
	}
	if (DIV_ROUND_UP(rate, denominator) > 256U) {
		return -EINVAL;
	}
	period->shift = shift;
	period->denominator = denominator;
	period->counts = rate / denominator;
	period->remainder = rate % denominator;
	return 0;
}

static uint16_t stm8_tim4_next_counts(void)
{
	uint16_t counts = stm8_tim4_period.counts;

	if (stm8_tim4_period.remainder == 0U) {
		return counts;
	}

	stm8_tim4_fraction += stm8_tim4_period.remainder;
	if (stm8_tim4_fraction >= stm8_tim4_period.denominator) {
		stm8_tim4_fraction -= stm8_tim4_period.denominator;
		counts++;
	}
	return counts;
}

static uint32_t stm8_tim4_phase_get(uint8_t count)
{
	if (stm8_tim4_cycles_per_count != 0U) {
		return (uint32_t)count * stm8_tim4_cycles_per_count;
	}
	return (uint32_t)count * STM8_TIM4_CYCLES_PER_TICK / stm8_tim4_current_counts;
}

static void stm8_tim4_scale_update(void)
{
	stm8_tim4_cycles_per_count = STM8_TIM4_CYCLES_PER_TICK % stm8_tim4_current_counts == 0U
					     ? STM8_TIM4_CYCLES_PER_TICK / stm8_tim4_current_counts
					     : 0U;
}

static int stm8_tim4_clock_prepare(const struct device *dev, uint32_t rate)
{
	struct stm8_tim4_period period;

	ARG_UNUSED(dev);
	return stm8_tim4_period_get(rate, &period);
}

static void stm8_tim4_clock_changed(const struct device *dev, uint32_t rate)
{
	ARG_UNUSED(dev);
	if (!stm8_tim4_started) {
		return;
	}
	sys_write8(0U, STM8_TIM4_CR1_REG);
	uint8_t count = sys_read8(STM8_TIM4_CNTR_REG);
	uint8_t pending = sys_read8(STM8_TIM4_SR1_REG) & STM8_TIM_SR_UIF;
	uint32_t phase = stm8_tim4_phase_get(count);

	if (pending == 0U) {
		phase += stm8_tim4_phase_bias;
	}
	(void)stm8_tim4_period_get(rate, &stm8_tim4_period);
	stm8_tim4_fraction = 0U;
	stm8_tim4_current_counts = stm8_tim4_next_counts();
	stm8_tim4_scale_update();
	uint8_t new_count = MIN((phase * stm8_tim4_current_counts / STM8_TIM4_CYCLES_PER_TICK),
				stm8_tim4_current_counts - 1U);

	/* Retain the sub-count phase so the reported reference cycles never jump backwards. */
	stm8_tim4_phase_bias =
		phase - (uint32_t)new_count * STM8_TIM4_CYCLES_PER_TICK / stm8_tim4_current_counts;
	sys_write8(stm8_tim4_period.shift, STM8_TIM4_PSCR_REG);
	sys_write8(stm8_tim4_current_counts - 1U, STM8_TIM4_ARR_REG);
	sys_write8(STM8_TIM_EGR_UG, STM8_TIM4_EGR_REG);
	sys_write8(new_count, STM8_TIM4_CNTR_REG);
	sys_write8(pending, STM8_TIM4_SR1_REG);
	sys_write8(STM8_TIM_CR1_CEN, STM8_TIM4_CR1_REG);
}

#endif

static void stm8_tim4_isr(const void *arg)
{
	k_spinlock_key_t key = sys_clock_lock();

	ARG_UNUSED(arg);
	sys_write8(0, STM8_TIM4_SR1_REG);
	stm8_tim4_cycle_base += STM8_TIM4_CYCLES_PER_TICK;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	stm8_tim4_phase_bias = 0U;
	if (stm8_tim4_period.remainder != 0U) {
		stm8_tim4_current_counts = stm8_tim4_next_counts();
		stm8_tim4_scale_update();
		sys_write8(stm8_tim4_current_counts - 1U, STM8_TIM4_ARR_REG);
	}
#endif
	sys_clock_announce_locked(1, key);
}

void sys_clock_set_timeout(uint32_t ticks, bool idle)
{
	/* The free-running periodic timer always announces one tick per interrupt. */
	ARG_UNUSED(ticks);
	ARG_UNUSED(idle);
}

uint32_t sys_clock_elapsed(void)
{
	return 0;
}

uint32_t sys_clock_cycle_get_32(void)
{
	unsigned int key = irq_lock();
	uint32_t cycles = stm8_tim4_cycle_base;
	uint8_t count = sys_read8(STM8_TIM4_CNTR_REG);
	bool pending = (sys_read8(STM8_TIM4_SR1_REG) & STM8_TIM_SR_UIF) != 0U;

	if (pending) {
		/* Re-read after rollover to avoid pairing the previous counter value with UIF. */
		count = sys_read8(STM8_TIM4_CNTR_REG);
		cycles += STM8_TIM4_CYCLES_PER_TICK;
	}
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	if (!stm8_tim4_started) {
		irq_unlock(key);
		return 0U;
	}
	uint32_t phase = stm8_tim4_phase_get(count);

	phase += pending ? 0U : stm8_tim4_phase_bias;
	cycles += MIN(phase, STM8_TIM4_CYCLES_PER_TICK - 1U);
#else
	cycles += (uint32_t)count * STM8_TIM4_PRESCALER;
#endif
	irq_unlock(key);
	return cycles;
}

static int stm8_tim4_init(void)
{
	const struct device *clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(STM8_TIM4_TIMER_NODE));
	clock_control_subsys_t id =
		(clock_control_subsys_t)DT_CLOCKS_CELL(STM8_TIM4_TIMER_NODE, id);
	uint32_t rate;
	int err;

	if (!device_is_ready(clock)) {
		return -ENODEV;
	}
	err = clock_control_get_rate(clock, id, &rate);
	if (err != 0) {
		return err;
	}
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	err = stm8_tim4_period_get(rate, &stm8_tim4_period);
	if (err != 0) {
		return err;
	}
	stm8_tim4_current_counts = stm8_tim4_next_counts();
	stm8_tim4_scale_update();
#else
	if (rate != CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) {
		return -EINVAL;
	}
#endif
	err = clock_control_on(clock, id);
	if (err != 0) {
		return err;
	}
	intc_stm8_irq_register(DT_IRQN(STM8_TIM4_TIMER_NODE), NULL, stm8_tim4_irq_set_enabled);
	IRQ_CONNECT(DT_IRQN(STM8_TIM4_TIMER_NODE), DT_IRQ(STM8_TIM4_TIMER_NODE, priority),
		    stm8_tim4_isr, NULL, 0);
	sys_write8(0, STM8_TIM4_CR1_REG);
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	sys_write8(stm8_tim4_period.shift, STM8_TIM4_PSCR_REG);
	sys_write8(stm8_tim4_current_counts - 1U, STM8_TIM4_ARR_REG);
	stm8_tim4_started = true;
#else
	sys_write8(STM8_TIM4_PRESCALER_SHIFT, STM8_TIM4_PSCR_REG);
	sys_write8(STM8_TIM4_COUNTS_PER_TICK - 1U, STM8_TIM4_ARR_REG);
#endif
	sys_write8(STM8_TIM_EGR_UG, STM8_TIM4_EGR_REG);
	sys_write8(0, STM8_TIM4_SR1_REG);
	irq_enable(DT_IRQN(STM8_TIM4_TIMER_NODE));
	sys_write8(STM8_TIM_SR_UIF, STM8_TIM4_IER_REG);
	sys_write8(STM8_TIM_CR1_CEN, STM8_TIM4_CR1_REG);
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	stm8_tim4_clock_client.clock_id = STM8_CLOCK_MASTER;
	stm8_tim4_clock_client.prepare = stm8_tim4_clock_prepare;
	stm8_tim4_clock_client.changed = stm8_tim4_clock_changed;
	return clock_control_stm8_register_client(&stm8_tim4_clock_client);
#else
	return 0;
#endif
}

#endif /* CONFIG_TICKLESS_KERNEL */

SYS_INIT(stm8_tim4_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
