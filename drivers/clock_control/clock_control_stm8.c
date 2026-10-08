/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_clock

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>
#include <zephyr/dt-bindings/clock/stm8-clock.h>
#include <zephyr/irq.h>

#define STM8_CLK_ICKR           0x00U
#define STM8_CLK_ECKR           0x01U
#define STM8_CLK_SWR            0x04U
#define STM8_CLK_SWCR           0x05U
#define STM8_CLK_SWCR_SWIF      0x08U
#define STM8_CLK_SWCR_SWEN      0x02U
#define STM8_CLK_READY_RETRIES  UINT16_MAX
#define STM8_CLK_OPT3           0x4805U
#define STM8_CLK_OPT3_LSI_EN    0x08U
#define STM8_CLK_OPT7           0x480dU
#define STM8_CLK_OPT7_WAITSTATE 0x01U
#define STM8_CLK_ZERO_WAIT_MAX  16000000UL
#define STM8_CLK_MAX_RATE       24000000UL
#define STM8_CLK_SOURCE_HSE     0xb4U
#define STM8_CLK_SOURCE_LSI     0xd2U
#define STM8_CLK_CMSR           0x03U
#define STM8_CLK_CKDIVR         0x06U
#define STM8_CLK_PCKENR1        0x07U
#define STM8_CLK_PCKENR2        0x0aU
#define STM8_CLK_SOURCE_HSI     0xe1U
#define STM8_CLK_HSIDIV_SHIFT   3U
#define STM8_CLK_HSIDIV_MASK    3U
#define STM8_CLK_CPUDIV_MASK    7U

#define STM8_CLK_BASE        DT_INST_REG_ADDR(0)
#define STM8_CLK_CMSR_REG    (STM8_CLK_BASE + STM8_CLK_CMSR)
#define STM8_CLK_CKDIVR_REG  (STM8_CLK_BASE + STM8_CLK_CKDIVR)
#define STM8_CLK_PCKENR1_REG (STM8_CLK_BASE + STM8_CLK_PCKENR1)
#define STM8_CLK_PCKENR2_REG (STM8_CLK_BASE + STM8_CLK_PCKENR2)
#define STM8_CLK_HSI_RATE    DT_PROP(DT_CLOCKS_CTLR_BY_NAME(DT_DRV_INST(0), hsi), clock_frequency)
#define STM8_CLK_HSI_DIVISOR DT_INST_PROP(0, st_hsi_divisor)
#define STM8_CLK_CPU_DIVISOR DT_INST_PROP(0, st_cpu_divisor)

#define STM8_CLK_HSE_RATE    DT_PROP(DT_CLOCKS_CTLR_BY_NAME(DT_DRV_INST(0), hse), clock_frequency)
#define STM8_CLK_LSI_RATE    DT_PROP(DT_CLOCKS_CTLR_BY_NAME(DT_DRV_INST(0), lsi), clock_frequency)
#define STM8_CLK_BOOT_SOURCE (STM8_CLOCK_HSI + DT_INST_ENUM_IDX(0, st_clock_source))
#define STM8_CLK_BOOT_RATE                                                                         \
	(STM8_CLK_BOOT_SOURCE == STM8_CLOCK_HSI   ? STM8_CLK_HSI_RATE / STM8_CLK_HSI_DIVISOR       \
	 : STM8_CLK_BOOT_SOURCE == STM8_CLOCK_HSE ? STM8_CLK_HSE_RATE                              \
						  : STM8_CLK_LSI_RATE)

BUILD_ASSERT(STM8_CLK_BOOT_RATE == DT_INST_PROP(0, clock_frequency),
	     "clock-frequency must describe the initial master clock");

static bool clock_control_stm8_transition;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static sys_slist_t clock_control_stm8_clients = SYS_SLIST_STATIC_INIT(&clock_control_stm8_clients);

int clock_control_stm8_register_client(struct clock_control_stm8_client *client)
{
	unsigned int key = irq_lock();
	struct clock_control_stm8_client *entry;

	if (client == NULL || client->prepare == NULL || client->changed == NULL ||
	    (client->clock_id != STM8_CLOCK_MASTER && client->clock_id != STM8_CLOCK_CPU)) {
		irq_unlock(key);
		return -EINVAL;
	}
	if (clock_control_stm8_transition) {
		irq_unlock(key);
		return -EBUSY;
	}
	SYS_SLIST_FOR_EACH_CONTAINER(&clock_control_stm8_clients, entry, node) {
		if (entry == client) {
			irq_unlock(key);
			return -EINVAL;
		}
	}
	sys_slist_append(&clock_control_stm8_clients, &client->node);
	irq_unlock(key);
	return 0;
}
#endif

static uint8_t clock_control_stm8_source_id(uint8_t source)
{
	return source == STM8_CLK_SOURCE_HSI   ? STM8_CLOCK_HSI
	       : source == STM8_CLK_SOURCE_HSE ? STM8_CLOCK_HSE
	       : source == STM8_CLK_SOURCE_LSI ? STM8_CLOCK_LSI
					       : 0U;
}

static uint8_t clock_control_stm8_source_value(uint8_t id)
{
	return id == STM8_CLOCK_HSI   ? STM8_CLK_SOURCE_HSI
	       : id == STM8_CLOCK_HSE ? STM8_CLK_SOURCE_HSE
				      : STM8_CLK_SOURCE_LSI;
}

static int clock_control_stm8_source_rate(uint8_t id, uint32_t *rate)
{
	switch (id) {
	case STM8_CLOCK_HSI:
		*rate = STM8_CLK_HSI_RATE;
		break;
	case STM8_CLOCK_HSE:
		if (!DT_NODE_HAS_STATUS_OKAY(DT_CLOCKS_CTLR_BY_NAME(DT_DRV_INST(0), hse))) {
			return -ENOTSUP;
		}
		*rate = STM8_CLK_HSE_RATE;
		break;
	case STM8_CLOCK_LSI:
		*rate = STM8_CLK_LSI_RATE;
		break;
	default:
		return -EINVAL;
	}
	return *rate != 0U && *rate <= STM8_CLK_MAX_RATE ? 0 : -EINVAL;
}

static int clock_control_stm8_oscillator(uint8_t id, bool enable)
{
	uint32_t rate;
	int err = clock_control_stm8_source_rate(id, &rate);
	uintptr_t reg = STM8_CLK_BASE + (id == STM8_CLOCK_HSE ? STM8_CLK_ECKR : STM8_CLK_ICKR);
	uint8_t mask = id == STM8_CLOCK_LSI ? 0x08U : 0x01U;
	unsigned int key;

	if (err != 0) {
		return err;
	}
	key = irq_lock();
	if (clock_control_stm8_transition ||
	    (!enable && clock_control_stm8_source_id(sys_read8(STM8_CLK_CMSR_REG)) == id)) {
		irq_unlock(key);
		return -EBUSY;
	}
	uint8_t value = sys_read8(reg);

	sys_write8(enable ? value | mask : value & ~mask, reg);
	irq_unlock(key);
	if (!enable) {
		return (sys_read8(reg) & mask) == 0U ? 0 : -EBUSY;
	}
	for (uint16_t retry = 0U; retry < STM8_CLK_READY_RETRIES; retry++) {
		if ((sys_read8(reg) & (mask << 1)) != 0U) {
			return 0;
		}
	}
	return -ETIMEDOUT;
}

static bool clock_control_stm8_valid_id(uint8_t id)
{
	return (id >= STM8_CLOCK_I2C && id <= STM8_CLOCK_TIM1) || id == STM8_CLOCK_AWU ||
	       id == STM8_CLOCK_ADC || id == STM8_CLOCK_CAN || id == STM8_CLOCK_CPU ||
	       id == STM8_CLOCK_MASTER || (id >= STM8_CLOCK_HSI && id <= STM8_CLOCK_LSI);
}

static int clock_control_stm8_gate(const struct device *dev, clock_control_subsys_t subsys,
				   bool enable)
{
	uintptr_t raw_id = (uintptr_t)subsys;
	uint8_t id = raw_id;
	uintptr_t reg;
	uint8_t mask;
	unsigned int key;

	ARG_UNUSED(dev);
	if (raw_id > STM8_CLOCK_LSI || !clock_control_stm8_valid_id(id)) {
		return -EINVAL;
	}
	if (id >= STM8_CLOCK_HSI) {
		return clock_control_stm8_oscillator(id, enable);
	}
	if (id >= STM8_CLOCK_CPU) {
		return enable ? 0 : -ENOTSUP;
	}
	id--;
	reg = id < 8U ? STM8_CLK_PCKENR1_REG : STM8_CLK_PCKENR2_REG;
	mask = (uint8_t)(1U << (id & 7U));
	key = irq_lock();
	sys_write8(enable ? sys_read8(reg) | mask : sys_read8(reg) & ~mask, reg);
	irq_unlock(key);
	return 0;
}

static int clock_control_stm8_on(const struct device *dev, clock_control_subsys_t subsys)
{
	return clock_control_stm8_gate(dev, subsys, true);
}

static int clock_control_stm8_off(const struct device *dev, clock_control_subsys_t subsys)
{
	return clock_control_stm8_gate(dev, subsys, false);
}

static int clock_control_stm8_get_rate(const struct device *dev, clock_control_subsys_t subsys,
				       uint32_t *rate)
{
	uintptr_t raw_id = (uintptr_t)subsys;
	uint8_t id = raw_id;
	unsigned int key = irq_lock();
	uint8_t dividers = sys_read8(STM8_CLK_CKDIVR_REG);
	uint8_t source = clock_control_stm8_source_id(sys_read8(STM8_CLK_CMSR_REG));

	irq_unlock(key);

	ARG_UNUSED(dev);
	if (raw_id > STM8_CLOCK_LSI || !clock_control_stm8_valid_id(id)) {
		return -EINVAL;
	}
	if (id >= STM8_CLOCK_HSI) {
		return clock_control_stm8_source_rate(id, rate);
	}
	int err = clock_control_stm8_source_rate(source, rate);

	if (err != 0) {
		return err;
	}
	if (source == STM8_CLOCK_HSI) {
		*rate >>= (dividers >> STM8_CLK_HSIDIV_SHIFT) & STM8_CLK_HSIDIV_MASK;
	}
	if (id == STM8_CLOCK_CPU) {
		*rate >>= dividers & STM8_CLK_CPUDIV_MASK;
	}
	return 0;
}

static enum clock_control_status clock_control_stm8_get_status(const struct device *dev,
							       clock_control_subsys_t subsys)
{
	uintptr_t raw_id = (uintptr_t)subsys;
	uint8_t id = raw_id;
	uintptr_t reg;

	ARG_UNUSED(dev);
	if (raw_id > STM8_CLOCK_LSI || !clock_control_stm8_valid_id(id)) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
	if (id >= STM8_CLOCK_HSI) {
		uint8_t mask = id == STM8_CLOCK_LSI ? 0x08U : 0x01U;

		reg = STM8_CLK_BASE + (id == STM8_CLOCK_HSE ? STM8_CLK_ECKR : STM8_CLK_ICKR);
		return (sys_read8(reg) & mask) != 0U ? CLOCK_CONTROL_STATUS_ON
						     : CLOCK_CONTROL_STATUS_OFF;
	}
	if (id >= STM8_CLOCK_CPU) {
		return CLOCK_CONTROL_STATUS_ON;
	}
	id--;
	reg = id < 8U ? STM8_CLK_PCKENR1_REG : STM8_CLK_PCKENR2_REG;
	uint8_t mask = (uint8_t)(1U << (id & 7U));

	return (sys_read8(reg) & mask) != 0U ? CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static int clock_control_stm8_switch_locked(const struct device *dev,
					    const struct stm8_clock_control_config *settings)
{
	uint32_t source_rate;
	uint32_t old_rate;
	uint32_t rate;
	uint8_t old_source = sys_read8(STM8_CLK_CMSR_REG);
	uint8_t old_divider = sys_read8(STM8_CLK_CKDIVR_REG);
	uint8_t source;
	unsigned int key;
	int err;

	if (settings == NULL || settings->source < STM8_CLOCK_HSI ||
	    settings->source > STM8_CLOCK_LSI || settings->hsi_divisor == 0U ||
	    settings->hsi_divisor > 8U || !is_power_of_two(settings->hsi_divisor) ||
	    settings->cpu_divisor == 0U || settings->cpu_divisor > 128U ||
	    !is_power_of_two(settings->cpu_divisor)) {
		return -EINVAL;
	}
	err = clock_control_stm8_source_rate(settings->source, &source_rate);
	if (err != 0) {
		return err;
	}
	rate = source_rate / (settings->source == STM8_CLOCK_HSI ? settings->hsi_divisor : 1U);
	if (settings->source == STM8_CLOCK_LSI &&
	    (sys_read8(STM8_CLK_OPT3) & STM8_CLK_OPT3_LSI_EN) == 0U) {
		return -ENOTSUP;
	}
	if (rate / settings->cpu_divisor > STM8_CLK_ZERO_WAIT_MAX &&
	    (sys_read8(STM8_CLK_OPT7) & STM8_CLK_OPT7_WAITSTATE) == 0U) {
		return -ENOTSUP;
	}
	err = clock_control_stm8_get_rate(dev, (clock_control_subsys_t)STM8_CLOCK_MASTER,
					  &old_rate);
	if (err != 0) {
		return err;
	}
	source = clock_control_stm8_source_value(settings->source);
	if (source != old_source) {
		/* Manual switch: keep the old source running until the new oscillator is stable. */
		sys_write8(0U, STM8_CLK_BASE + STM8_CLK_SWCR);
		sys_write8(source, STM8_CLK_BASE + STM8_CLK_SWR);
		uint16_t retry;

		for (retry = 0U; retry < STM8_CLK_READY_RETRIES; retry++) {
			if ((sys_read8(STM8_CLK_BASE + STM8_CLK_SWCR) & STM8_CLK_SWCR_SWIF) != 0U) {
				break;
			}
		}
		if (retry == STM8_CLK_READY_RETRIES) {
			sys_write8(0U, STM8_CLK_BASE + STM8_CLK_SWCR);
			return -ETIMEDOUT;
		}
	}
	key = irq_lock();
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	uint32_t old_cpu_rate = old_rate >> (old_divider & STM8_CLK_CPUDIV_MASK);
	uint32_t cpu_rate = rate / settings->cpu_divisor;

	if (rate != old_rate || cpu_rate != old_cpu_rate) {
		struct clock_control_stm8_client *client;

		SYS_SLIST_FOR_EACH_CONTAINER(&clock_control_stm8_clients, client, node) {
			uint32_t next = client->clock_id == STM8_CLOCK_CPU ? cpu_rate : rate;
			uint32_t previous =
				client->clock_id == STM8_CLOCK_CPU ? old_cpu_rate : old_rate;

			if (next != previous &&
			    (client->dev == NULL || device_is_ready(client->dev))) {
				err = client->prepare(client->dev, next);
				if (err != 0) {
					irq_unlock(key);
					sys_write8(0U, STM8_CLK_BASE + STM8_CLK_SWCR);
					return err;
				}
			}
		}
	}
#endif
	/* Apply the final divider only after checking both the old and new CPU rates. */
	uint8_t divider = (LOG2(settings->hsi_divisor) << STM8_CLK_HSIDIV_SHIFT) |
			  LOG2(settings->cpu_divisor);

	if (source != old_source) {
		/* A conservative CPU divider avoids an intermediate frequency above 16 MHz. */
		sys_write8((sys_read8(STM8_CLK_CKDIVR_REG) & ~STM8_CLK_CPUDIV_MASK) |
				   STM8_CLK_CPUDIV_MASK,
			   STM8_CLK_CKDIVR_REG);
		sys_write8(STM8_CLK_SWCR_SWEN, STM8_CLK_BASE + STM8_CLK_SWCR);
		uint16_t retry;

		for (retry = 0U; retry < STM8_CLK_READY_RETRIES; retry++) {
			if (sys_read8(STM8_CLK_CMSR_REG) == source) {
				break;
			}
		}
		if (retry == STM8_CLK_READY_RETRIES) {
			sys_write8(0U, STM8_CLK_BASE + STM8_CLK_SWCR);
			sys_write8(old_divider, STM8_CLK_CKDIVR_REG);
			irq_unlock(key);
			return -ETIMEDOUT;
		}
	}
	sys_write8(divider, STM8_CLK_CKDIVR_REG);
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	if (rate != old_rate || cpu_rate != old_cpu_rate) {
		struct clock_control_stm8_client *client;

		SYS_SLIST_FOR_EACH_CONTAINER(&clock_control_stm8_clients, client, node) {
			uint32_t next = client->clock_id == STM8_CLOCK_CPU ? cpu_rate : rate;
			uint32_t previous =
				client->clock_id == STM8_CLOCK_CPU ? old_cpu_rate : old_rate;

			if (next != previous &&
			    (client->dev == NULL || device_is_ready(client->dev))) {
				client->changed(client->dev, next);
			}
		}
	}
#endif
	irq_unlock(key);
	return 0;
}

static int clock_control_stm8_transition_begin(void)
{
	unsigned int key = irq_lock();

	if (clock_control_stm8_transition) {
		irq_unlock(key);
		return -EBUSY;
	}
	clock_control_stm8_transition = true;
	irq_unlock(key);
	return 0;
}

static void clock_control_stm8_transition_end(void)
{
	unsigned int key = irq_lock();

	clock_control_stm8_transition = false;
	irq_unlock(key);
}

static int clock_control_stm8_switch(const struct device *dev,
				     const struct stm8_clock_control_config *settings)
{
	int err = clock_control_stm8_transition_begin();

	if (err != 0) {
		return err;
	}
	err = clock_control_stm8_switch_locked(dev, settings);
	clock_control_stm8_transition_end();
	return err;
}

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static int clock_control_stm8_configure(const struct device *dev, clock_control_subsys_t subsys,
					void *settings)
{
	if ((uintptr_t)subsys != STM8_CLOCK_MASTER) {
		return -ENOTSUP;
	}
	return clock_control_stm8_switch(dev, settings);
}

static int clock_control_stm8_set_rate(const struct device *dev, clock_control_subsys_t subsys,
				       clock_control_subsys_rate_t requested)
{
	uintptr_t id = (uintptr_t)subsys;
	uint32_t rate;
	uint32_t base;
	int err = clock_control_stm8_transition_begin();

	if (err != 0) {
		return err;
	}
	uint8_t divider = sys_read8(STM8_CLK_CKDIVR_REG);
	uint8_t source = clock_control_stm8_source_id(sys_read8(STM8_CLK_CMSR_REG));
	struct stm8_clock_control_config settings = {
		.source = source,
		.hsi_divisor = 1U << ((divider >> STM8_CLK_HSIDIV_SHIFT) & STM8_CLK_HSIDIV_MASK),
		.cpu_divisor = 1U << (divider & STM8_CLK_CPUDIV_MASK),
	};

	if (requested == NULL || *(uint32_t *)requested == 0U) {
		err = -EINVAL;
		goto out;
	}
	rate = *(uint32_t *)requested;
	if (id == STM8_CLOCK_CPU) {
		err = clock_control_stm8_get_rate(dev, (clock_control_subsys_t)STM8_CLOCK_MASTER,
						  &base);
	} else if (id == STM8_CLOCK_MASTER && source == STM8_CLOCK_HSI) {
		err = clock_control_stm8_source_rate(source, &base);
	} else {
		err = -ENOTSUP;
		goto out;
	}
	if (err != 0) {
		goto out;
	}
	uint32_t divisor = base / rate;

	if (base % rate != 0U || divisor == 0U || !is_power_of_two(divisor) ||
	    divisor > (id == STM8_CLOCK_CPU ? 128U : 8U)) {
		err = -EINVAL;
		goto out;
	}
	if (id == STM8_CLOCK_CPU) {
		settings.cpu_divisor = divisor;
	} else {
		settings.hsi_divisor = divisor;
	}
	err = clock_control_stm8_switch_locked(dev, &settings);
out:
	clock_control_stm8_transition_end();
	return err;
}
#endif

static int clock_control_stm8_init(const struct device *dev)
{
	/* Reset selects HSI; the SoC early hook has already set its dividers. */
	if (STM8_CLK_BOOT_SOURCE == STM8_CLOCK_HSI) {
		return sys_read8(STM8_CLK_CMSR_REG) == STM8_CLK_SOURCE_HSI ? 0 : -EIO;
	}
	const struct stm8_clock_control_config settings = {
		.source = STM8_CLK_BOOT_SOURCE,
		.hsi_divisor = STM8_CLK_HSI_DIVISOR,
		.cpu_divisor = STM8_CLK_CPU_DIVISOR,
	};

	return clock_control_stm8_switch(dev, &settings);
}

static DEVICE_API(clock_control, clock_control_stm8_driver_api) = {
	.on = clock_control_stm8_on,
	.off = clock_control_stm8_off,
	.get_rate = clock_control_stm8_get_rate,
	.get_status = clock_control_stm8_get_status,
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	.configure = clock_control_stm8_configure,
	.set_rate = clock_control_stm8_set_rate,
#endif
};

DEVICE_DT_INST_DEFINE(0, clock_control_stm8_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_stm8_driver_api);
