/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>

#define STM8_GPIO_IDR 0x01U
#define STM8_GPIO_DDR 0x02U
#define STM8_GPIO_CR1 0x03U
#define STM8_GPIO_CR2 0x04U
#define STM8_EXTI_CR1 0x50a0U
#define STM8_EXTI_CR2 0x50a1U

struct gpio_stm8_config {
	struct gpio_driver_config common;
	uintptr_t base;
	uint8_t open_drain;
	uint8_t exti_mask;
	uint8_t port_index;
	void (*irq_config)(void);
};

struct gpio_stm8_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	uint8_t irq_mask;
	bool vector_enabled;
};

static void gpio_stm8_irq_set_enabled(const struct device *dev, bool enabled)
{
	const struct gpio_stm8_config *cfg = dev->config;
	struct gpio_stm8_data *data = dev->data;
	uint8_t mask = cfg->exti_mask & ~sys_read8(cfg->base + STM8_GPIO_DDR);

	data->vector_enabled = enabled;
	sys_write8((sys_read8(cfg->base + STM8_GPIO_CR2) & ~mask) |
			   (enabled ? data->irq_mask & mask : 0U),
		   cfg->base + STM8_GPIO_CR2);
}

static int gpio_stm8_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_stm8_config *cfg = dev->config;
	struct gpio_stm8_data *data = dev->data;
	uint8_t mask;
	uint8_t cr1 = 0U;
	unsigned int key;

	if (!gpio_port_pin_is_supported(cfg->common.port_pin_mask, pin)) {
		return -EINVAL;
	}
	if ((flags & GPIO_PULL_DOWN) != 0U ||
	    ((flags & GPIO_SINGLE_ENDED) != 0U && (flags & GPIO_LINE_OPEN_DRAIN) == 0U)) {
		return -ENOTSUP;
	}
	mask = (uint8_t)(1U << (uint8_t)pin);
	if ((cfg->open_drain & mask) != 0U &&
	    (((flags & GPIO_OUTPUT) != 0U && (flags & GPIO_OPEN_DRAIN) == 0U) ||
	     (flags & GPIO_PULL_UP) != 0U)) {
		return -ENOTSUP;
	}
	if ((flags & GPIO_OUTPUT) != 0U && (flags & GPIO_PULL_UP) != 0U) {
		return -ENOTSUP;
	}
	key = irq_lock();
	sys_write8(sys_read8(cfg->base + STM8_GPIO_CR2) & ~mask, cfg->base + STM8_GPIO_CR2);
	data->irq_mask &= ~mask;
	if ((flags & GPIO_OUTPUT) != 0U) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			sys_write8(sys_read8(cfg->base) | mask, cfg->base);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			sys_write8(sys_read8(cfg->base) & ~mask, cfg->base);
		}
		cr1 = (flags & GPIO_OPEN_DRAIN) == 0U ? mask : 0U;
	} else {
		sys_write8(sys_read8(cfg->base + STM8_GPIO_DDR) & ~mask, cfg->base + STM8_GPIO_DDR);
		cr1 = (flags & GPIO_PULL_UP) != 0U ? mask : 0U;
	}
	sys_write8((sys_read8(cfg->base + STM8_GPIO_CR1) & ~mask) | cr1, cfg->base + STM8_GPIO_CR1);
	if ((flags & GPIO_OUTPUT) != 0U) {
		sys_write8(sys_read8(cfg->base + STM8_GPIO_DDR) | mask, cfg->base + STM8_GPIO_DDR);
	}
	irq_unlock(key);
	return 0;
}

static int gpio_stm8_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_stm8_config *cfg = dev->config;

	*value = sys_read8(cfg->base + STM8_GPIO_IDR) & cfg->common.port_pin_mask;
	return 0;
}

static int gpio_stm8_port_update(const struct device *dev, gpio_port_pins_t mask,
				 gpio_port_value_t value, bool toggle)
{
	const struct gpio_stm8_config *cfg = dev->config;
	unsigned int key;
	uint8_t old;

	if ((mask & ~cfg->common.port_pin_mask) != 0U) {
		return -EINVAL;
	}
	key = irq_lock();
	old = sys_read8(cfg->base);
	sys_write8(toggle ? old ^ mask : (old & ~mask) | (value & mask), cfg->base);
	irq_unlock(key);
	return 0;
}

static int gpio_stm8_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	return gpio_stm8_port_update(dev, mask, value, false);
}

static int gpio_stm8_port_set_bits_raw(const struct device *dev, gpio_port_pins_t mask)
{
	return gpio_stm8_port_update(dev, mask, mask, false);
}

static int gpio_stm8_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t mask)
{
	return gpio_stm8_port_update(dev, mask, 0U, false);
}

static int gpio_stm8_port_toggle_bits(const struct device *dev, gpio_port_pins_t mask)
{
	return gpio_stm8_port_update(dev, mask, 0U, true);
}

static int gpio_stm8_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					     enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_stm8_config *cfg = dev->config;
	struct gpio_stm8_data *data = dev->data;
	uintptr_t exti;
	uint8_t mask;
	uint8_t sensitivity;
	uint8_t shift;
	unsigned int key;

	if (!gpio_port_pin_is_supported(cfg->common.port_pin_mask, pin)) {
		return -EINVAL;
	}
	mask = (uint8_t)(1U << (uint8_t)pin);
	if (mode == GPIO_INT_MODE_DISABLED) {
		key = irq_lock();
		data->irq_mask &= ~mask;
		/* CR2 controls output slew, so do not alter an output pin here. */
		if ((sys_read8(cfg->base + STM8_GPIO_DDR) & mask) == 0U) {
			sys_write8(sys_read8(cfg->base + STM8_GPIO_CR2) & ~mask,
				   cfg->base + STM8_GPIO_CR2);
		}
		irq_unlock(key);
		return 0;
	}
	if ((cfg->exti_mask & mask) == 0U) {
		return -ENOTSUP;
	}
	if (mode == GPIO_INT_MODE_EDGE) {
		if (trig != GPIO_INT_TRIG_LOW && trig != GPIO_INT_TRIG_HIGH &&
		    trig != GPIO_INT_TRIG_BOTH) {
			return -EINVAL;
		}
		sensitivity = trig == GPIO_INT_TRIG_HIGH ? 1U : trig == GPIO_INT_TRIG_LOW ? 2U : 3U;
	} else if (mode == GPIO_INT_MODE_LEVEL && trig == GPIO_INT_TRIG_LOW) {
		sensitivity = 0U;
	} else {
		return -ENOTSUP;
	}
	/* EXTI sensitivity registers are writable only at CPU interrupt level 3. */
	key = irq_lock();
	if ((sys_read8(cfg->base + STM8_GPIO_DDR) & mask) != 0U) {
		irq_unlock(key);
		return -EINVAL;
	}
	if ((data->irq_mask & ~mask) != 0U) {
		irq_unlock(key);
		return -EBUSY;
	}
	exti = cfg->port_index < 4U ? STM8_EXTI_CR1 : STM8_EXTI_CR2;
	shift = cfg->port_index < 4U ? cfg->port_index * 2U : 0U;
	sys_write8((sys_read8(exti) & ~(3U << shift)) | (sensitivity << shift), exti);
	data->irq_mask |= mask;
	if (data->vector_enabled) {
		sys_write8(sys_read8(cfg->base + STM8_GPIO_CR2) | mask, cfg->base + STM8_GPIO_CR2);
	}
	irq_unlock(key);
	return 0;
}

static int gpio_stm8_manage_callback(const struct device *dev, struct gpio_callback *cb, bool set)
{
	struct gpio_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	int err = gpio_manage_callback(&data->callbacks, cb, set);

	irq_unlock(key);
	return err;
}

static void gpio_stm8_isr(const void *arg)
{
	const struct device *dev = arg;
	struct gpio_stm8_data *data = dev->data;

	gpio_fire_callbacks(&data->callbacks, dev, data->irq_mask);
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_stm8_pin_get_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t *flags)
{
	const struct gpio_stm8_config *cfg = dev->config;
	const struct gpio_stm8_data *data = dev->data;
	uint8_t mask;

	if (!gpio_port_pin_is_supported(cfg->common.port_pin_mask, pin)) {
		return -EINVAL;
	}
	mask = (uint8_t)(1U << (uint8_t)pin);
	if ((sys_read8(cfg->base + STM8_GPIO_DDR) & mask) != 0U) {
		*flags = GPIO_OUTPUT;
		if ((sys_read8(cfg->base + STM8_GPIO_CR1) & mask) == 0U) {
			*flags |= GPIO_OPEN_DRAIN;
		}
	} else {
		*flags = GPIO_INPUT;
		if ((sys_read8(cfg->base + STM8_GPIO_CR1) & mask) != 0U) {
			*flags |= GPIO_PULL_UP;
		}
	}
	if ((data->common.invert & mask) != 0U) {
		*flags |= GPIO_ACTIVE_LOW;
	}
	return 0;
}
#endif

#ifdef CONFIG_GPIO_GET_DIRECTION
static int gpio_stm8_port_get_direction(const struct device *dev, gpio_port_pins_t map,
					gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	const struct gpio_stm8_config *cfg = dev->config;
	uint8_t ddr = sys_read8(cfg->base + STM8_GPIO_DDR);

	map &= cfg->common.port_pin_mask;
	if (inputs != NULL) {
		*inputs = map & ~ddr;
	}
	if (outputs != NULL) {
		*outputs = map & ddr;
	}
	return 0;
}
#endif

static int gpio_stm8_init(const struct device *dev)
{
	const struct gpio_stm8_config *cfg = dev->config;

	if (cfg->irq_config != NULL) {
		cfg->irq_config();
	}
	/* Leave existing pinctrl and SWIM configurations untouched. */
	return 0;
}

static DEVICE_API(gpio, gpio_stm8_driver_api) = {
	.pin_configure = gpio_stm8_pin_configure,
	.port_get_raw = gpio_stm8_port_get_raw,
	.port_set_masked_raw = gpio_stm8_port_set_masked_raw,
	.port_set_bits_raw = gpio_stm8_port_set_bits_raw,
	.port_clear_bits_raw = gpio_stm8_port_clear_bits_raw,
	.port_toggle_bits = gpio_stm8_port_toggle_bits,
	.pin_interrupt_configure = gpio_stm8_pin_interrupt_configure,
	.manage_callback = gpio_stm8_manage_callback,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_stm8_pin_get_config,
#endif
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_stm8_port_get_direction,
#endif
};

#define GPIO_STM8_IRQ_DEFINE(n)                                                                    \
	static void gpio_stm8_irq_config_##n(void)                                                 \
	{                                                                                          \
		intc_stm8_irq_register(DT_INST_IRQ_BY_NAME(n, port, irq), DEVICE_DT_INST_GET(n),   \
				       gpio_stm8_irq_set_enabled);                                 \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, port, irq),                                     \
			    DT_INST_IRQ_BY_NAME(n, port, priority), gpio_stm8_isr,                 \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, port, irq));                                     \
	}

#define GPIO_STM8_INIT(n)                                                                          \
	IF_ENABLED(DT_INST_IRQ_HAS_NAME(n, port),                                                  \
		   (GPIO_STM8_IRQ_DEFINE(n))) \
	static struct gpio_stm8_data gpio_stm8_data_##n;                                           \
	static const struct gpio_stm8_config gpio_stm8_config_##n = {                              \
		.common = {.port_pin_mask = DT_INST_PROP(n, st_pin_mask) &                         \
					    GPIO_DT_INST_PORT_PIN_MASK_NGPIOS_EXC(                 \
						    n, DT_INST_PROP(n, ngpios))},                  \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.open_drain = DT_INST_PROP(n, st_true_open_drain_mask),                            \
		.exti_mask = DT_INST_PROP(n, st_exti_mask),                                        \
		.port_index = DT_INST_PROP(n, st_port_index),                                      \
		.irq_config = COND_CODE_1(DT_INST_IRQ_HAS_NAME(n, port), \
					(gpio_stm8_irq_config_##n), (NULL)), \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, gpio_stm8_init, NULL, &gpio_stm8_data_##n, &gpio_stm8_config_##n, \
			      PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_stm8_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_STM8_INIT)
