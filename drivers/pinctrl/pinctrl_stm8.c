/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>

#define STM8_GPIO_DDR    0x02U
#define STM8_GPIO_CR1    0x03U
#define STM8_GPIO_CR2    0x04U
#define STM8_PINCTRL_AFR 0x4803U

#define PINCTRL_STM8_PORT_CONFIG(node)                                                             \
	{.base = DT_REG_ADDR(node),                                                                \
	 .mask = DT_PROP(node, st_pin_mask) &                                                      \
		 GPIO_DT_PORT_PIN_MASK_NGPIOS_EXC(node, DT_PROP(node, ngpios)),                    \
	 .open_drain = DT_PROP_OR(node, st_true_open_drain_mask, 0)}

static const struct {
	uintptr_t base;
	uint8_t mask;
	uint8_t open_drain;
} pinctrl_stm8_ports[] = {
	PINCTRL_STM8_PORT_CONFIG(DT_NODELABEL(gpioa)),
	PINCTRL_STM8_PORT_CONFIG(DT_NODELABEL(gpiob)),
	PINCTRL_STM8_PORT_CONFIG(DT_NODELABEL(gpioc)),
	PINCTRL_STM8_PORT_CONFIG(DT_NODELABEL(gpiod)),
	PINCTRL_STM8_PORT_CONFIG(DT_NODELABEL(gpioe)),
	PINCTRL_STM8_PORT_CONFIG(DT_NODELABEL(gpiof)),
};

static int pinctrl_stm8_validate_pin(const pinctrl_soc_pin_t *pin)
{
	uint8_t port = pin->pinmux >> STM8_PINMUX_PORT_SHIFT;
	uint8_t mask = (uint8_t)(1U << (pin->pinmux & STM8_PINMUX_PIN_MASK));
	uint8_t flags = pin->flags;

	if (port >= ARRAY_SIZE(pinctrl_stm8_ports) ||
	    (pinctrl_stm8_ports[port].mask & mask) == 0U ||
	    (flags & (STM8_PIN_HIGH | STM8_PIN_LOW)) == (STM8_PIN_HIGH | STM8_PIN_LOW)) {
		return -EINVAL;
	}
	if ((pinctrl_stm8_ports[port].open_drain & mask) != 0U &&
	    (((flags & STM8_PIN_OUTPUT) != 0U && (flags & STM8_PIN_OPEN_DRAIN) == 0U) ||
	     (flags & STM8_PIN_PULL_UP) != 0U)) {
		return -ENOTSUP;
	}
	if ((flags & (STM8_PIN_OUTPUT | STM8_PIN_PULL_UP)) ==
	    (STM8_PIN_OUTPUT | STM8_PIN_PULL_UP)) {
		return -ENOTSUP;
	}
	if ((pin->remap_value & ~pin->remap_mask) != 0U) {
		return -EINVAL;
	}
	/* Remaps are nonvolatile option bytes, not a runtime pin multiplexer. */
	if ((sys_read8(STM8_PINCTRL_AFR) & pin->remap_mask) != pin->remap_value) {
		return -ENOTSUP;
	}
	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	unsigned int key;

	ARG_UNUSED(reg);
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		int err = pinctrl_stm8_validate_pin(&pins[i]);

		if (err != 0) {
			return err;
		}
	}
	key = irq_lock();
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		const pinctrl_soc_pin_t *pin = &pins[i];
		uintptr_t base = pinctrl_stm8_ports[pin->pinmux >> STM8_PINMUX_PORT_SHIFT].base;
		uint8_t mask = (uint8_t)(1U << (pin->pinmux & STM8_PINMUX_PIN_MASK));
		uint8_t flags = pin->flags;
		uint8_t cr1 = 0U;
		uint8_t cr2 = 0U;

		/* Disable input interrupts before changing direction or output type. */
		sys_write8(sys_read8(base + STM8_GPIO_CR2) & ~mask, base + STM8_GPIO_CR2);
		if ((flags & STM8_PIN_HIGH) != 0U) {
			sys_write8(sys_read8(base) | mask, base);
		} else if ((flags & STM8_PIN_LOW) != 0U) {
			sys_write8(sys_read8(base) & ~mask, base);
		}
		if ((flags & STM8_PIN_OUTPUT) != 0U) {
			cr1 = (flags & STM8_PIN_OPEN_DRAIN) == 0U ? mask : 0U;
			cr2 = (flags & STM8_PIN_FAST) != 0U ? mask : 0U;
		} else {
			cr1 = (flags & STM8_PIN_PULL_UP) != 0U ? mask : 0U;
			sys_write8(sys_read8(base + STM8_GPIO_DDR) & ~mask, base + STM8_GPIO_DDR);
		}
		sys_write8((sys_read8(base + STM8_GPIO_CR1) & ~mask) | cr1, base + STM8_GPIO_CR1);
		if ((flags & STM8_PIN_OUTPUT) != 0U) {
			sys_write8(sys_read8(base + STM8_GPIO_DDR) | mask, base + STM8_GPIO_DDR);
		}
		sys_write8((sys_read8(base + STM8_GPIO_CR2) & ~mask) | cr2, base + STM8_GPIO_CR2);
	}
	irq_unlock(key);
	return 0;
}
