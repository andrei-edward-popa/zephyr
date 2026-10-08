/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_INTC_STM8_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_INTC_STM8_H_

#include <stdbool.h>

struct device;

/**
 * @internal
 * @brief Set the software priority of an STM8 interrupt.
 * @param irq Interrupt vector number.
 * @param priority Zephyr IRQ priority, from zero through two.
 */
void intc_stm8_irq_priority_set(unsigned int irq, unsigned int priority);

/**
 * @internal
 * @brief Register a peripheral's vector mask operation during initialization.
 *
 * STM8 ITC provides priorities but no per-vector enable registers. The
 * peripheral owns its source masks and must retain requested enables while
 * its vector is disabled. The callback runs with interrupts locked.
 *
 * @param irq Interrupt vector number.
 * @param dev Peripheral device, or NULL for the system timer.
 * @param set_enabled Callback to mask or restore this vector's sources.
 */
void intc_stm8_irq_register(unsigned int irq, const struct device *dev,
			    void (*set_enabled)(const struct device *dev, bool enabled));

#endif /* ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_INTC_STM8_H_ */
