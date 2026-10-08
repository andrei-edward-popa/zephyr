/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_itc

#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>

#define STM8_ITC_SPR_BASE        DT_INST_REG_ADDR(0)
#define STM8_ITC_VECTORS_PER_SPR 4U
#define STM8_ITC_PRIORITY_BITS   2U
#define STM8_ITC_PRIORITY_MASK   3U

struct intc_stm8_source {
	const struct device *dev;
	void (*set_enabled)(const struct device *dev, bool enabled);
	bool enabled;
};

static struct intc_stm8_source intc_stm8_sources[CONFIG_NUM_IRQS];

void intc_stm8_irq_priority_set(unsigned int irq, unsigned int priority)
{
	unsigned int key = irq_lock();
	uintptr_t reg = STM8_ITC_SPR_BASE + irq / STM8_ITC_VECTORS_PER_SPR;
	unsigned int shift = (irq % STM8_ITC_VECTORS_PER_SPR) * STM8_ITC_PRIORITY_BITS;
	uint8_t encoding = priority == 0U ? 3U : priority == 1U ? 0U : 1U;

	__ASSERT(irq < CONFIG_NUM_IRQS && priority < CONFIG_NUM_IRQ_PRIO_LEVELS,
		 "Invalid STM8 IRQ priority");
	/* Hardware level 0 (encoding 2) cannot be written to ITC. */
	sys_write8((sys_read8(reg) & ~(STM8_ITC_PRIORITY_MASK << shift)) | (encoding << shift),
		   reg);
	irq_unlock(key);
}

void intc_stm8_irq_register(unsigned int irq, const struct device *dev,
			    void (*set_enabled)(const struct device *dev, bool enabled))
{
	unsigned int key = irq_lock();

	__ASSERT(irq < CONFIG_NUM_IRQS && set_enabled != NULL, "Invalid STM8 IRQ source");
	__ASSERT(intc_stm8_sources[irq].set_enabled == NULL, "STM8 IRQ source already registered");
	intc_stm8_sources[irq].dev = dev;
	intc_stm8_sources[irq].set_enabled = set_enabled;
	set_enabled(dev, false);
	irq_unlock(key);
}

static void intc_stm8_irq_set_enabled(unsigned int irq, bool enabled)
{
	unsigned int key = irq_lock();

	__ASSERT(irq < CONFIG_NUM_IRQS && intc_stm8_sources[irq].set_enabled != NULL,
		 "Unregistered STM8 IRQ source");
	if (irq < CONFIG_NUM_IRQS && intc_stm8_sources[irq].set_enabled != NULL) {
		intc_stm8_sources[irq].set_enabled(intc_stm8_sources[irq].dev, enabled);
		intc_stm8_sources[irq].enabled = enabled;
	}
	irq_unlock(key);
}

void arch_irq_enable(unsigned int irq)
{
	intc_stm8_irq_set_enabled(irq, true);
}

void arch_irq_disable(unsigned int irq)
{
	intc_stm8_irq_set_enabled(irq, false);
}

int arch_irq_is_enabled(unsigned int irq)
{
	__ASSERT(irq < CONFIG_NUM_IRQS, "Invalid STM8 IRQ number");
	return irq < CONFIG_NUM_IRQS && intc_stm8_sources[irq].enabled;
}
