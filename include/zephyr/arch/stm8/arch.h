/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_ARCH_STM8_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_STM8_ARCH_H_

#define ARCH_STACK_PTR_ALIGN 1

#ifndef _ASMLANGUAGE
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>
#include <zephyr/arch/stm8/thread.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/stm8/sys_bitops.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/arch/stm8/arch_inlines.h>

#include <zephyr/arch/stm8/exception.h>

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	uint8_t key;

	__asm__ volatile("push cc\n\tpop %0\n\tsim" : "=a"(key) : : "cc", "memory");
	return key;
}

static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
	uint8_t cc = key;

	__asm__ volatile("push %0\n\tpop cc" : : "a"(cc) : "cc", "memory");
}

static inline bool arch_irq_unlocked(unsigned int key)
{
	return (key & 0x28U) != 0x28U;
}

static ALWAYS_INLINE bool arch_cpu_irqs_are_enabled(void)
{
	uint8_t cc;

	__asm__ volatile("push cc\n\tpop %0" : "=a"(cc) : : "memory");
	return arch_irq_unlocked(cc);
}

static ALWAYS_INLINE struct _cpu *arch_curr_cpu(void)
{
	return &_kernel.cpus[0];
}

static ALWAYS_INLINE void arch_nop(void)
{
	__asm__ volatile("nop");
}

uint32_t sys_clock_cycle_get_32(void);

static inline uint32_t arch_k_cycle_get_32(void)
{
	return sys_clock_cycle_get_32();
}

uint64_t sys_clock_cycle_get_64(void);

static inline uint64_t arch_k_cycle_get_64(void)
{
	return sys_clock_cycle_get_64();
}

#define ARCH_IRQ_CONNECT(irq, priority, isr, arg, flags)                                           \
	do {                                                                                       \
		Z_ISR_DECLARE(irq, 0, isr, arg);                                                   \
		intc_stm8_irq_priority_set(irq, priority);                                         \
	} while (false)

void z_stm8_fatal(unsigned int reason);
#define ARCH_EXCEPT(reason) z_stm8_fatal(reason)
#endif
#endif
