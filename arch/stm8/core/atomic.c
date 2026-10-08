/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/atomic.h>

BUILD_ASSERT(!IS_ENABLED(CONFIG_SMP));

bool atomic_cas(atomic_t *target, atomic_val_t old_value, atomic_val_t new_value)
{
	unsigned int key = arch_irq_lock();
	bool changed = *target == old_value;

	if (changed) {
		*target = new_value;
	}
	arch_irq_unlock(key);
	return changed;
}

bool atomic_ptr_cas(atomic_ptr_t *target, void *old_value, void *new_value)
{
	unsigned int key = arch_irq_lock();
	bool changed = *target == old_value;

	if (changed) {
		*target = new_value;
	}
	arch_irq_unlock(key);
	return changed;
}

atomic_val_t atomic_add(atomic_t *target, atomic_val_t value)
{
	unsigned int key = arch_irq_lock();
	atomic_val_t old_value = *target;

	*target = (atomic_val_t)((unsigned long)old_value + (unsigned long)value);
	arch_irq_unlock(key);
	return old_value;
}

atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value)
{
	unsigned int key = arch_irq_lock();
	atomic_val_t old_value = *target;

	*target = (atomic_val_t)((unsigned long)old_value - (unsigned long)value);
	arch_irq_unlock(key);
	return old_value;
}

atomic_val_t atomic_or(atomic_t *target, atomic_val_t value)
{
	unsigned int key = arch_irq_lock();
	atomic_val_t old_value = *target;

	*target = (atomic_val_t)(old_value | value);
	arch_irq_unlock(key);
	return old_value;
}

atomic_val_t atomic_xor(atomic_t *target, atomic_val_t value)
{
	unsigned int key = arch_irq_lock();
	atomic_val_t old_value = *target;

	*target = (atomic_val_t)(old_value ^ value);
	arch_irq_unlock(key);
	return old_value;
}

atomic_val_t atomic_and(atomic_t *target, atomic_val_t value)
{
	unsigned int key = arch_irq_lock();
	atomic_val_t old_value = *target;

	*target = (atomic_val_t)(old_value & value);
	arch_irq_unlock(key);
	return old_value;
}

atomic_val_t atomic_nand(atomic_t *target, atomic_val_t value)
{
	unsigned int key = arch_irq_lock();
	atomic_val_t old_value = *target;

	*target = (atomic_val_t)(~(old_value & value));
	arch_irq_unlock(key);
	return old_value;
}

atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
	unsigned int key = arch_irq_lock();
	atomic_val_t old_value = *target;

	*target = (atomic_val_t)(value);
	arch_irq_unlock(key);
	return old_value;
}

atomic_val_t atomic_get(const atomic_t *target)
{
	unsigned int key = arch_irq_lock();
	atomic_val_t value = *target;

	arch_irq_unlock(key);
	return value;
}

void *atomic_ptr_get(const atomic_ptr_t *target)
{
	unsigned int key = arch_irq_lock();
	void *value = *target;

	arch_irq_unlock(key);
	return value;
}

void *atomic_ptr_set(atomic_ptr_t *target, void *value)
{
	unsigned int key = arch_irq_lock();
	void *old_value = *target;

	*target = value;
	arch_irq_unlock(key);
	return old_value;
}

atomic_val_t atomic_inc(atomic_t *target)
{
	return atomic_add(target, 1);
}

atomic_val_t atomic_dec(atomic_t *target)
{
	return atomic_sub(target, 1);
}

atomic_val_t atomic_clear(atomic_t *target)
{
	return atomic_set(target, 0);
}

void *atomic_ptr_clear(atomic_ptr_t *target)
{
	return atomic_ptr_set(target, NULL);
}
