/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_ARCH_STM8_FAR_IO_H_
#define ZEPHYR_INCLUDE_ARCH_STM8_FAR_IO_H_

#define ARCH_STM8_FAR_WRITE4_CONTROL 0
#define ARCH_STM8_FAR_WRITE4_SETUP   2
#define ARCH_STM8_FAR_WRITE4_FINISH  4
#define ARCH_STM8_FAR_WRITE4_STATUS  6
#define ARCH_STM8_FAR_WRITE4_MASK    8

#ifndef _ASMLANGUAGE

#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/util.h>

struct arch_stm8_far_write4_config {
	volatile uint8_t *control;
	uint8_t setup[2];
	uint8_t finish[2];
	volatile uint8_t *status;
	uint8_t ready_mask;
};

BUILD_ASSERT(offsetof(struct arch_stm8_far_write4_config, control) == ARCH_STM8_FAR_WRITE4_CONTROL);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write4_config, setup) == ARCH_STM8_FAR_WRITE4_SETUP);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write4_config, finish) == ARCH_STM8_FAR_WRITE4_FINISH);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write4_config, status) == ARCH_STM8_FAR_WRITE4_STATUS);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write4_config, ready_mask) == ARCH_STM8_FAR_WRITE4_MASK);

/** @cond INTERNAL_HIDDEN */

/**
 * @brief Read a byte from the 24-bit physical address space.
 * @param address Physical address below 0x1000000.
 * @return Byte stored at the address.
 */
uint8_t arch_stm8_far_read8(uint32_t address);
/**
 * @brief Write a byte to the 24-bit physical address space.
 * @param address Physical address below 0x1000000.
 * @param value Byte to write.
 */
void arch_stm8_far_write8(uint32_t address, uint8_t value);
/**
 * @brief Execute a four-byte transfer and control sequence entirely from RAM.
 * @param address Physical address below 0x1000000.
 * @param data Four source bytes residing in RAM.
 * @param config Control bytes, status address and ready mask, residing in RAM.
 *
 * The setup and finish values apply to two adjacent control registers.
 * Interrupts remain masked during setup, transfer, polling and cleanup.
 *
 * @return Status value containing a bit from ready_mask, or zero on timeout.
 */
uint8_t arch_stm8_far_write4(uint32_t address, const uint8_t *data,
			   const struct arch_stm8_far_write4_config *config);

/** @endcond */

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_STM8_FAR_IO_H_ */
