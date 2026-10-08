/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_ARCH_STM8_FAR_IO_H_
#define ZEPHYR_INCLUDE_ARCH_STM8_FAR_IO_H_

#define ARCH_STM8_FAR_WRITE_CONTROL 0
#define ARCH_STM8_FAR_WRITE_SETUP   2
#define ARCH_STM8_FAR_WRITE_FINISH  4
#define ARCH_STM8_FAR_WRITE_STATUS  6
#define ARCH_STM8_FAR_WRITE_MASK    8
#define ARCH_STM8_FAR_WRITE_COUNT   9
#define ARCH_STM8_FAR_WRITE_POLL    10

#ifndef _ASMLANGUAGE

#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/util.h>

struct arch_stm8_far_write_config {
	volatile uint8_t *control;
	uint8_t setup[2];
	uint8_t finish[2];
	volatile uint8_t *status;
	uint8_t ready_mask;
	uint8_t count;
	void (*poll)(void);
};

BUILD_ASSERT(offsetof(struct arch_stm8_far_write_config, control) == ARCH_STM8_FAR_WRITE_CONTROL);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write_config, setup) == ARCH_STM8_FAR_WRITE_SETUP);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write_config, finish) == ARCH_STM8_FAR_WRITE_FINISH);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write_config, status) == ARCH_STM8_FAR_WRITE_STATUS);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write_config, ready_mask) == ARCH_STM8_FAR_WRITE_MASK);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write_config, count) == ARCH_STM8_FAR_WRITE_COUNT);
BUILD_ASSERT(offsetof(struct arch_stm8_far_write_config, poll) == ARCH_STM8_FAR_WRITE_POLL);

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
 * @brief Execute a transfer and control sequence entirely from RAM.
 * @param address Physical address below 0x1000000.
 * @param data One through four source bytes residing in RAM.
 * @param config Control bytes, byte count, status and optional poll callback, residing in RAM.
 *
 * The setup and finish values apply to two adjacent control registers.
 * Interrupts remain masked during setup, transfer, polling and cleanup.
 * A poll callback must reside entirely in RAM and must preserve shared transfer state.
 *
 * @return Status value containing a bit from ready_mask, or zero on timeout.
 */
uint8_t arch_stm8_far_write(uint32_t address, const uint8_t *data,
			    const struct arch_stm8_far_write_config *config);

/** @endcond */

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_STM8_FAR_IO_H_ */
