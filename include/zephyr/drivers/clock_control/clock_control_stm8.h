/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_STM8_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_STM8_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/slist.h>
#include <zephyr/dt-bindings/clock/stm8-clock.h>

/**
 * @brief STM8 master clock configuration for clock_control_configure().
 *
 * Use STM8_CLOCK_MASTER as the subsystem. LSI selection requires the LSI_EN
 * option bit; the driver never programs option bytes. clock_control_set_rate()
 * accepts a pointer to a uint32_t rate in Hz for STM8_CLOCK_CPU or
 * STM8_CLOCK_MASTER. Master rate changes select a divider of the current source.
 */
struct stm8_clock_control_config {
	/** Source: STM8_CLOCK_HSI, STM8_CLOCK_HSE or STM8_CLOCK_LSI. */
	uint8_t source;
	/** HSI divisor: 1, 2, 4 or 8. Ignored by other sources. */
	uint8_t hsi_divisor;
	/** CPU divisor: a power of two from 1 through 128. */
	uint8_t cpu_divisor;
};

/**
 * @internal
 * @brief Clock-dependent driver callbacks for a master or CPU frequency change.
 *
 * Callbacks execute with IRQs locked. prepare() must validate the requested
 * rate without changing hardware; returning an error cancels the transition.
 * changed() updates idle peripheral timing after the clock has changed.
 */
struct clock_control_stm8_client {
	/** List node owned by the clock driver. */
	sys_snode_t node;
	/** STM8_CLOCK_MASTER or STM8_CLOCK_CPU. */
	uint8_t clock_id;
	/** Peripheral device, or NULL for the system timer. */
	const struct device *dev;
	/** Validate a frequency in Hz; return zero or a negative errno. */
	int (*prepare)(const struct device *dev, uint32_t rate);
	/** Apply a previously validated frequency in Hz. */
	void (*changed)(const struct device *dev, uint32_t rate);
};

/**
 * @internal
 * @brief Register a clock-dependent driver during device initialization.
 *
 * @param client Persistent callback descriptor; it must outlive the clock driver.
 * @retval 0 Registration successful.
 * @retval -EINVAL Missing callbacks or a descriptor already registered.
 * @retval -EBUSY A clock transition is in progress.
 */
int clock_control_stm8_register_client(struct clock_control_stm8_client *client);

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_STM8_H_ */
