/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_TIMER_STM8_TIM4_H_
#define ZEPHYR_INCLUDE_DRIVERS_TIMER_STM8_TIM4_H_

/**
 * @internal
 * @brief Account for TIM4 rollover while Flash is unavailable.
 *
 * This function executes from RAM with interrupts already locked. It retains
 * reference cycles without invoking the kernel; the next timer interrupt
 * announces elapsed ticks after the Flash operation has finished.
 */
void stm8_tim4_poll(void);

#endif /* ZEPHYR_INCLUDE_DRIVERS_TIMER_STM8_TIM4_H_ */
