/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_STM8_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_STM8_CLOCK_H_

/* Peripheral IDs are one plus the bit index in PCKENR1/PCKENR2. */
#define STM8_CLOCK_I2C    1
#define STM8_CLOCK_SPI    2
#define STM8_CLOCK_UART1  3
#define STM8_CLOCK_UART3  4
#define STM8_CLOCK_TIM4   5
#define STM8_CLOCK_TIM2   6
#define STM8_CLOCK_TIM3   7
#define STM8_CLOCK_TIM1   8
#define STM8_CLOCK_AWU    11
#define STM8_CLOCK_ADC    12
#define STM8_CLOCK_CAN    16
#define STM8_CLOCK_CPU    17
#define STM8_CLOCK_MASTER 18
#define STM8_CLOCK_HSI    19
#define STM8_CLOCK_HSE    20
#define STM8_CLOCK_LSI    21

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_STM8_CLOCK_H_ */
