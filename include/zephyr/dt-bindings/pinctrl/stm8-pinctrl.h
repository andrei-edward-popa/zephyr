/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_STM8_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_STM8_PINCTRL_H_

#define STM8_PORT_A            0
#define STM8_PORT_B            1
#define STM8_PORT_C            2
#define STM8_PORT_D            3
#define STM8_PORT_E            4
#define STM8_PORT_F            5
#define STM8_PINMUX_PORT_SHIFT 3
#define STM8_PINMUX_PIN_MASK   0x7
#define STM8_PINMUX(port, pin) (((port) << STM8_PINMUX_PORT_SHIFT) | (pin))

/* AFR6 remaps I2C from PE1/PE2 to the true open-drain PB4/PB5 pins. */
#define STM8_PINCTRL_AFR6_I2C 0x40

/* AFR7 selects BEEP instead of TIM2_CH1 on PD4. */
#define STM8_PINCTRL_AFR7_BEEP 0x80

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_STM8_PINCTRL_H_ */
