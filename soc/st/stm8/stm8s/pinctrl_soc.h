/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_SOC_ST_STM8_STM8S_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ST_STM8_STM8S_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <zephyr/dt-bindings/pinctrl/stm8-pinctrl.h>

#define STM8_PIN_OUTPUT     BIT(0)
#define STM8_PIN_PULL_UP    BIT(1)
#define STM8_PIN_OPEN_DRAIN BIT(2)
#define STM8_PIN_FAST       BIT(3)
#define STM8_PIN_HIGH       BIT(4)
#define STM8_PIN_LOW        BIT(5)

typedef struct {
	uint8_t pinmux;
	uint8_t flags;
	uint8_t remap_mask;
	uint8_t remap_value;
} pinctrl_soc_pin_t;

#define Z_PINCTRL_STM8_PIN_INIT(node_id, prop, idx)                                                \
	{                                                                                          \
		.pinmux = DT_PROP_BY_IDX(node_id, prop, idx),                                      \
		.flags = ((DT_PROP(node_id, output_enable) || DT_PROP(node_id, output_high) ||     \
			   DT_PROP(node_id, output_low))                                           \
				  ? STM8_PIN_OUTPUT                                                \
				  : 0) |                                                           \
			 (DT_PROP(node_id, bias_pull_up) ? STM8_PIN_PULL_UP : 0) |                 \
			 (DT_PROP(node_id, drive_open_drain) ? STM8_PIN_OPEN_DRAIN : 0) |          \
			 (DT_PROP(node_id, slew_rate) ? STM8_PIN_FAST : 0) |                       \
			 (DT_PROP(node_id, output_high) ? STM8_PIN_HIGH : 0) |                     \
			 (DT_PROP(node_id, output_low) ? STM8_PIN_LOW : 0),                        \
		.remap_mask = DT_PROP(node_id, st_remap_mask),                                     \
		.remap_value = DT_PROP(node_id, st_remap_value),                                   \
	},

#define Z_PINCTRL_STM8_GROUP_INIT(node_id)                                                         \
	DT_FOREACH_PROP_ELEM(node_id, pinmux, Z_PINCTRL_STM8_PIN_INIT)

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	DT_FOREACH_CHILD(DT_PHANDLE_BY_IDX(node_id, prop, idx), Z_PINCTRL_STM8_GROUP_INIT)

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_PROP_ELEM_SEP(node_id, prop, Z_PINCTRL_STATE_PIN_INIT, ())}

#endif /* ZEPHYR_SOC_ST_STM8_STM8S_PINCTRL_SOC_H_ */
