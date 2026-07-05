/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT raspberrypi_pico_mdio_pio

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/kernel.h>

/* Pico SDK includes */
#include <hardware/clocks.h>
#include <hardware/pio.h>

#define MDIO_PIO_C22_ST       1
#define MDIO_PIO_C22_OP_WRITE 1
#define MDIO_PIO_C22_OP_READ  2

#define MDIO_PIO_C45_ST       0
#define MDIO_PIO_C45_OP_ADDR  0
#define MDIO_PIO_C45_OP_WRITE 1
#define MDIO_PIO_C45_OP_READ  3

#define MDIO_PIO_OP_MASK   0x3
#define MDIO_PIO_ADDR_MASK 0x1F
#define MDIO_PIO_ADDR_MAX  MDIO_PIO_ADDR_MASK

#define MDIO_PIO_TA_WRITE 0x2
#define MDIO_PIO_PREAMBLE 0xFFFF

#define MDIO_PIO_FRAME_BITS  16
#define MDIO_PIO_HEADER_BITS 14

#define MDIO_PIO_FRAME_ST_SHIFT    14
#define MDIO_PIO_FRAME_OP_SHIFT    12
#define MDIO_PIO_FRAME_PRTAD_SHIFT 7
#define MDIO_PIO_FRAME_ADDR_SHIFT  2

#define MDIO_PIO_HEADER_ST_SHIFT    12
#define MDIO_PIO_HEADER_OP_SHIFT    10
#define MDIO_PIO_HEADER_PRTAD_SHIFT 5

#define MDIO_PIO_MDC_CYCLES 4

#define MDIO_PIO_FRAME16(st, op, prtad, addr)                                                      \
	(((st) << MDIO_PIO_FRAME_ST_SHIFT) |                                                       \
	 (((op) & MDIO_PIO_OP_MASK) << MDIO_PIO_FRAME_OP_SHIFT) |                                  \
	 (((prtad) & MDIO_PIO_ADDR_MASK) << MDIO_PIO_FRAME_PRTAD_SHIFT) |                          \
	 (((addr) & MDIO_PIO_ADDR_MASK) << MDIO_PIO_FRAME_ADDR_SHIFT) | MDIO_PIO_TA_WRITE)

#define MDIO_PIO_HEADER14(st, op, prtad, addr)                                                     \
	(((st) << MDIO_PIO_HEADER_ST_SHIFT) |                                                      \
	 (((op) & MDIO_PIO_OP_MASK) << MDIO_PIO_HEADER_OP_SHIFT) |                                 \
	 (((prtad) & MDIO_PIO_ADDR_MASK) << MDIO_PIO_HEADER_PRTAD_SHIFT) |                         \
	 ((addr) & MDIO_PIO_ADDR_MASK))

RPI_PICO_PIO_DEFINE_PROGRAM(mdio_mdc, 0, 1,
		/*     .wrap_target                    */
	0xc120, /*  0: irq    wait 0       side 0 [1] */
	0xd121, /*  1: irq    wait 1       side 1 [1] */
		/*     .wrap                           */
);

RPI_PICO_PIO_DEFINE_PROGRAM(mdio_write, 0, 6,
		/*     .wrap_target      */
	0x80a0, /*  0: pull   block      */
	0xa0ef, /*  1: mov    osr, ~osr  */
	0xe02f, /*  2: set    x, 15      */
	0x20c0, /*  3: wait   1 irq, 0   */
	0x6081, /*  4: out    pindirs, 1 */
	0x20c1, /*  5: wait   1 irq, 1   */
	0x0043, /*  6: jmp    x--, 3     */
		/*     .wrap             */
);

RPI_PICO_PIO_DEFINE_PROGRAM(mdio_read, 0, 19,
		/*     .wrap_target      */
	0x80a0, /*  0: pull   block      */
	0xa0ef, /*  1: mov    osr, ~osr  */
	0xe02d, /*  2: set    x, 13      */
	0x20c0, /*  3: wait   1 irq, 0   */
	0x6081, /*  4: out    pindirs, 1 */
	0x20c1, /*  5: wait   1 irq, 1   */
	0x0043, /*  6: jmp    x--, 3     */
	0x20c0, /*  7: wait   1 irq, 0   */
	0xe080, /*  8: set    pindirs, 0 */
	0x20c1, /*  9: wait   1 irq, 1   */
	0x20c0, /* 10: wait   1 irq, 0   */
	0xe080, /* 11: set    pindirs, 0 */
	0x20c1, /* 12: wait   1 irq, 1   */
	0xe02f, /* 13: set    x, 15      */
	0x20c0, /* 14: wait   1 irq, 0   */
	0xe080, /* 15: set    pindirs, 0 */
	0x20c1, /* 16: wait   1 irq, 1   */
	0x4001, /* 17: in     pins, 1    */
	0x004e, /* 18: jmp    x--, 14    */
	0x8020, /* 19: push   block      */
		/*     .wrap             */
);

struct mdio_rpi_pico_pio_config {
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	uint32_t mdc_pin;
	uint32_t mdio_pin;
	uint32_t clock_frequency;
};

struct mdio_rpi_pico_pio_data {
	size_t sm_mdc;
	size_t sm_write;
	size_t sm_read;
	struct k_mutex lock;
};

static inline float mdio_pio_clkdiv(uint32_t freq)
{
	return (float)clock_get_hz(clk_sys) / (freq * MDIO_PIO_MDC_CYCLES);
}

static void mdio_pio_write_word(PIO pio, uint32_t sm, uint16_t word)
{
	uint32_t osr_aligned_word;

	osr_aligned_word = (uint32_t)word;
	osr_aligned_word <<= (32 - MDIO_PIO_FRAME_BITS);

	pio_sm_put_blocking(pio, sm, osr_aligned_word);
}

static void mdio_pio_wait_write_complete(PIO pio, uint32_t sm, size_t words, uint32_t mdc_freq)
{
	/*
	 * The write SM streams until TX FIFO becomes empty, then stalls at
	 * pull block. Wait enough time for all queued bits to leave the wire.
	 */
	k_busy_wait(DIV_ROUND_UP(words * MDIO_PIO_FRAME_BITS * USEC_PER_SEC, mdc_freq));

	pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 0));
}

static void mdio_pio_write_preamble(PIO pio, uint32_t sm, uint32_t mdc_freq)
{
	pio_sm_clear_fifos(pio, sm);
	mdio_pio_write_word(pio, sm, MDIO_PIO_PREAMBLE);
	mdio_pio_write_word(pio, sm, MDIO_PIO_PREAMBLE);
	mdio_pio_wait_write_complete(pio, sm, 2, mdc_freq);
}

static void mdio_pio_write_frame(PIO pio, uint32_t sm, uint16_t frame, uint16_t payload,
				 uint32_t mdc_freq)
{
	pio_sm_clear_fifos(pio, sm);
	mdio_pio_write_word(pio, sm, MDIO_PIO_PREAMBLE);
	mdio_pio_write_word(pio, sm, MDIO_PIO_PREAMBLE);
	mdio_pio_write_word(pio, sm, frame);
	mdio_pio_write_word(pio, sm, payload);
	mdio_pio_wait_write_complete(pio, sm, 4, mdc_freq);
}

static uint16_t mdio_pio_read_frame(PIO pio, uint32_t sm, uint16_t header, uint32_t mdc_freq)
{
	uint32_t osr_aligned_header;
	uint32_t raw;

	osr_aligned_header = (uint32_t)(header & BIT_MASK(MDIO_PIO_HEADER_BITS));
	osr_aligned_header <<= (32 - MDIO_PIO_HEADER_BITS);

	pio_sm_clear_fifos(pio, sm);
	pio_sm_put_blocking(pio, sm, osr_aligned_header);

	raw = pio_sm_get_blocking(pio, sm);

	pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 0));

	return (uint16_t)raw;
}

static int mdio_rpi_pico_pio_read_c22(const struct device *dev, uint8_t prtad, uint8_t regad,
				      uint16_t *regval)
{
	const struct mdio_rpi_pico_pio_config *config = dev->config;
	struct mdio_rpi_pico_pio_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	uint16_t header;

	if ((!regval) || (prtad > MDIO_PIO_ADDR_MAX) || (regad > MDIO_PIO_ADDR_MAX)) {
		return -EINVAL;
	}

	header = MDIO_PIO_HEADER14(MDIO_PIO_C22_ST, MDIO_PIO_C22_OP_READ, prtad, regad);

	k_mutex_lock(&data->lock, K_FOREVER);

	mdio_pio_write_preamble(pio, data->sm_write, config->clock_frequency);
	*regval = mdio_pio_read_frame(pio, data->sm_read, header, config->clock_frequency);

	k_mutex_unlock(&data->lock);

	return 0;
}

static int mdio_rpi_pico_pio_write_c22(const struct device *dev, uint8_t prtad, uint8_t regad,
				       uint16_t regval)
{
	const struct mdio_rpi_pico_pio_config *config = dev->config;
	struct mdio_rpi_pico_pio_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	uint16_t frame;

	if ((prtad > MDIO_PIO_ADDR_MAX) || (regad > MDIO_PIO_ADDR_MAX)) {
		return -EINVAL;
	}

	frame = MDIO_PIO_FRAME16(MDIO_PIO_C22_ST, MDIO_PIO_C22_OP_WRITE, prtad, regad);

	k_mutex_lock(&data->lock, K_FOREVER);

	mdio_pio_write_frame(pio, data->sm_write, frame, regval, config->clock_frequency);

	k_mutex_unlock(&data->lock);

	return 0;
}

static void mdio_rpi_pico_pio_c45_address(const struct device *dev, uint8_t prtad, uint8_t devad,
					  uint16_t regad)
{
	const struct mdio_rpi_pico_pio_config *config = dev->config;
	struct mdio_rpi_pico_pio_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	uint16_t frame;

	frame = MDIO_PIO_FRAME16(MDIO_PIO_C45_ST, MDIO_PIO_C45_OP_ADDR, prtad, devad);
	mdio_pio_write_frame(pio, data->sm_write, frame, regad, config->clock_frequency);
}

static int mdio_rpi_pico_pio_read_c45(const struct device *dev, uint8_t prtad, uint8_t devad,
				      uint16_t regad, uint16_t *regval)
{
	const struct mdio_rpi_pico_pio_config *config = dev->config;
	struct mdio_rpi_pico_pio_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	uint16_t header;

	if ((!regval) || (prtad > MDIO_PIO_ADDR_MAX) || (devad > MDIO_PIO_ADDR_MAX)) {
		return -EINVAL;
	}

	header = MDIO_PIO_HEADER14(MDIO_PIO_C45_ST, MDIO_PIO_C45_OP_READ, prtad, devad);

	k_mutex_lock(&data->lock, K_FOREVER);

	mdio_rpi_pico_pio_c45_address(dev, prtad, devad, regad);
	mdio_pio_write_preamble(pio, data->sm_write, config->clock_frequency);
	*regval = mdio_pio_read_frame(pio, data->sm_read, header, config->clock_frequency);

	k_mutex_unlock(&data->lock);

	return 0;
}

static int mdio_rpi_pico_pio_write_c45(const struct device *dev, uint8_t prtad, uint8_t devad,
				       uint16_t regad, uint16_t regval)
{
	const struct mdio_rpi_pico_pio_config *config = dev->config;
	struct mdio_rpi_pico_pio_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);
	uint16_t frame;

	if ((prtad > MDIO_PIO_ADDR_MAX) || (devad > MDIO_PIO_ADDR_MAX)) {
		return -EINVAL;
	}

	frame = MDIO_PIO_FRAME16(MDIO_PIO_C45_ST, MDIO_PIO_C45_OP_WRITE, prtad, devad);

	k_mutex_lock(&data->lock, K_FOREVER);

	mdio_rpi_pico_pio_c45_address(dev, prtad, devad, regad);
	mdio_pio_write_frame(pio, data->sm_write, frame, regval, config->clock_frequency);

	k_mutex_unlock(&data->lock);

	return 0;
}

static int pio_mdio_mdc_init(PIO pio, uint32_t sm, uint32_t mdc_pin, float div)
{
	pio_sm_config sm_config = pio_get_default_sm_config();
	uint32_t offset;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(mdio_mdc))) {
		return -EBUSY;
	}

	offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(mdio_mdc));

	sm_config_set_sideset(&sm_config, 1, false, false);
	sm_config_set_sideset_pins(&sm_config, mdc_pin);
	sm_config_set_clkdiv(&sm_config, div);
	sm_config_set_wrap(&sm_config, offset + RPI_PICO_PIO_GET_WRAP_TARGET(mdio_mdc),
			   offset + RPI_PICO_PIO_GET_WRAP(mdio_mdc));

	pio_sm_set_pins_with_mask(pio, sm, 0U, BIT(mdc_pin));
	pio_sm_set_pindirs_with_mask(pio, sm, BIT(mdc_pin), BIT(mdc_pin));

	pio_sm_init(pio, sm, offset, &sm_config);
	pio_sm_set_enabled(pio, sm, true);

	return 0;
}

static int pio_mdio_write_init(PIO pio, uint32_t sm, uint32_t mdio_pin)
{
	pio_sm_config sm_config = pio_get_default_sm_config();
	uint32_t offset;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(mdio_write))) {
		return -EBUSY;
	}

	offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(mdio_write));

	sm_config_set_out_pins(&sm_config, mdio_pin, 1);
	sm_config_set_set_pins(&sm_config, mdio_pin, 1);
	sm_config_set_out_shift(&sm_config, false, false, 32);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
	sm_config_set_wrap(&sm_config, offset + RPI_PICO_PIO_GET_WRAP_TARGET(mdio_write),
			   offset + RPI_PICO_PIO_GET_WRAP(mdio_write));

	pio_sm_set_pins_with_mask(pio, sm, 0U, BIT(mdio_pin));
	pio_sm_set_pindirs_with_mask(pio, sm, 0U, BIT(mdio_pin));

	pio_sm_init(pio, sm, offset, &sm_config);
	pio_sm_set_enabled(pio, sm, true);

	return 0;
}

static int pio_mdio_read_init(PIO pio, uint32_t sm, uint32_t mdio_pin)
{
	pio_sm_config sm_config = pio_get_default_sm_config();
	uint32_t offset;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(mdio_read))) {
		return -EBUSY;
	}

	offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(mdio_read));

	sm_config_set_out_pins(&sm_config, mdio_pin, 1);
	sm_config_set_set_pins(&sm_config, mdio_pin, 1);
	sm_config_set_in_pins(&sm_config, mdio_pin);
	sm_config_set_out_shift(&sm_config, false, false, 32);
	sm_config_set_in_shift(&sm_config, false, false, 32);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_NONE);
	sm_config_set_wrap(&sm_config, offset + RPI_PICO_PIO_GET_WRAP_TARGET(mdio_read),
			   offset + RPI_PICO_PIO_GET_WRAP(mdio_read));

	pio_sm_set_pins_with_mask(pio, sm, 0U, BIT(mdio_pin));
	pio_sm_set_pindirs_with_mask(pio, sm, 0U, BIT(mdio_pin));

	pio_sm_init(pio, sm, offset, &sm_config);
	pio_sm_set_enabled(pio, sm, true);

	return 0;
}

static int mdio_rpi_pico_pio_init(const struct device *dev)
{
	const struct mdio_rpi_pico_pio_config *config = dev->config;
	struct mdio_rpi_pico_pio_data *data = dev->data;
	float div;
	PIO pio;
	int ret;

	if (!device_is_ready(config->piodev)) {
		return -ENODEV;
	}

	k_mutex_init(&data->lock);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	pio = pio_rpi_pico_get_pio(config->piodev);
	div = mdio_pio_clkdiv(config->clock_frequency);

	ret = pio_rpi_pico_allocate_sm(config->piodev, &data->sm_mdc);
	if (ret != 0) {
		return ret;
	}

	ret = pio_rpi_pico_allocate_sm(config->piodev, &data->sm_write);
	if (ret != 0) {
		return ret;
	}

	ret = pio_rpi_pico_allocate_sm(config->piodev, &data->sm_read);
	if (ret != 0) {
		return ret;
	}

	ret = pio_mdio_write_init(pio, data->sm_write, config->mdio_pin);
	if (ret != 0) {
		return ret;
	}

	ret = pio_mdio_read_init(pio, data->sm_read, config->mdio_pin);
	if (ret != 0) {
		return ret;
	}

	ret = pio_mdio_mdc_init(pio, data->sm_mdc, config->mdc_pin, div);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static struct mdio_driver_api mdio_rpi_pico_pio_api = {
	.read = mdio_rpi_pico_pio_read_c22,
	.write = mdio_rpi_pico_pio_write_c22,
	.read_c45 = mdio_rpi_pico_pio_read_c45,
	.write_c45 = mdio_rpi_pico_pio_write_c45,
};

#define MDIO_RPI_PICO_PIO_INIT(inst)                                                               \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static const struct mdio_rpi_pico_pio_config mdio_rpi_pico_pio_config_##inst = {           \
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.mdc_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(inst, default, 0, mdc_pins, 0),        \
		.mdio_pin = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(inst, default, 0, mdio_pins, 0),      \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                            \
	};                                                                                         \
                                                                                                   \
	static struct mdio_rpi_pico_pio_data mdio_rpi_pico_pio_data_##inst;                        \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, mdio_rpi_pico_pio_init, NULL, &mdio_rpi_pico_pio_data_##inst,  \
			      &mdio_rpi_pico_pio_config_##inst, POST_KERNEL,                       \
			      CONFIG_MDIO_INIT_PRIORITY, &mdio_rpi_pico_pio_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_RPI_PICO_PIO_INIT)
