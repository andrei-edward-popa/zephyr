/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_spi

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>
#include <zephyr/logging/log.h>

#define STM8_SPI_CR1          0x00U
#define STM8_SPI_CR2          0x01U
#define STM8_SPI_ICR          0x02U
#define STM8_SPI_SR           0x03U
#define STM8_SPI_DR           0x04U
#define STM8_SPI_CR1_LSBFIRST 0x80U
#define STM8_SPI_CR1_SPE      0x40U
#define STM8_SPI_CR1_BR_SHIFT 3U
#define STM8_SPI_CR1_MSTR     0x04U
#define STM8_SPI_CR1_CPOL     0x02U
#define STM8_SPI_CR1_CPHA     0x01U
#define STM8_SPI_CR2_SSM      0x02U
#define STM8_SPI_CR2_SSI      0x01U
#define STM8_SPI_ICR_RXIE     0x40U
#define STM8_SPI_ICR_ERRIE    0x20U
#define STM8_SPI_ICR_ALL      0xf0U
#define STM8_SPI_SR_BSY       0x80U
#define STM8_SPI_SR_OVR       0x40U
#define STM8_SPI_SR_MODF      0x20U
#define STM8_SPI_SR_RXNE      0x01U

LOG_MODULE_REGISTER(spi_stm8, CONFIG_SPI_LOG_LEVEL);
#include "spi_context.h"

#define STM8_SPI_IDLE_RETRIES    1024U
#define STM8_SPI_WORD_BITS       8U
#define STM8_SPI_PRESCALER_MAX   7U
#define PINCTRL_STATE_PERIPHERAL PINCTRL_STATE_PRIV_START

struct spi_stm8_config {
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
	bool software_nss;
};

struct spi_stm8_data {
	uint8_t interrupt_mask;
	bool vector_enabled;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	struct clock_control_stm8_client clock_client;
#endif
	struct spi_context ctx;
	struct k_timer watchdog;
	const struct device *dev;
	uint32_t frequency;
	bool active;
	bool peripheral;
};

static void spi_stm8_interrupt_write(const struct device *dev, uint8_t value)
{
	const struct spi_stm8_config *cfg = dev->config;
	struct spi_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	data->interrupt_mask = value;
	sys_write8(data->vector_enabled ? value : 0U, cfg->base + STM8_SPI_ICR);
	irq_unlock(key);
}

static void spi_stm8_irq_set_enabled(const struct device *dev, bool enabled)
{
	const struct spi_stm8_config *cfg = dev->config;
	struct spi_stm8_data *data = dev->data;

	data->vector_enabled = enabled;
	sys_write8(enabled ? data->interrupt_mask : 0U, cfg->base + STM8_SPI_ICR);
}

static void spi_stm8_finish(const struct device *dev, int status)
{
	const struct spi_stm8_config *cfg = dev->config;
	struct spi_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	if (!data->active) {
		irq_unlock(key);
		return;
	}
	data->active = false;
	spi_stm8_interrupt_write(dev, 0U);
	k_timer_stop(&data->watchdog);
	/* RXNE marks a complete byte; bound the final hardware BSY check. */
	for (uint16_t i = 0U; i < STM8_SPI_IDLE_RETRIES; i++) {
		if ((sys_read8(cfg->base + STM8_SPI_SR) & STM8_SPI_SR_BSY) == 0U) {
			break;
		}
		if (i == STM8_SPI_IDLE_RETRIES - 1U) {
			status = -ETIMEDOUT;
		}
	}
	sys_write8(sys_read8(cfg->base + STM8_SPI_CR1) & ~STM8_SPI_CR1_SPE,
		   cfg->base + STM8_SPI_CR1);
	(void)sys_read8(cfg->base + STM8_SPI_DR);
	(void)sys_read8(cfg->base + STM8_SPI_SR);
	spi_context_cs_control(&data->ctx, false);
	spi_context_complete(&data->ctx, dev, status);
	irq_unlock(key);
}

static void spi_stm8_timeout(struct k_timer *timer)
{
	struct spi_stm8_data *data = CONTAINER_OF(timer, struct spi_stm8_data, watchdog);

	spi_stm8_finish(data->dev, -ETIMEDOUT);
}

static void spi_stm8_send_byte(const struct device *dev)
{
	const struct spi_stm8_config *cfg = dev->config;
	struct spi_stm8_data *data = dev->data;
	uint8_t byte = spi_context_tx_buf_on(&data->ctx) ? *data->ctx.tx_buf : 0U;

	spi_context_update_tx(&data->ctx, 1U, 1U);
	sys_write8(byte, cfg->base + STM8_SPI_DR);
}

static void spi_stm8_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct spi_stm8_config *cfg = dev->config;
	struct spi_stm8_data *data = dev->data;
	uint8_t status = sys_read8(cfg->base + STM8_SPI_SR);

	if (!data->active) {
		return;
	}
	if ((status & (STM8_SPI_SR_OVR | STM8_SPI_SR_MODF)) != 0U) {
		spi_stm8_finish(dev, -EIO);
		return;
	}
	if ((status & STM8_SPI_SR_RXNE) == 0U) {
		return;
	}
	uint8_t byte = sys_read8(cfg->base + STM8_SPI_DR);

	if (spi_context_rx_buf_on(&data->ctx)) {
		*data->ctx.rx_buf = byte;
	}
	spi_context_update_rx(&data->ctx, 1U, 1U);
	if (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
		spi_stm8_send_byte(dev);
	} else {
		spi_stm8_finish(dev, 0);
	}
}

static int spi_stm8_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_stm8_config *cfg = dev->config;
	struct spi_stm8_data *data = dev->data;
	uint32_t rate;
	uint8_t divider = 0U;
	bool peripheral = (config->operation & SPI_OP_MODE_PERIPHERAL) != 0U;
	uint8_t control = peripheral ? 0U : STM8_SPI_CR1_MSTR;
	int err;

	if ((config->operation & (SPI_MODE_LOOP | SPI_HALF_DUPLEX | SPI_LINES_MASK)) != 0U ||
	    SPI_WORD_SIZE_GET(config->operation) != STM8_SPI_WORD_BITS) {
		return -ENOTSUP;
	}
	if (peripheral && (!IS_ENABLED(CONFIG_SPI_PERIPHERAL) || spi_cs_is_gpio(config) ||
			   (config->operation & (SPI_HOLD_ON_CS | SPI_CS_ACTIVE_HIGH)) != 0U)) {
		return -ENOTSUP;
	}
	if ((!peripheral && config->frequency == 0U) ||
	    (!spi_cs_is_gpio(config) && config->peripheral != 0U)) {
		return -EINVAL;
	}
	err = clock_control_get_rate(cfg->clock, cfg->clock_id, &rate);
	if (err != 0) {
		return err;
	}
	rate /= 2U;
	while (!peripheral && rate > config->frequency && divider < STM8_SPI_PRESCALER_MAX) {
		rate /= 2U;
		divider++;
	}
	if (!peripheral && rate > config->frequency) {
		return -EINVAL;
	}
	control |= divider << STM8_SPI_CR1_BR_SHIFT;
	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		control |= STM8_SPI_CR1_CPOL;
	}
	if ((config->operation & SPI_MODE_CPHA) != 0U) {
		control |= STM8_SPI_CR1_CPHA;
	}
	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		control |= STM8_SPI_CR1_LSBFIRST;
	}
	err = pinctrl_apply_state(cfg->pcfg,
				  peripheral ? PINCTRL_STATE_PERIPHERAL : PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}
	sys_write8(0U, cfg->base + STM8_SPI_CR1);
	sys_write8(peripheral ? (cfg->software_nss ? STM8_SPI_CR2_SSM : 0U)
			      : STM8_SPI_CR2_SSM | STM8_SPI_CR2_SSI,
		   cfg->base + STM8_SPI_CR2);
	/* Set MSTR and SPE together, as required by ES036 section 2.5. */
	sys_write8(control | (peripheral ? 0U : STM8_SPI_CR1_SPE), cfg->base + STM8_SPI_CR1);
	data->frequency = rate;
	data->peripheral = peripheral;
	data->ctx.config = config;
	return 0;
}

static int spi_stm8_transceive_common(const struct device *dev, const struct spi_config *config,
				      const struct spi_buf_set *tx, const struct spi_buf_set *rx,
				      bool asynchronous, spi_callback_t callback, void *user_data)
{
	const struct spi_stm8_config *cfg = dev->config;
	struct spi_stm8_data *data = dev->data;
	int err;

	spi_context_lock(&data->ctx, asynchronous, callback, user_data, config);
	err = spi_stm8_configure(dev, config);
	if (err != 0) {
		spi_context_release(&data->ctx, err);
		return err;
	}
	spi_context_buffers_setup(&data->ctx, tx, rx, 1U);
	if (data->peripheral && data->ctx.max_count > INT_MAX) {
		spi_context_release(&data->ctx, -EINVAL);
		return -EINVAL;
	}
	if (!spi_context_tx_on(&data->ctx) && !spi_context_rx_on(&data->ctx)) {
		sys_write8(0U, cfg->base + STM8_SPI_CR1);
		spi_context_complete(&data->ctx, dev, 0);
	} else {
		uint32_t timeout_ms = (uint32_t)data->ctx.max_count * STM8_SPI_WORD_BITS * 1000U /
					      data->frequency +
				      CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE;

		if (!data->peripheral) {
			spi_context_cs_control(&data->ctx, true);
		}
		data->active = true;
		if (!data->peripheral) {
			k_timer_start(&data->watchdog, K_MSEC(MAX(timeout_ms, 1U)), K_NO_WAIT);
		}
		/* Preload DR before enabling a CPHA=0 peripheral. */
		spi_stm8_send_byte(dev);
		spi_stm8_interrupt_write(dev, STM8_SPI_ICR_RXIE | STM8_SPI_ICR_ERRIE);
		if (data->peripheral) {
			sys_write8(sys_read8(cfg->base + STM8_SPI_CR1) | STM8_SPI_CR1_SPE,
				   cfg->base + STM8_SPI_CR1);
		}
	}
	if (!asynchronous) {
		err = k_sem_take(&data->ctx.sync, K_FOREVER);
		if (err == 0) {
			err = data->ctx.sync_status;
#ifdef CONFIG_SPI_PERIPHERAL
			if (err == 0 && data->peripheral) {
				err = data->ctx.recv_frames;
			}
#endif
		}
	}
	spi_context_release(&data->ctx, err);
	return err;
}

static int spi_stm8_transceive(const struct device *dev, const struct spi_config *config,
			       const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	return spi_stm8_transceive_common(dev, config, tx, rx, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_stm8_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx, const struct spi_buf_set *rx,
				     spi_callback_t callback, void *user_data)
{
	return spi_stm8_transceive_common(dev, config, tx, rx, true, callback, user_data);
}
#endif

static int spi_stm8_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_stm8_data *data = dev->data;

	if (!spi_context_configured(&data->ctx, config) || data->active) {
		return -EINVAL;
	}
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static int spi_stm8_clock_prepare(const struct device *dev, uint32_t rate)
{
	struct spi_stm8_data *data = dev->data;

	ARG_UNUSED(rate);
	return data->active || data->ctx.owner != NULL ? -EBUSY : 0;
}

static void spi_stm8_clock_changed(const struct device *dev, uint32_t rate)
{
	/* Each transfer derives its divider from the current clock rate. */
	ARG_UNUSED(dev);
	ARG_UNUSED(rate);
}
#endif

static int spi_stm8_init(const struct device *dev)
{
	const struct spi_stm8_config *cfg = dev->config;
	struct spi_stm8_data *data = dev->data;
	int err;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}
	err = clock_control_on(cfg->clock, cfg->clock_id);
	if (err != 0) {
		return err;
	}
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}
	err = spi_context_cs_configure_all(&data->ctx);
	if (err != 0) {
		return err;
	}
	data->dev = dev;
	k_timer_init(&data->watchdog, spi_stm8_timeout, NULL);
	spi_stm8_interrupt_write(dev, 0U);
	sys_write8(0U, cfg->base + STM8_SPI_CR1);
	cfg->irq_config(dev);
	spi_context_unlock_unconditionally(&data->ctx);
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	data->clock_client.clock_id = STM8_CLOCK_MASTER;
	data->clock_client.dev = dev;
	data->clock_client.prepare = spi_stm8_clock_prepare;
	data->clock_client.changed = spi_stm8_clock_changed;
	return clock_control_stm8_register_client(&data->clock_client);
#else
	return 0;
#endif
}

static DEVICE_API(spi, spi_stm8_driver_api) = {
	.transceive = spi_stm8_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_stm8_transceive_async,
#endif
	.release = spi_stm8_release,
};

#define SPI_STM8_INIT(n)                                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void spi_stm8_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		intc_stm8_irq_register(DT_INST_IRQN(n), DEVICE_DT_INST_GET(n),                     \
				       spi_stm8_irq_set_enabled);                                  \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), spi_stm8_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
		ARG_UNUSED(dev);                                                                   \
	}                                                                                          \
	static struct spi_stm8_data spi_stm8_data_##n = {                                          \
		SPI_CONTEXT_INIT_LOCK(spi_stm8_data_##n, ctx),                                     \
		SPI_CONTEXT_INIT_SYNC(spi_stm8_data_##n, ctx),                                     \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	static const struct spi_stm8_config spi_stm8_config_##n = {                                \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config = spi_stm8_irq_config_##n,                                             \
		.software_nss = DT_INST_PROP(n, st_software_nss),                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, spi_stm8_init, NULL, &spi_stm8_data_##n, &spi_stm8_config_##n,    \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_stm8_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_STM8_INIT)
