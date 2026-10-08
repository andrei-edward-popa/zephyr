/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_uart

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>

#define STM8_UART_SR             0x00U
#define STM8_UART_DR             0x01U
#define STM8_UART_BRR1           0x02U
#define STM8_UART_BRR2           0x03U
#define STM8_UART_CR1            0x04U
#define STM8_UART_CR2            0x05U
#define STM8_UART_CR3            0x06U
#define STM8_UART_SR_TXE         0x80U
#define STM8_UART_SR_TC          0x40U
#define STM8_UART_SR_RXNE        0x20U
#define STM8_UART_SR_OR          0x08U
#define STM8_UART_SR_ERRORS      0x0fU
#define STM8_UART_CR1_PIEN       0x01U
#define STM8_UART_CR1_PS         0x02U
#define STM8_UART_CR1_PCEN       0x04U
#define STM8_UART_CR1_M          0x10U
#define STM8_UART_CR2_TIEN       0x80U
#define STM8_UART_CR2_TCIEN      0x40U
#define STM8_UART_CR2_RIEN       0x20U
#define STM8_UART_CR2_ILIEN      0x10U
#define STM8_UART_CR2_TEN        0x08U
#define STM8_UART_CR2_REN        0x04U
#define STM8_UART_DIV_MIN        16U
#define STM8_UART_CR3_STOP_SHIFT 4U

struct uart_stm8_config {
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
	const struct pinctrl_dev_config *pcfg;
	uint32_t baudrate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config)(const struct device *dev);
#endif
};

struct uart_stm8_data {
	uint8_t interrupt_control1;
	uint8_t interrupt_control2;
	bool tx_vector_enabled;
	bool rx_vector_enabled;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	struct clock_control_stm8_client clock_client;
#endif
	uint8_t errors;
	uint8_t rx_mask;
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	struct uart_config configuration;
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *user_data;
	uint8_t rx_byte;
	bool rx_pending;
	bool rx_enabled;
	bool err_enabled;
#endif
};

static void uart_stm8_control1_write(const struct device *dev, uint8_t value)
{
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	data->interrupt_control1 = value & STM8_UART_CR1_PIEN;
	sys_write8(data->rx_vector_enabled ? value : value & ~STM8_UART_CR1_PIEN,
		   cfg->base + STM8_UART_CR1);
	irq_unlock(key);
}

static void uart_stm8_control2_write(const struct device *dev, uint8_t value)
{
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	data->interrupt_control2 = value & (STM8_UART_CR2_TIEN | STM8_UART_CR2_TCIEN |
					    STM8_UART_CR2_RIEN | STM8_UART_CR2_ILIEN);
	if (!data->tx_vector_enabled) {
		value &= ~(STM8_UART_CR2_TIEN | STM8_UART_CR2_TCIEN);
	}
	if (!data->rx_vector_enabled) {
		value &= ~(STM8_UART_CR2_RIEN | STM8_UART_CR2_ILIEN);
	}
	sys_write8(value, cfg->base + STM8_UART_CR2);
	irq_unlock(key);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_stm8_irq_restore(const struct device *dev)
{
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	uint8_t control1 = sys_read8(cfg->base + STM8_UART_CR1) & ~STM8_UART_CR1_PIEN;
	uint8_t control2 =
		sys_read8(cfg->base + STM8_UART_CR2) & ~(STM8_UART_CR2_TIEN | STM8_UART_CR2_TCIEN |
							 STM8_UART_CR2_RIEN | STM8_UART_CR2_ILIEN);

	if (data->tx_vector_enabled) {
		control2 |= data->interrupt_control2 & (STM8_UART_CR2_TIEN | STM8_UART_CR2_TCIEN);
	}
	if (data->rx_vector_enabled) {
		control1 |= data->interrupt_control1;
		control2 |= data->interrupt_control2 & (STM8_UART_CR2_RIEN | STM8_UART_CR2_ILIEN);
	}
	sys_write8(control1, cfg->base + STM8_UART_CR1);
	sys_write8(control2, cfg->base + STM8_UART_CR2);
}

static void uart_stm8_tx_irq_set_enabled(const struct device *dev, bool enabled)
{
	struct uart_stm8_data *data = dev->data;

	data->tx_vector_enabled = enabled;
	uart_stm8_irq_restore(dev);
}

static void uart_stm8_rx_irq_set_enabled(const struct device *dev, bool enabled)
{
	struct uart_stm8_data *data = dev->data;

	data->rx_vector_enabled = enabled;
	uart_stm8_irq_restore(dev);
}
#endif

static int uart_stm8_poll_in(const struct device *dev, unsigned char *byte)
{
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	int err = -1;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->rx_pending) {
		*byte = data->rx_byte;
		data->rx_pending = false;
		err = 0;
	}
#endif
	if (err != 0) {
		uint8_t status = sys_read8(cfg->base + STM8_UART_SR);

		if ((status & STM8_UART_SR_RXNE) != 0U) {
			data->errors |= status & STM8_UART_SR_ERRORS;
			*byte = sys_read8(cfg->base + STM8_UART_DR) & data->rx_mask;
			err = 0;
		}
	}
	irq_unlock(key);
	return err;
}

static void uart_stm8_poll_out(const struct device *dev, unsigned char byte)
{
	const struct uart_stm8_config *cfg = dev->config;

	while ((sys_read8(cfg->base + STM8_UART_SR) & STM8_UART_SR_TXE) == 0U) {
	}
	sys_write8(byte, cfg->base + STM8_UART_DR);
}

static int uart_stm8_err_check(const struct device *dev)
{
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	uint8_t status = sys_read8(cfg->base + STM8_UART_SR) | data->errors;
	int errors;

	data->errors = 0U;
	errors = ((status >> 3) & 1U) * UART_ERROR_OVERRUN |
		 ((status >> 2) & 1U) * UART_ERROR_NOISE |
		 ((status >> 1) & 1U) * UART_ERROR_FRAMING | (status & 1U) * UART_ERROR_PARITY;
	if ((status & STM8_UART_SR_ERRORS) != 0U &&
	    (sys_read8(cfg->base + STM8_UART_SR) & STM8_UART_SR_RXNE) == 0U) {
		/* Clear flags by the SR/DR sequence without discarding a pending byte. */
		(void)sys_read8(cfg->base + STM8_UART_DR);
	}
	irq_unlock(key);
	return errors;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_stm8_irq_tx_mask(const struct device *dev, bool enable)
{
	const struct uart_stm8_config *cfg = dev->config;
	unsigned int key = irq_lock();
	uint8_t control = (sys_read8(cfg->base + STM8_UART_CR2) |
			   ((struct uart_stm8_data *)dev->data)->interrupt_control2);

	uart_stm8_control2_write(dev, enable ? control | STM8_UART_CR2_TIEN
					     : control & ~STM8_UART_CR2_TIEN);
	irq_unlock(key);
}

static int uart_stm8_fifo_fill(const struct device *dev, const uint8_t *buffer, int size)
{
	const struct uart_stm8_config *cfg = dev->config;
	int count = 0;

	while (count < size && (sys_read8(cfg->base + STM8_UART_SR) & STM8_UART_SR_TXE) != 0U) {
		sys_write8(buffer[count++], cfg->base + STM8_UART_DR);
	}
	return count;
}

static int uart_stm8_fifo_read(const struct device *dev, uint8_t *buffer, int size)
{
	int count = 0;

	while (count < size && uart_stm8_poll_in(dev, &buffer[count]) == 0) {
		count++;
	}
	return count;
}

static void uart_stm8_irq_tx_enable(const struct device *dev)
{
	uart_stm8_irq_tx_mask(dev, true);
}

static void uart_stm8_irq_tx_disable(const struct device *dev)
{
	uart_stm8_irq_tx_mask(dev, false);
}

static void uart_stm8_receive_interrupts(const struct device *dev, bool receive, bool enable)
{
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	unsigned int key = irq_lock();
	uint8_t control = (sys_read8(cfg->base + STM8_UART_CR2) |
			   ((struct uart_stm8_data *)dev->data)->interrupt_control2) &
			  ~STM8_UART_CR2_RIEN;
	uint8_t control1 = (sys_read8(cfg->base + STM8_UART_CR1) |
			    ((struct uart_stm8_data *)dev->data)->interrupt_control1) &
			   ~STM8_UART_CR1_PIEN;

	if (receive) {
		data->rx_enabled = enable;
	} else {
		data->err_enabled = enable;
	}
	/* Overrun and RXNE share RIEN; parity uses CR1.PIEN. */
	if (data->rx_enabled || data->err_enabled) {
		control |= STM8_UART_CR2_RIEN;
	}
	if (data->err_enabled) {
		control1 |= STM8_UART_CR1_PIEN;
	}
	uart_stm8_control1_write(dev, control1);
	uart_stm8_control2_write(dev, control);
	irq_unlock(key);
}

static void uart_stm8_irq_rx_enable(const struct device *dev)
{
	uart_stm8_receive_interrupts(dev, true, true);
}

static void uart_stm8_irq_rx_disable(const struct device *dev)
{
	uart_stm8_receive_interrupts(dev, true, false);
}

static void uart_stm8_irq_err_enable(const struct device *dev)
{
	uart_stm8_receive_interrupts(dev, false, true);
}

static void uart_stm8_irq_err_disable(const struct device *dev)
{
	uart_stm8_receive_interrupts(dev, false, false);
}

static int uart_stm8_irq_tx_ready(const struct device *dev)
{
	const struct uart_stm8_config *cfg = dev->config;

	return ((sys_read8(cfg->base + STM8_UART_CR2) |
		 ((struct uart_stm8_data *)dev->data)->interrupt_control2) &
		STM8_UART_CR2_TIEN) != 0U &&
	       (sys_read8(cfg->base + STM8_UART_SR) & STM8_UART_SR_TXE) != 0U;
}

static int uart_stm8_irq_tx_complete(const struct device *dev)
{
	const struct uart_stm8_config *cfg = dev->config;

	return (sys_read8(cfg->base + STM8_UART_SR) & STM8_UART_SR_TC) != 0U;
}

static int uart_stm8_irq_rx_ready(const struct device *dev)
{
	const struct uart_stm8_data *data = dev->data;

	return data->rx_enabled && data->rx_pending;
}

static void uart_stm8_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static int uart_stm8_irq_is_pending(const struct device *dev)
{
	const struct uart_stm8_data *data = dev->data;

	return uart_stm8_irq_tx_ready(dev) || uart_stm8_irq_rx_ready(dev) ||
	       (data->err_enabled && data->errors != 0U);
}

static void uart_stm8_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t callback, void *user_data)
{
	struct uart_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	data->callback = callback;
	data->user_data = user_data;
	irq_unlock(key);
}

#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_stm8_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	uint8_t status = sys_read8(cfg->base + STM8_UART_SR);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	data->errors |= status & STM8_UART_SR_ERRORS;
	if ((status & STM8_UART_SR_RXNE) != 0U &&
	    ((sys_read8(cfg->base + STM8_UART_CR2) |
	      ((struct uart_stm8_data *)dev->data)->interrupt_control2) &
	     STM8_UART_CR2_RIEN) != 0U) {
		uint8_t byte = sys_read8(cfg->base + STM8_UART_DR) & data->rx_mask;

		/* Consume RXNE even for error-only IRQs, so polling can make progress. */
		if (data->rx_pending) {
			data->errors |= STM8_UART_SR_OR;
		} else {
			data->rx_byte = byte;
			data->rx_pending = true;
		}
	}
	if (data->callback != NULL) {
		if (uart_stm8_irq_is_pending(dev)) {
			data->callback(dev, data->user_data);
		}
	} else {
		uart_stm8_control1_write(
			dev, (sys_read8(cfg->base + STM8_UART_CR1) |
			      ((struct uart_stm8_data *)dev->data)->interrupt_control1) &
				     ~STM8_UART_CR1_PIEN);
		uart_stm8_control2_write(
			dev, (sys_read8(cfg->base + STM8_UART_CR2) |
			      ((struct uart_stm8_data *)dev->data)->interrupt_control2) &
				     ~(STM8_UART_CR2_TIEN | STM8_UART_CR2_RIEN));
		data->rx_enabled = false;
		data->err_enabled = false;
	}
#else
	ARG_UNUSED(data);
#endif
}
#endif

static int uart_stm8_apply_config(const struct device *dev, const struct uart_config *config)
{
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	uint32_t frequency;
	uint32_t divisor;
	uint8_t control1 = 0U;
	uint8_t control3;
	unsigned int key;
	int err;

	if (config->flow_ctrl != UART_CFG_FLOW_CTRL_NONE ||
	    (config->parity != UART_CFG_PARITY_NONE && config->parity != UART_CFG_PARITY_EVEN &&
	     config->parity != UART_CFG_PARITY_ODD)) {
		return -ENOTSUP;
	}
	if (config->data_bits == UART_CFG_DATA_BITS_8) {
		if (config->parity != UART_CFG_PARITY_NONE) {
			control1 |= STM8_UART_CR1_M;
		}
	} else if (config->data_bits != UART_CFG_DATA_BITS_7 ||
		   config->parity == UART_CFG_PARITY_NONE) {
		return -ENOTSUP;
	}
	if (config->parity != UART_CFG_PARITY_NONE) {
		control1 |= STM8_UART_CR1_PCEN;
		if (config->parity == UART_CFG_PARITY_ODD) {
			control1 |= STM8_UART_CR1_PS;
		}
	}
	switch (config->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		control3 = 0U;
		break;
	case UART_CFG_STOP_BITS_2:
		control3 = 2U << STM8_UART_CR3_STOP_SHIFT;
		break;
	case UART_CFG_STOP_BITS_1_5:
		control3 = 3U << STM8_UART_CR3_STOP_SHIFT;
		break;
	default:
		return -ENOTSUP;
	}
	/* RM0016: nine-bit hardware words require one stop bit. */
	if ((control1 & STM8_UART_CR1_M) != 0U && control3 != 0U) {
		return -ENOTSUP;
	}
	if (config->baudrate == 0U) {
		return -EINVAL;
	}
	err = clock_control_get_rate(cfg->clock, cfg->clock_id, &frequency);
	if (err != 0) {
		return err;
	}
	divisor = (frequency + config->baudrate / 2U) / config->baudrate;
	if (divisor < STM8_UART_DIV_MIN || divisor > UINT16_MAX) {
		return -EINVAL;
	}
	key = irq_lock();
	uint8_t control2 = (sys_read8(cfg->base + STM8_UART_CR2) |
			    ((struct uart_stm8_data *)dev->data)->interrupt_control2);
	uint8_t status = sys_read8(cfg->base + STM8_UART_SR);

	if (((control2 & STM8_UART_CR2_TEN) != 0U && (status & STM8_UART_SR_TC) == 0U) ||
	    (status & STM8_UART_SR_RXNE) != 0U) {
		irq_unlock(key);
		return -EBUSY;
	}
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->rx_pending || (control2 & STM8_UART_CR2_TIEN) != 0U) {
		irq_unlock(key);
		return -EBUSY;
	}
#endif
	control1 |= (sys_read8(cfg->base + STM8_UART_CR1) |
		     ((struct uart_stm8_data *)dev->data)->interrupt_control1) &
		    STM8_UART_CR1_PIEN;
	uart_stm8_control2_write(dev, 0U);
	uart_stm8_control1_write(dev, control1);
	sys_write8(control3, cfg->base + STM8_UART_CR3);
	/* BRR2 must precede BRR1, which latches the complete divisor. */
	sys_write8(((divisor >> 8) & 0xf0U) | (divisor & 0x0fU), cfg->base + STM8_UART_BRR2);
	sys_write8(divisor >> 4, cfg->base + STM8_UART_BRR1);
	uart_stm8_control2_write(dev, control2 | STM8_UART_CR2_TEN | STM8_UART_CR2_REN);
	data->rx_mask = config->data_bits == UART_CFG_DATA_BITS_7 ? 0x7fU : 0xffU;
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	data->configuration = *config;
#else
	ARG_UNUSED(data);
#endif
	irq_unlock(key);
	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_stm8_config_get(const struct device *dev, struct uart_config *config)
{
	struct uart_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	*config = data->configuration;
	irq_unlock(key);
	return 0;
}
#endif

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static int uart_stm8_clock_prepare(const struct device *dev, uint32_t rate)
{
	const struct uart_stm8_config *cfg = dev->config;
	struct uart_stm8_data *data = dev->data;
	uint32_t baudrate = cfg->baudrate;

	if (clock_control_get_status(cfg->clock, cfg->clock_id) == CLOCK_CONTROL_STATUS_OFF) {
		return 0;
	}
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	baudrate = data->configuration.baudrate;
#endif
	uint32_t divisor = (rate + baudrate / 2U) / baudrate;

	if (divisor < STM8_UART_DIV_MIN || divisor > UINT16_MAX) {
		return -EINVAL;
	}
	uint8_t status = sys_read8(cfg->base + STM8_UART_SR);

	if ((status & STM8_UART_SR_TC) == 0U || (status & STM8_UART_SR_RXNE) != 0U) {
		return -EBUSY;
	}
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->rx_pending || ((sys_read8(cfg->base + STM8_UART_CR2) |
				  ((struct uart_stm8_data *)dev->data)->interrupt_control2) &
				 STM8_UART_CR2_TIEN) != 0U) {
		return -EBUSY;
	}
#endif
	ARG_UNUSED(data);
	return 0;
}

static void uart_stm8_clock_changed(const struct device *dev, uint32_t rate)
{
	const struct uart_stm8_config *cfg = dev->config;
	uint32_t baudrate = cfg->baudrate;

	if (clock_control_get_status(cfg->clock, cfg->clock_id) == CLOCK_CONTROL_STATUS_OFF) {
		return;
	}
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	const struct uart_stm8_data *data = dev->data;

	baudrate = data->configuration.baudrate;
#endif
	uint32_t divisor = (rate + baudrate / 2U) / baudrate;
	uint8_t control = (sys_read8(cfg->base + STM8_UART_CR2) |
			   ((struct uart_stm8_data *)dev->data)->interrupt_control2);

	uart_stm8_control2_write(dev, 0U);
	sys_write8(((divisor >> 8) & 0xf0U) | (divisor & 0x0fU), cfg->base + STM8_UART_BRR2);
	sys_write8(divisor >> 4, cfg->base + STM8_UART_BRR1);
	uart_stm8_control2_write(dev, control);
}
#endif

static int uart_stm8_init(const struct device *dev)
{
	const struct uart_stm8_config *cfg = dev->config;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	struct uart_stm8_data *data = dev->data;
#endif
	const struct uart_config config = {
		.baudrate = cfg->baudrate,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
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
	uart_stm8_control2_write(dev, 0U);
	err = uart_stm8_apply_config(dev, &config);
	if (err != 0) {
		return err;
	}
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config(dev);
#endif
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	data->clock_client.clock_id = STM8_CLOCK_MASTER;
	data->clock_client.dev = dev;
	data->clock_client.prepare = uart_stm8_clock_prepare;
	data->clock_client.changed = uart_stm8_clock_changed;
	return clock_control_stm8_register_client(&data->clock_client);
#else
	return 0;
#endif
}

static DEVICE_API(uart, uart_stm8_driver_api) = {
	.poll_in = uart_stm8_poll_in,
	.poll_out = uart_stm8_poll_out,
	.err_check = uart_stm8_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_stm8_apply_config,
	.config_get = uart_stm8_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_stm8_fifo_fill,
	.fifo_read = uart_stm8_fifo_read,
	.irq_tx_enable = uart_stm8_irq_tx_enable,
	.irq_tx_disable = uart_stm8_irq_tx_disable,
	.irq_tx_ready = uart_stm8_irq_tx_ready,
	.irq_tx_complete = uart_stm8_irq_tx_complete,
	.irq_rx_enable = uart_stm8_irq_rx_enable,
	.irq_rx_disable = uart_stm8_irq_rx_disable,
	.irq_err_enable = uart_stm8_irq_err_enable,
	.irq_err_disable = uart_stm8_irq_err_disable,
	.irq_is_pending = uart_stm8_irq_is_pending,
	.irq_update = uart_stm8_irq_update,
	.irq_callback_set = uart_stm8_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_STM8_IRQ_DEFINE(n)                                                                    \
	static void uart_stm8_irq_config_##n(const struct device *dev)                             \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		intc_stm8_irq_register(DT_INST_IRQ_BY_NAME(n, tx, irq), DEVICE_DT_INST_GET(n),     \
				       uart_stm8_tx_irq_set_enabled);                              \
		intc_stm8_irq_register(DT_INST_IRQ_BY_NAME(n, rx, irq), DEVICE_DT_INST_GET(n),     \
				       uart_stm8_rx_irq_set_enabled);                              \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, tx, irq), DT_INST_IRQ_BY_NAME(n, tx, priority), \
			    uart_stm8_isr, DEVICE_DT_INST_GET(n), 0);                              \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, rx, irq), DT_INST_IRQ_BY_NAME(n, rx, priority), \
			    uart_stm8_isr, DEVICE_DT_INST_GET(n), 0);                              \
		irq_enable(DT_INST_IRQ_BY_NAME(n, tx, irq));                                       \
		irq_enable(DT_INST_IRQ_BY_NAME(n, rx, irq));                                       \
	}
#define UART_STM8_IRQ_CONFIG(n) .irq_config = uart_stm8_irq_config_##n,
#else
#define UART_STM8_IRQ_DEFINE(n)
#define UART_STM8_IRQ_CONFIG(n)
#endif

#define UART_STM8_INIT(n)                                                                          \
	UART_STM8_IRQ_DEFINE(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct uart_stm8_data uart_stm8_data_##n;                                           \
	static const struct uart_stm8_config uart_stm8_config_##n = {                              \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.baudrate = DT_INST_PROP(n, current_speed),                                        \
		UART_STM8_IRQ_CONFIG(n)};                                                          \
	DEVICE_DT_INST_DEFINE(n, uart_stm8_init, NULL, &uart_stm8_data_##n, &uart_stm8_config_##n, \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, &uart_stm8_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_STM8_INIT)
