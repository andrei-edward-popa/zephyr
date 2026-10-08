/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_i2c

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_stm8.h>
#include <zephyr/drivers/clock_control/clock_control_stm8.h>

#define STM8_I2C_CR1           0x00U
#define STM8_I2C_CR2           0x01U
#define STM8_I2C_FREQR         0x02U
#define STM8_I2C_OARL          0x03U
#define STM8_I2C_OARH          0x04U
#define STM8_I2C_DR            0x06U
#define STM8_I2C_SR1           0x07U
#define STM8_I2C_SR2           0x08U
#define STM8_I2C_SR3           0x09U
#define STM8_I2C_ITR           0x0aU
#define STM8_I2C_CCRL          0x0bU
#define STM8_I2C_CCRH          0x0cU
#define STM8_I2C_TRISER        0x0dU
#define STM8_I2C_CR1_PE        0x01U
#define STM8_I2C_CR2_SWRST     0x80U
#define STM8_I2C_CR2_POS       0x08U
#define STM8_I2C_CR2_ACK       0x04U
#define STM8_I2C_CR2_STOP      0x02U
#define STM8_I2C_CR2_START     0x01U
#define STM8_I2C_OARH_ADDCONF  0x40U
#define STM8_I2C_OARH_ADDMODE  0x80U
#define STM8_I2C_SR1_TXE       0x80U
#define STM8_I2C_SR1_RXNE      0x40U
#define STM8_I2C_SR1_STOPF     0x10U
#define STM8_I2C_SR1_ADD10     0x08U
#define STM8_I2C_SR1_BTF       0x04U
#define STM8_I2C_SR1_ADDR      0x02U
#define STM8_I2C_SR1_SB        0x01U
#define STM8_I2C_SR2_ARLO      0x02U
#define STM8_I2C_SR2_AF        0x04U
#define STM8_I2C_SR2_ERRORS    0x0fU
#define STM8_I2C_SR3_BUSY      0x02U
#define STM8_I2C_SR3_TRA       0x04U
#define STM8_I2C_ITR_BUFFER    0x04U
#define STM8_I2C_ITR_EVENT     0x02U
#define STM8_I2C_ITR_ERROR     0x01U
#define STM8_I2C_CCRH_FS       0x80U
#define STM8_I2C_CCR_MAX       0x0fffU
#define STM8_I2C_STANDARD_RATE 88000U
#define STM8_I2C_FAST_RATE     400000U
#define STM8_I2C_MHZ           1000000U

#include "i2c-priv.h"

#define STM8_I2C_TRANSFER_TIMEOUT_MS 100U
#define STM8_I2C_ADDRESS_MAX         0x7fU
#define STM8_I2C_ADDRESS_10_MAX      0x3ffU
#define STM8_I2C_ADDRESS_10_HEADER   0xf0U
#define STM8_I2C_RECOVERY_DELAY_US   5U
#define STM8_I2C_RECOVERY_RETRIES    200U
#define STM8_I2C_RECOVERY_PULSES     9U
#define STM8_I2C_CCR_STANDARD_MIN    4U
#define STM8_I2C_FAST_RISE_NS        300U

struct i2c_stm8_config {
	uintptr_t base;
	const struct device *clock;
	clock_control_subsys_t clock_id;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_GPIO
	struct gpio_dt_spec scl;
	struct gpio_dt_spec sda;
#endif
	uint32_t bitrate;
	void (*irq_config)(const struct device *dev);
};

struct i2c_stm8_data {
	uint8_t interrupt_mask;
	bool vector_enabled;
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	struct clock_control_stm8_client clock_client;
#endif
	struct k_sem lock;
	struct k_sem done;
	struct i2c_msg *msgs;
	uint32_t remaining;
	uint32_t offset;
	uint32_t configuration;
	uint8_t cursor;
	uint8_t end;
	uint16_t address;
	uint8_t ending;
	int status;
	bool read;
	bool ten_bit;
	bool read_header;
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target;
	uint32_t controller_configuration;
	bool target_active;
	bool target_read;
	bool target_rejected;
#endif
};

static void i2c_stm8_interrupt_write(const struct device *dev, uint8_t value)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	unsigned int key = irq_lock();

	data->interrupt_mask = value;
	sys_write8(data->vector_enabled ? value : 0U, cfg->base + STM8_I2C_ITR);
	irq_unlock(key);
}

static void i2c_stm8_irq_set_enabled(const struct device *dev, bool enabled)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;

	data->vector_enabled = enabled;
	sys_write8(enabled ? data->interrupt_mask : 0U, cfg->base + STM8_I2C_ITR);
}

static int i2c_stm8_configure_locked(const struct device *dev, uint32_t config)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	uint32_t rate;
	uint32_t speed;
	uint32_t ccr;
	uint8_t fast = 0U;
	uint8_t rise;
	int err;

	if ((config & I2C_MODE_CONTROLLER) == 0U) {
		return -ENOTSUP;
	}
	if (I2C_SPEED_GET(config) == I2C_SPEED_STANDARD) {
		/* ES036: repeated START at 100 kHz violates tSU;STA under stretching. */
		speed = STM8_I2C_STANDARD_RATE;
	} else if (I2C_SPEED_GET(config) == I2C_SPEED_FAST) {
		speed = STM8_I2C_FAST_RATE;
		fast = STM8_I2C_CCRH_FS;
	} else {
		return -ENOTSUP;
	}
	err = clock_control_get_rate(cfg->clock, cfg->clock_id, &rate);
	if (err != 0) {
		return err;
	}
	uint8_t mhz = rate / STM8_I2C_MHZ;

	if (mhz == 0U || mhz > 16U || (fast != 0U && mhz < 4U)) {
		return -EINVAL;
	}
	ccr = DIV_ROUND_UP(rate, speed * (fast != 0U ? 3U : 2U));
	ccr = MAX(ccr, fast != 0U ? 1U : STM8_I2C_CCR_STANDARD_MIN);
	if (ccr > STM8_I2C_CCR_MAX) {
		return -EINVAL;
	}
	rise = fast != 0U ? ((uint32_t)mhz * STM8_I2C_FAST_RISE_NS / 1000U) + 1U : mhz + 1U;
	sys_write8(0U, cfg->base + STM8_I2C_CR1);
	i2c_stm8_interrupt_write(dev, 0U);
	sys_write8(0U, cfg->base + STM8_I2C_CR2);
	sys_write8(mhz, cfg->base + STM8_I2C_FREQR);
	sys_write8(STM8_I2C_OARH_ADDCONF, cfg->base + STM8_I2C_OARH);
	sys_write8(ccr, cfg->base + STM8_I2C_CCRL);
	sys_write8((ccr >> 8) | fast, cfg->base + STM8_I2C_CCRH);
	sys_write8(rise, cfg->base + STM8_I2C_TRISER);
	sys_write8(STM8_I2C_CR1_PE, cfg->base + STM8_I2C_CR1);
	data->configuration = config;
	return 0;
}

static int i2c_stm8_configure(const struct device *dev, uint32_t config)
{
	struct i2c_stm8_data *data = dev->data;
	int err = k_sem_take(&data->lock, K_FOREVER);

	if (err == 0) {
#ifdef CONFIG_I2C_TARGET
		if (data->target != NULL) {
			k_sem_give(&data->lock);
			return -EBUSY;
		}
#endif
		err = i2c_stm8_configure_locked(dev, config);
		k_sem_give(&data->lock);
	}
	return err;
}

static int i2c_stm8_get_config(const struct device *dev, uint32_t *config)
{
	struct i2c_stm8_data *data = dev->data;

	*config = data->configuration;
	return 0;
}

static void i2c_stm8_advance(struct i2c_stm8_data *data)
{
	while (data->cursor < data->end && data->offset == data->msgs[data->cursor].len) {
		data->cursor++;
		data->offset = 0U;
	}
}

static void i2c_stm8_receive_byte(const struct device *dev)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;

	i2c_stm8_advance(data);
	data->msgs[data->cursor].buf[data->offset++] = sys_read8(cfg->base + STM8_I2C_DR);
	data->remaining--;
}

static void i2c_stm8_transmit_byte(const struct device *dev)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;

	i2c_stm8_advance(data);
	sys_write8(data->msgs[data->cursor].buf[data->offset++], cfg->base + STM8_I2C_DR);
	data->remaining--;
}

static void i2c_stm8_end_condition(const struct device *dev)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;

	sys_write8(sys_read8(cfg->base + STM8_I2C_CR2) | data->ending, cfg->base + STM8_I2C_CR2);
}

static void i2c_stm8_complete(const struct device *dev, int status)
{
	struct i2c_stm8_data *data = dev->data;

	i2c_stm8_interrupt_write(dev, 0U);
	data->status = status;
	k_sem_give(&data->done);
}

#ifdef CONFIG_I2C_TARGET
static void i2c_stm8_target_stop(const struct device *dev)
{
	struct i2c_stm8_data *data = dev->data;

	if (data->target_active) {
		data->target_active = false;
		if (data->target->callbacks->stop != NULL) {
			(void)data->target->callbacks->stop(data->target);
		}
	}
}

static void i2c_stm8_target_ack(const struct device *dev, bool ack)
{
	const struct i2c_stm8_config *cfg = dev->config;
	uint8_t control = sys_read8(cfg->base + STM8_I2C_CR2);

	sys_write8(ack ? control | STM8_I2C_CR2_ACK : control & ~STM8_I2C_CR2_ACK,
		   cfg->base + STM8_I2C_CR2);
}

static void i2c_stm8_target_receive(const struct device *dev)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	uint8_t byte = sys_read8(cfg->base + STM8_I2C_DR);

	if (data->target_active && !data->target_rejected) {
		data->target_rejected =
			data->target->callbacks->write_received(data->target, byte) != 0;
		if (data->target_rejected) {
			i2c_stm8_target_ack(dev, false);
		}
	}
}

static void i2c_stm8_target_isr(const struct device *dev, uint8_t status, uint8_t errors)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	const struct i2c_target_callbacks *callbacks = data->target->callbacks;

	if (errors != 0U) {
		sys_write8(0U, cfg->base + STM8_I2C_SR2);
		/* A controller's final NACK is normal termination of a target read. */
		if ((errors & ~STM8_I2C_SR2_AF) != 0U && callbacks->error != NULL) {
			callbacks->error(data->target, (errors & STM8_I2C_SR2_ARLO) != 0U
							       ? I2C_ERROR_ARBITRATION
							       : I2C_ERROR_GENERIC);
		}
		i2c_stm8_target_stop(dev);
		i2c_stm8_target_ack(dev, true);
		i2c_stm8_interrupt_write(dev, STM8_I2C_ITR_EVENT | STM8_I2C_ITR_ERROR);
	}
	/* Drain the previous write before handling STOP or a repeated START. */
	if (!data->target_read && (status & STM8_I2C_SR1_RXNE) != 0U) {
		i2c_stm8_target_receive(dev);
		if ((status & STM8_I2C_SR1_BTF) != 0U) {
			i2c_stm8_target_receive(dev);
		}
	}
	if ((status & STM8_I2C_SR1_STOPF) != 0U) {
		(void)sys_read8(cfg->base + STM8_I2C_SR1);
		i2c_stm8_target_ack(dev, true);
		i2c_stm8_target_stop(dev);
		i2c_stm8_interrupt_write(dev, STM8_I2C_ITR_EVENT | STM8_I2C_ITR_ERROR);
	}
	if ((status & STM8_I2C_SR1_ADDR) != 0U) {
		(void)sys_read8(cfg->base + STM8_I2C_SR1);
		uint8_t direction = sys_read8(cfg->base + STM8_I2C_SR3);
		uint8_t byte = 0xffU;

		data->target_active = true;
		data->target_read = (direction & STM8_I2C_SR3_TRA) != 0U;
		if (data->target_read) {
			data->target_rejected = callbacks->read_requested(data->target, &byte) != 0;
			sys_write8(data->target_rejected ? 0xffU : byte, cfg->base + STM8_I2C_DR);
		} else {
			data->target_rejected = callbacks->write_requested(data->target) != 0;
		}
		i2c_stm8_target_ack(dev, !data->target_rejected);
		i2c_stm8_interrupt_write(dev,
					 STM8_I2C_ITR_EVENT | STM8_I2C_ITR_ERROR |
						 (data->target_read ? 0U : STM8_I2C_ITR_BUFFER));
		return;
	}
	/* BTF stretches SCL until DR is filled; avoid speculative TXE prefetch. */
	if (data->target_active && data->target_read && (status & STM8_I2C_SR1_BTF) != 0U) {
		uint8_t byte = 0xffU;

		if (!data->target_rejected) {
			data->target_rejected = callbacks->read_processed(data->target, &byte) != 0;
		}
		sys_write8(data->target_rejected ? 0xffU : byte, cfg->base + STM8_I2C_DR);
	}
}

static int i2c_stm8_target_register(const struct device *dev, struct i2c_target_config *target)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	int err;

	if (target == NULL || target->callbacks == NULL || target->address == 0U ||
	    target->address > ((target->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U
				       ? 0x3ffU
				       : STM8_I2C_ADDRESS_MAX) ||
	    target->callbacks->read_requested == NULL ||
	    target->callbacks->read_processed == NULL ||
	    target->callbacks->write_requested == NULL ||
	    target->callbacks->write_received == NULL) {
		return -EINVAL;
	}
	if ((target->flags & ~I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U) {
		return -ENOTSUP;
	}
	err = k_sem_take(&data->lock, K_NO_WAIT);
	if (err != 0) {
		return -EBUSY;
	}
	unsigned int key = irq_lock();

	if (data->target != NULL ||
	    (sys_read8(cfg->base + STM8_I2C_SR3) & STM8_I2C_SR3_BUSY) != 0U) {
		err = -EBUSY;
	} else {
		uint8_t high = STM8_I2C_OARH_ADDCONF;
		uint8_t low = target->address << 1;

		if ((target->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U) {
			high |= STM8_I2C_OARH_ADDMODE | ((target->address >> 7) & 0x06U);
			low = target->address;
		}
		sys_write8(0U, cfg->base + STM8_I2C_CR1);
		sys_write8(low, cfg->base + STM8_I2C_OARL);
		sys_write8(high, cfg->base + STM8_I2C_OARH);
		data->target = target;
		data->target_active = false;
		data->target_read = false;
		data->controller_configuration = data->configuration;
		data->configuration &= ~I2C_MODE_CONTROLLER;
		sys_write8(STM8_I2C_CR1_PE, cfg->base + STM8_I2C_CR1);
		sys_write8(STM8_I2C_CR2_ACK, cfg->base + STM8_I2C_CR2);
		i2c_stm8_interrupt_write(dev, STM8_I2C_ITR_EVENT | STM8_I2C_ITR_ERROR);
	}
	irq_unlock(key);
	k_sem_give(&data->lock);
	return err;
}

static int i2c_stm8_target_unregister(const struct device *dev, struct i2c_target_config *target)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	int err = k_sem_take(&data->lock, K_NO_WAIT);

	if (err != 0) {
		return -EBUSY;
	}
	unsigned int key = irq_lock();

	if (target == NULL || data->target != target) {
		err = -EINVAL;
	} else if (data->target_active ||
		   (sys_read8(cfg->base + STM8_I2C_SR3) & STM8_I2C_SR3_BUSY) != 0U) {
		err = -EBUSY;
	} else {
		i2c_stm8_interrupt_write(dev, 0U);
		data->target = NULL;
		err = i2c_stm8_configure_locked(dev, data->controller_configuration);
	}
	irq_unlock(key);
	k_sem_give(&data->lock);
	return err;
}
#endif

static void i2c_stm8_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	uint8_t errors = sys_read8(cfg->base + STM8_I2C_SR2) & STM8_I2C_SR2_ERRORS;
	uint8_t status = sys_read8(cfg->base + STM8_I2C_SR1);

#ifdef CONFIG_I2C_TARGET
	if (data->target != NULL) {
		i2c_stm8_target_isr(dev, status, errors);
		return;
	}
#endif
	if (errors != 0U) {
		/* Arbitration loss must release the bus without generating STOP. */
		if ((errors & STM8_I2C_SR2_ARLO) == 0U) {
			sys_write8(sys_read8(cfg->base + STM8_I2C_CR2) | STM8_I2C_CR2_STOP,
				   cfg->base + STM8_I2C_CR2);
		}
		sys_write8(0U, cfg->base + STM8_I2C_SR2);
		i2c_stm8_complete(dev, (errors & STM8_I2C_SR2_ARLO) != 0U ? -EAGAIN : -EIO);
		return;
	}
	if ((status & STM8_I2C_SR1_SB) != 0U) {
		uint8_t address = data->ten_bit
			? STM8_I2C_ADDRESS_10_HEADER | ((data->address >> 7) & 0x06U) |
				(data->read_header ? 1U : 0U)
			: (data->address << 1) | (data->read ? 1U : 0U);

		sys_write8(address, cfg->base + STM8_I2C_DR);
		return;
	}
	if ((status & STM8_I2C_SR1_ADD10) != 0U) {
		sys_write8(data->address, cfg->base + STM8_I2C_DR);
		return;
	}
	if ((status & STM8_I2C_SR1_ADDR) != 0U) {
		unsigned int key = irq_lock();

		if (data->ten_bit && data->read && !data->read_header) {
			/* RM0016 EV6: release ADDR before the repeated START/read header. */
			(void)sys_read8(cfg->base + STM8_I2C_SR3);
			data->read_header = true;
			sys_write8(sys_read8(cfg->base + STM8_I2C_CR2) | STM8_I2C_CR2_START,
				   cfg->base + STM8_I2C_CR2);
			irq_unlock(key);
			return;
		}
		if (data->read && data->remaining == 1U) {
			sys_write8(sys_read8(cfg->base + STM8_I2C_CR2) & ~STM8_I2C_CR2_ACK,
				   cfg->base + STM8_I2C_CR2);
		}
		(void)sys_read8(cfg->base + STM8_I2C_SR3);
		if (data->read && data->remaining == 2U) {
			sys_write8(sys_read8(cfg->base + STM8_I2C_CR2) & ~STM8_I2C_CR2_ACK,
				   cfg->base + STM8_I2C_CR2);
		} else if (data->read && data->remaining == 1U) {
			i2c_stm8_end_condition(dev);
			i2c_stm8_interrupt_write(dev, STM8_I2C_ITR_EVENT | STM8_I2C_ITR_ERROR |
							      STM8_I2C_ITR_BUFFER);
		} else if (!data->read) {
			if (data->remaining != 0U) {
				i2c_stm8_transmit_byte(dev);
			} else {
				i2c_stm8_end_condition(dev);
				i2c_stm8_complete(dev, 0);
			}
		}
		irq_unlock(key);
		return;
	}
	if (!data->read && (status & STM8_I2C_SR1_BTF) != 0U) {
		if (data->remaining != 0U) {
			i2c_stm8_transmit_byte(dev);
		} else {
			i2c_stm8_end_condition(dev);
			i2c_stm8_complete(dev, 0);
		}
	} else if (data->read && (status & STM8_I2C_SR1_BTF) != 0U) {
		if (data->remaining > 3U) {
			i2c_stm8_receive_byte(dev);
		} else {
			unsigned int key = irq_lock();

			/* RM0016 method 2; ES036 requires the final STOP/DR sequence atomic. */
			if (data->remaining == 3U) {
				sys_write8(sys_read8(cfg->base + STM8_I2C_CR2) & ~STM8_I2C_CR2_ACK,
					   cfg->base + STM8_I2C_CR2);
				i2c_stm8_receive_byte(dev);
				i2c_stm8_end_condition(dev);
				i2c_stm8_receive_byte(dev);
				i2c_stm8_interrupt_write(dev, STM8_I2C_ITR_EVENT |
								      STM8_I2C_ITR_ERROR |
								      STM8_I2C_ITR_BUFFER);
			} else if (data->remaining == 2U) {
				i2c_stm8_end_condition(dev);
				i2c_stm8_receive_byte(dev);
				i2c_stm8_receive_byte(dev);
				i2c_stm8_complete(dev, 0);
			}
			irq_unlock(key);
		}
	} else if (data->read && data->remaining == 1U && (status & STM8_I2C_SR1_RXNE) != 0U) {
		i2c_stm8_receive_byte(dev);
		i2c_stm8_complete(dev, 0);
	}
}

#ifdef CONFIG_GPIO
static int i2c_stm8_release_scl(const struct gpio_dt_spec *scl)
{
	int err = gpio_pin_set_dt(scl, 1);

	if (err != 0) {
		return err;
	}
	for (uint16_t i = 0U; i < STM8_I2C_RECOVERY_RETRIES; i++) {
		int level = gpio_pin_get_dt(scl);

		if (level != 0) {
			return level < 0 ? level : 0;
		}
		k_busy_wait(STM8_I2C_RECOVERY_DELAY_US);
	}
	return -EBUSY;
}

static int i2c_stm8_clear_bus(const struct i2c_stm8_config *cfg)
{
	int err = i2c_stm8_release_scl(&cfg->scl);

	if (err != 0) {
		return err;
	}
	for (uint8_t i = 0U; i < STM8_I2C_RECOVERY_PULSES; i++) {
		int level = gpio_pin_get_dt(&cfg->sda);

		if (level < 0) {
			return level;
		}
		if (level != 0) {
			break;
		}
		err = gpio_pin_set_dt(&cfg->scl, 0);
		if (err != 0) {
			return err;
		}
		k_busy_wait(STM8_I2C_RECOVERY_DELAY_US);
		err = i2c_stm8_release_scl(&cfg->scl);
		if (err != 0) {
			return err;
		}
		k_busy_wait(STM8_I2C_RECOVERY_DELAY_US);
	}
	/* Generate STOP with SDA changing only after SCL has been released. */
	err = gpio_pin_set_dt(&cfg->scl, 0);
	if (err == 0) {
		err = gpio_pin_set_dt(&cfg->sda, 0);
	}
	if (err == 0) {
		k_busy_wait(STM8_I2C_RECOVERY_DELAY_US);
		err = i2c_stm8_release_scl(&cfg->scl);
	}
	if (err == 0) {
		k_busy_wait(STM8_I2C_RECOVERY_DELAY_US);
		err = gpio_pin_set_dt(&cfg->sda, 1);
	}
	if (err == 0) {
		k_busy_wait(STM8_I2C_RECOVERY_DELAY_US);
		int level = gpio_pin_get_dt(&cfg->sda);

		err = level < 0 ? level : level == 0 ? -EBUSY : 0;
	}
	return err;
}
#endif

static int i2c_stm8_recover_bus(const struct device *dev)
{
#ifdef CONFIG_GPIO
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	int err;
	int restore;

	if (cfg->scl.port == NULL || cfg->sda.port == NULL) {
		return -ENOTSUP;
	}
	if (!gpio_is_ready_dt(&cfg->scl) || !gpio_is_ready_dt(&cfg->sda)) {
		return -ENODEV;
	}
	if (((cfg->scl.dt_flags | cfg->sda.dt_flags) & GPIO_ACTIVE_LOW) != 0U) {
		return -EINVAL;
	}
	err = k_sem_take(&data->lock, K_FOREVER);
	if (err != 0) {
		return err;
	}
#ifdef CONFIG_I2C_TARGET
	if (data->target != NULL) {
		k_sem_give(&data->lock);
		return -EBUSY;
	}
#endif
	i2c_stm8_interrupt_write(dev, 0U);
	sys_write8(0U, cfg->base + STM8_I2C_CR1);
	err = gpio_pin_configure_dt(&cfg->scl, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
	if (err == 0) {
		err = gpio_pin_configure_dt(&cfg->sda, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
	}
	if (err == 0) {
		err = i2c_stm8_clear_bus(cfg);
	}
	restore = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	sys_write8(STM8_I2C_CR2_SWRST, cfg->base + STM8_I2C_CR2);
	sys_write8(0U, cfg->base + STM8_I2C_CR2);
	if (restore == 0) {
		restore = i2c_stm8_configure_locked(dev, data->configuration);
	}
	k_sem_give(&data->lock);
	return err != 0 ? err : restore;
#else
	ARG_UNUSED(dev);
	return -ENOTSUP;
#endif
}

static int i2c_stm8_wait_idle(const struct device *dev)
{
	const struct i2c_stm8_config *cfg = dev->config;
	int64_t deadline = k_uptime_get() + STM8_I2C_TRANSFER_TIMEOUT_MS;

	while ((sys_read8(cfg->base + STM8_I2C_SR3) & STM8_I2C_SR3_BUSY) != 0U ||
	       (sys_read8(cfg->base + STM8_I2C_CR2) & STM8_I2C_CR2_STOP) != 0U) {
		if (k_uptime_get() >= deadline) {
			return -EBUSY;
		}
		k_msleep(1);
	}
	return 0;
}

static int i2c_stm8_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t count,
			     uint16_t address)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	int err;

	if (count == 0U) {
		return 0;
	}
	if (msgs == NULL || address > STM8_I2C_ADDRESS_10_MAX) {
		return -EINVAL;
	}
	for (uint8_t i = 0U; i < count; i++) {
		if (msgs[i].len > SIZE_MAX || (msgs[i].len != 0U && msgs[i].buf == NULL) ||
		    ((msgs[i].flags & I2C_MSG_READ) != 0U && msgs[i].len == 0U)) {
			return -EINVAL;
		}
		if (i > 0U && (msgs[i - 1U].flags & I2C_MSG_STOP) == 0U &&
		    (msgs[i].flags & I2C_MSG_RESTART) == 0U &&
		    ((msgs[i].flags ^ msgs[i - 1U].flags) &
		     (I2C_MSG_READ | I2C_MSG_ADDR_10_BITS)) != 0U) {
			return -ENOTSUP;
		}
	}
	err = k_sem_take(&data->lock, K_FOREVER);
	if (err != 0) {
		return err;
	}
#ifdef CONFIG_I2C_TARGET
	if (data->target != NULL) {
		k_sem_give(&data->lock);
		return -EBUSY;
	}
#endif
	for (uint8_t i = 0U; i < count; i++) {
		if (address > STM8_I2C_ADDRESS_MAX &&
		    (msgs[i].flags & I2C_MSG_ADDR_10_BITS) == 0U &&
		    (data->configuration & I2C_ADDR_10_BITS) == 0U) {
			k_sem_give(&data->lock);
			return -EINVAL;
		}
	}
	err = i2c_stm8_wait_idle(dev);
	for (uint8_t i = 0U; i < count && err == 0;) {
		uint8_t end = i;
		uint32_t length = msgs[i].len;

		while (end + 1U < count && (msgs[end].flags & I2C_MSG_STOP) == 0U &&
		       (msgs[end + 1U].flags & I2C_MSG_RESTART) == 0U) {
			end++;
			length += msgs[end].len;
		}
		data->msgs = msgs;
		data->cursor = i;
		data->end = end;
		data->offset = 0U;
		data->remaining = length;
		data->read = (msgs[i].flags & I2C_MSG_READ) != 0U;
		data->address = address;
		data->ten_bit = (msgs[i].flags & I2C_MSG_ADDR_10_BITS) != 0U ||
			       (data->configuration & I2C_ADDR_10_BITS) != 0U;
		data->read_header = false;
		data->ending = end + 1U == count || (msgs[end].flags & I2C_MSG_STOP) != 0U
				       ? STM8_I2C_CR2_STOP
				       : STM8_I2C_CR2_START;
		k_sem_reset(&data->done);
		uint8_t control = sys_read8(cfg->base + STM8_I2C_CR2);

		control &= ~(STM8_I2C_CR2_ACK | STM8_I2C_CR2_POS);
		if (data->read) {
			control |= STM8_I2C_CR2_ACK;
			if (length == 2U) {
				control |= STM8_I2C_CR2_POS;
			}
		}
		if (i == 0U || (msgs[i - 1U].flags & I2C_MSG_STOP) != 0U) {
			control |= STM8_I2C_CR2_START;
		}
		sys_write8(control, cfg->base + STM8_I2C_CR2);
		i2c_stm8_interrupt_write(dev, STM8_I2C_ITR_EVENT | STM8_I2C_ITR_ERROR);
		err = k_sem_take(&data->done, K_MSEC(STM8_I2C_TRANSFER_TIMEOUT_MS));
		if (err == 0) {
			err = data->status;
		} else {
			err = -ETIMEDOUT;
		}
		if (err == 0 && data->ending == STM8_I2C_CR2_STOP) {
			err = i2c_stm8_wait_idle(dev);
		}
		i = end + 1U;
	}
	if (err != 0) {
		unsigned int key = irq_lock();

		i2c_stm8_interrupt_write(dev, 0U);
		sys_write8(STM8_I2C_CR2_SWRST, cfg->base + STM8_I2C_CR2);
		sys_write8(0U, cfg->base + STM8_I2C_CR2);
		irq_unlock(key);
		(void)i2c_stm8_configure_locked(dev, data->configuration);
	}
	k_sem_give(&data->lock);
	return err;
}

#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
static int i2c_stm8_clock_prepare(const struct device *dev, uint32_t rate)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;

	if (clock_control_get_status(cfg->clock, cfg->clock_id) == CLOCK_CONTROL_STATUS_OFF) {
		return 0;
	}
#ifdef CONFIG_I2C_TARGET
	if (data->target != NULL) {
		return -EBUSY;
	}
#endif
	if (k_sem_count_get(&data->lock) == 0U ||
	    (sys_read8(cfg->base + STM8_I2C_SR3) & STM8_I2C_SR3_BUSY) != 0U) {
		return -EBUSY;
	}
	uint32_t minimum =
		I2C_SPEED_GET(data->configuration) == I2C_SPEED_FAST ? 4000000UL : STM8_I2C_MHZ;

	return rate < minimum || rate > 16000000UL ? -EINVAL : 0;
}

static void i2c_stm8_clock_changed(const struct device *dev, uint32_t rate)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;

	ARG_UNUSED(rate);
	if (clock_control_get_status(cfg->clock, cfg->clock_id) != CLOCK_CONTROL_STATUS_OFF) {
		(void)i2c_stm8_configure_locked(dev, data->configuration);
	}
}
#endif

static int i2c_stm8_init(const struct device *dev)
{
	const struct i2c_stm8_config *cfg = dev->config;
	struct i2c_stm8_data *data = dev->data;
	int err;

	k_sem_init(&data->lock, 1U, 1U);
	k_sem_init(&data->done, 0U, 1U);
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
	err = i2c_stm8_configure_locked(dev,
					I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bitrate));
	if (err != 0) {
		return err;
	}
	cfg->irq_config(dev);
#ifdef CONFIG_CLOCK_CONTROL_STM8_RUNTIME
	data->clock_client.clock_id = STM8_CLOCK_MASTER;
	data->clock_client.dev = dev;
	data->clock_client.prepare = i2c_stm8_clock_prepare;
	data->clock_client.changed = i2c_stm8_clock_changed;
	return clock_control_stm8_register_client(&data->clock_client);
#else
	return 0;
#endif
}

static DEVICE_API(i2c, i2c_stm8_driver_api) = {
	.configure = i2c_stm8_configure,
	.get_config = i2c_stm8_get_config,
	.transfer = i2c_stm8_transfer,
	.recover_bus = i2c_stm8_recover_bus,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_stm8_target_register,
	.target_unregister = i2c_stm8_target_unregister,
#endif
};

#define I2C_STM8_INIT(n)                                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_stm8_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		intc_stm8_irq_register(DT_INST_IRQN(n), DEVICE_DT_INST_GET(n),                     \
				       i2c_stm8_irq_set_enabled);                                  \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2c_stm8_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
		ARG_UNUSED(dev);                                                                   \
	}                                                                                          \
	static struct i2c_stm8_data i2c_stm8_data_##n;                                             \
	static const struct i2c_stm8_config i2c_stm8_config_##n = {                                \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		IF_ENABLED(CONFIG_GPIO,                                                          \
			   (.scl = GPIO_DT_SPEC_INST_GET_OR(n, scl_gpios, {0}),                    \
			    .sda = GPIO_DT_SPEC_INST_GET_OR(n, sda_gpios, {0}),))                  \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.irq_config = i2c_stm8_irq_config_##n,                                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, i2c_stm8_init, NULL, &i2c_stm8_data_##n, &i2c_stm8_config_##n,    \
			      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, &i2c_stm8_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_STM8_INIT)
