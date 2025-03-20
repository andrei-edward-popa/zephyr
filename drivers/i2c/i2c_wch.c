/*
 * Copyright (c) 2025 Andrei-Edward Popa
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_i2c

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_wch);

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>

#include <ch32fun.h>

#include "i2c-priv.h"

typedef void (*irq_config_func_t)(const struct device *port);

struct i2c_wch_config {
	I2C_TypeDef *regs;
	uint32_t speed;
	uint8_t clk_id;
	const struct device *clk_dev;
	irq_config_func_t irq_config_func;
	const struct pinctrl_dev_config *pcfg;
};

struct i2c_wch_data {
	struct k_sem completion;
	uint32_t dev_config;
	struct i2c_msg *msg;
	uint16_t addr;
};

static void i2c_wch_isr(const struct device *dev)
{
	const struct i2c_wch_config *config = dev->config;
	struct i2c_wch_data *data = dev->data;
	I2C_TypeDef *regs = config->regs;
	uint32_t status = regs->STAR1;

	if (status & I2C_STAR1_BERR) {
		regs->STAR1 &= ~I2C_STAR1_BERR;
		k_sem_give(&data->completion);
		return;
	}

	if (status & I2C_STAR1_ARLO) {
		regs->STAR1 &= ~I2C_STAR1_ARLO;
		k_sem_give(&data->completion);
		return;
	}

	if (status & I2C_STAR1_SB) {
		regs->DATAR = (data->addr << 1) | ((data->msg->flags & I2C_MSG_READ) ? 1 : 0);
	} else if (status & I2C_STAR1_ADDR) {
		(void)regs->STAR2;
	} else if (status & I2C_STAR1_TXE) {
		if (data->msg->len > 0) {
			regs->DATAR = *data->msg->buf++;
			data->msg->len--;
		} 
		if (data->msg->len == 0) {
			regs->CTLR1 |= I2C_CTLR1_STOP;
			k_sem_give(&data->completion);
		}
	} else if (status & I2C_STAR1_RXNE) {
		*data->msg->buf++ = regs->DATAR;
		data->msg->len--;
		if (data->msg->len == 0) {
			regs->CTLR1 |= I2C_CTLR1_STOP;
			k_sem_give(&data->completion);
		}
	}
}

static int i2c_wch_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_wch_config *config = dev->config;
	struct i2c_wch_data *data = dev->data;
	I2C_TypeDef *regs = config->regs;
	clock_control_subsys_t clk_sys;
	uint32_t clock_rate;
	uint16_t tmp_reg;
	int err;

	if (!(I2C_MODE_CONTROLLER & dev_config)) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
	case I2C_SPEED_FAST:
		break;
	default:
		return -EINVAL;
	}

	clk_sys = (clock_control_subsys_t)(uintptr_t)config->clk_id;

	err = clock_control_get_rate(config->clk_dev, clk_sys, &clock_rate);
	if (err != 0) {
		return err;
	}

	tmp_reg = regs->CTLR2;
	tmp_reg &= ~I2C_CTLR2_FREQ;
	tmp_reg |= (uint16_t)(clock_rate / 1000000);
	regs->CTLR2 = tmp_reg;

	regs->CTLR1 &= ~I2C_CTLR1_PE;

	tmp_reg = (uint16_t)(clock_rate / (config->speed * 3));
	if ((tmp_reg & I2C_CKCFGR_CCR) == 0) {
		tmp_reg |= 1;
	}
	tmp_reg |= I2C_CKCFGR_FS;
	regs->CKCFGR = tmp_reg;

	regs->CTLR1 |= I2C_CTLR1_PE;

	regs->CTLR2 |= (I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITBUFEN);

	data->dev_config = dev_config;

	return 0;
}

static int i2c_wch_transfer(const struct device *dev, struct i2c_msg *msgs,
			    uint8_t num_msgs, uint16_t addr)
{
	const struct i2c_wch_config *config = dev->config;
	struct i2c_wch_data *data = dev->data;
	I2C_TypeDef *regs = config->regs;

	for (int i = 0; i < num_msgs; i++) {
		while (regs->STAR2 & I2C_STAR2_BUSY);

		data->msg = &msgs[i];
		data->addr = addr;

		regs->CTLR1 |= I2C_CTLR1_START;

		k_sem_take(&data->completion, K_FOREVER);
	}

	return 0;
};

static int i2c_wch_get_config(const struct device *dev, uint32_t *config)
{
	struct i2c_wch_data *data = dev->data;

	if (data->dev_config == 0) {
		return -EIO;
	}

	*config = data->dev_config;

	return 0;
}

static int i2c_wch_init(const struct device *dev)
{
	const struct i2c_wch_config *config = dev->config;
	struct i2c_wch_data *data = dev->data;
	clock_control_subsys_t clk_sys;
	int err;

	clk_sys = (clock_control_subsys_t)(uintptr_t)config->clk_id;

	err = clock_control_on(config->clk_dev, clk_sys);
	if (err < 0) {
		return err;
	}

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	err = i2c_wch_configure(dev, I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(config->speed));
	if (err < 0) {
		return err;
	}
 
	k_sem_init(&data->completion, 0, 1);

	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(i2c, i2c_wch_api) = {
	.configure = i2c_wch_configure,
	.get_config = i2c_wch_get_config,
	.transfer = i2c_wch_transfer,
};

#define I2C_WCH_INIT(inst)								\
	PINCTRL_DT_INST_DEFINE(inst);							\
											\
	static void i2c_wch_config_func_##inst(const struct device *dev);		\
											\
	static struct i2c_wch_config i2c_wch_cfg_##inst = {				\
		.regs = (I2C_TypeDef *)DT_INST_REG_ADDR(inst),				\
		.speed = DT_INST_PROP(inst, clock_frequency),				\
		.clk_id = DT_INST_CLOCKS_CELL(inst, id),				\
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),			\
		.irq_config_func = i2c_wch_config_func_##inst,				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst)				\
	};										\
											\
	static struct i2c_wch_data i2c_wch_data_##inst;					\
											\
	I2C_DEVICE_DT_INST_DEFINE(inst, i2c_wch_init, NULL, &i2c_wch_data_##inst,	\
				 &i2c_wch_cfg_##inst, PRE_KERNEL_2,			\
				 CONFIG_I2C_INIT_PRIORITY, &i2c_wch_api);		\
											\
	static void i2c_wch_config_func_##inst(const struct device *dev)		\
	{										\
		ARG_UNUSED(dev);							\
											\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),		\
				i2c_wch_isr, DEVICE_DT_INST_GET(inst), 0);		\
											\
		irq_enable(DT_INST_IRQN(inst));						\
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_WCH_INIT)
