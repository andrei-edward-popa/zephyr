/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_eeprom

#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>

struct eeprom_stm8_config {
	const struct device *flash;
	size_t size;
};

static int eeprom_stm8_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct eeprom_stm8_config *config = dev->config;

	return flash_read(config->flash, offset, data, len);
}

static int eeprom_stm8_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const struct eeprom_stm8_config *config = dev->config;

	return flash_write(config->flash, offset, data, len);
}

static size_t eeprom_stm8_size(const struct device *dev)
{
	const struct eeprom_stm8_config *config = dev->config;

	return config->size;
}

static int eeprom_stm8_init(const struct device *dev)
{
	const struct eeprom_stm8_config *config = dev->config;

	return device_is_ready(config->flash) ? 0 : -ENODEV;
}

static DEVICE_API(eeprom, eeprom_stm8_api) = {
	.read = eeprom_stm8_read,
	.write = eeprom_stm8_write,
	.size = eeprom_stm8_size,
};

#define STM8_EEPROM_DEFINE(inst)                                                                   \
	static const struct eeprom_stm8_config eeprom_stm8_config_##inst = {                       \
		.flash = DEVICE_DT_GET(DT_INST_PHANDLE(inst, flash)),                              \
		.size = DT_REG_SIZE(DT_INST_PHANDLE(inst, flash)),                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, eeprom_stm8_init, NULL, NULL, &eeprom_stm8_config_##inst,      \
			      POST_KERNEL, CONFIG_EEPROM_INIT_PRIORITY, &eeprom_stm8_api);

DT_INST_FOREACH_STATUS_OKAY(STM8_EEPROM_DEFINE)
