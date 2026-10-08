/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT st_stm8_nv_flash

#include <zephyr/arch/stm8/far_io.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>

#define STM8_FLASH_CR2         0x01U
#define STM8_FLASH_NCR2        0x02U
#define STM8_FLASH_IAPSR       0x05U
#define STM8_FLASH_PUKR        0x08U
#define STM8_FLASH_DUKR        0x0aU
#define STM8_FLASH_CR2_ERASE   BIT(5)
#define STM8_FLASH_IAPSR_ERROR BIT(0)
#define STM8_FLASH_IAPSR_PUL   BIT(1)
#define STM8_FLASH_IAPSR_EOP   BIT(2)
#define STM8_FLASH_IAPSR_DUL   BIT(3)
#define STM8_FLASH_KEY1        0x56U
#define STM8_FLASH_KEY2        0xaeU
#define STM8_FLASH_TIMEOUT_US  20000U

struct flash_stm8_config {
	mem_addr_t controller;
	uint32_t base;
	uint32_t size;
	size_t block_size;
	bool eeprom;
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	struct flash_pages_layout layout;
#endif
};

/* Program Flash and EEPROM share one controller and its mode registers. */
K_MUTEX_DEFINE(flash_stm8_lock);

static const struct flash_parameters flash_stm8_parameters = {
	.write_block_size = 1U,
	.caps = {.no_explicit_erase = true},
	.erase_value = 0U,
};

static bool flash_stm8_valid_range(const struct flash_stm8_config *config,
				 off_t offset, size_t len)
{
	return offset >= 0 && (uint32_t)offset <= config->size &&
	       (uint32_t)len <= config->size - (uint32_t)offset;
}

static int flash_stm8_wait(const struct flash_stm8_config *config)
{
	for (uint16_t remaining = STM8_FLASH_TIMEOUT_US; remaining > 0U; remaining--) {
		uint8_t status = sys_read8(config->controller + STM8_FLASH_IAPSR);

		if ((status & STM8_FLASH_IAPSR_ERROR) != 0U) {
			return -EACCES;
		}
		if ((status & STM8_FLASH_IAPSR_EOP) != 0U) {
			return 0;
		}
		k_busy_wait(1U);
	}
	return -ETIMEDOUT;
}

static int flash_stm8_unlock(const struct flash_stm8_config *config)
{
	uint8_t unlocked = config->eeprom ? STM8_FLASH_IAPSR_DUL : STM8_FLASH_IAPSR_PUL;
	mem_addr_t key = config->controller +
			 (config->eeprom ? STM8_FLASH_DUKR : STM8_FLASH_PUKR);

	if ((sys_read8(config->controller + STM8_FLASH_IAPSR) & unlocked) == 0U) {
		sys_write8(config->eeprom ? STM8_FLASH_KEY2 : STM8_FLASH_KEY1, key);
		sys_write8(config->eeprom ? STM8_FLASH_KEY1 : STM8_FLASH_KEY2, key);
	}
	if ((sys_read8(config->controller + STM8_FLASH_IAPSR) & unlocked) == 0U) {
		return -EACCES;
	}
	sys_write8(0U, config->controller + STM8_FLASH_CR2);
	sys_write8(UINT8_MAX, config->controller + STM8_FLASH_NCR2);
	return 0;
}

static void flash_stm8_lock_controller(const struct flash_stm8_config *config)
{
	mem_addr_t status = config->controller + STM8_FLASH_IAPSR;
	uint8_t unlocked = config->eeprom ? STM8_FLASH_IAPSR_DUL : STM8_FLASH_IAPSR_PUL;

	sys_write8(0U, config->controller + STM8_FLASH_CR2);
	sys_write8(UINT8_MAX, config->controller + STM8_FLASH_NCR2);
	sys_write8(sys_read8(status) & ~unlocked, status);
}

static int flash_stm8_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct flash_stm8_config *config = dev->config;
	uint8_t *bytes = data;

	if (!flash_stm8_valid_range(config, offset, len) || (data == NULL && len > 0U)) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	int ret = k_mutex_lock(&flash_stm8_lock, K_FOREVER);

	if (ret != 0) {
		return ret;
	}
	uint32_t address = config->base + (uint32_t)offset;

	for (size_t i = 0U; i < len; i++) {
		bytes[i] = arch_stm8_far_read8(address++);
	}
	return k_mutex_unlock(&flash_stm8_lock);
}

static int flash_stm8_modify(const struct device *dev, off_t offset, const void *data,
			     size_t len, bool erase)
{
	const struct flash_stm8_config *config = dev->config;
	const uint8_t *bytes = data;
	uint8_t zero_word[4] = {0U};

	if (!flash_stm8_valid_range(config, offset, len) ||
	    (!erase && data == NULL && len > 0U) ||
	    (erase && ((uint32_t)offset % config->block_size != 0U ||
		       len % config->block_size != 0U))) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	int ret = k_mutex_lock(&flash_stm8_lock, K_FOREVER);

	if (ret != 0) {
		return ret;
	}
	ret = flash_stm8_unlock(config);
	uint32_t address = config->base + (uint32_t)offset;

	while (ret == 0 && len > 0U) {
		if (erase) {
			const struct arch_stm8_far_write4_config transaction = {
				.control =
					(volatile uint8_t *)(config->controller + STM8_FLASH_CR2),
				.setup = {STM8_FLASH_CR2_ERASE, (uint8_t)~STM8_FLASH_CR2_ERASE},
				.finish = {0U, UINT8_MAX},
				.status =
					(volatile uint8_t *)(config->controller + STM8_FLASH_IAPSR),
				.ready_mask = STM8_FLASH_IAPSR_EOP | STM8_FLASH_IAPSR_ERROR,
			};
			uint8_t status = arch_stm8_far_write4(address, zero_word, &transaction);

			ret = (status & STM8_FLASH_IAPSR_ERROR) != 0U ? -EACCES :
			      ((status & STM8_FLASH_IAPSR_EOP) != 0U ? 0 : -ETIMEDOUT);
			address += config->block_size;
			len -= config->block_size;
		} else {
			arch_stm8_far_write8(address, *bytes);
			ret = flash_stm8_wait(config);
			if (ret == 0 && arch_stm8_far_read8(address) != *bytes) {
				ret = -EIO;
			}
			address++;
			bytes++;
			len--;
		}
	}
	flash_stm8_lock_controller(config);
	int unlock_ret = k_mutex_unlock(&flash_stm8_lock);

	return ret != 0 ? ret : unlock_ret;
}

static int flash_stm8_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	return flash_stm8_modify(dev, offset, data, len, false);
}

static int flash_stm8_erase(const struct device *dev, off_t offset, size_t len)
{
	return flash_stm8_modify(dev, offset, NULL, len, true);
}

static const struct flash_parameters *flash_stm8_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &flash_stm8_parameters;
}

static int flash_stm8_get_size(const struct device *dev, uint64_t *size)
{
	const struct flash_stm8_config *config = dev->config;

	*size = config->size;
	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void flash_stm8_page_layout(const struct device *dev,
				   const struct flash_pages_layout **layout, size_t *count)
{
	const struct flash_stm8_config *config = dev->config;

	*layout = &config->layout;
	*count = 1U;
}
#endif

static DEVICE_API(flash, flash_stm8_api) = {
	.read = flash_stm8_read,
	.write = flash_stm8_write,
	.erase = flash_stm8_erase,
	.get_parameters = flash_stm8_get_parameters,
	.get_size = flash_stm8_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_stm8_page_layout,
#endif
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT
#define STM8_FLASH_LAYOUT(inst)                                                                    \
	.layout = {.pages_count = DT_INST_REG_SIZE(inst) / DT_INST_PROP(inst, erase_block_size),   \
		   .pages_size = DT_INST_PROP(inst, erase_block_size)},
#else
#define STM8_FLASH_LAYOUT(inst)
#endif

#define STM8_FLASH_DEFINE(inst)                                                                    \
	static const struct flash_stm8_config flash_stm8_config_##inst = {                         \
		.controller = DT_REG_ADDR(DT_INST_PARENT(inst)),                                   \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.size = DT_INST_REG_SIZE(inst),                                                    \
		.block_size = DT_INST_PROP(inst, erase_block_size),                                \
		.eeprom = DT_INST_PROP(inst, st_data_eeprom),                                      \
		STM8_FLASH_LAYOUT(inst)                                                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, NULL, &flash_stm8_config_##inst, POST_KERNEL,      \
			      CONFIG_FLASH_INIT_PRIORITY, &flash_stm8_api);

DT_INST_FOREACH_STATUS_OKAY(STM8_FLASH_DEFINE)
