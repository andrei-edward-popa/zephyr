/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <stdio.h>


#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(i2c_0))
#define I2C_DEV_NODE	DT_ALIAS(i2c_0)
#else
#error "Please set the correct I2C device"
#endif

static int i2c_scan(const struct device *const dev)
{
	uint8_t cnt = 0, first = 0x04, last = 0x77;

	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	for (uint8_t i = 0; i <= last; i += 16) {
		printf("%02x: ", i);
		for (uint8_t j = 0; j < 16; j++) {
			if (i + j < first || i + j > last) {
				printf("   ");
				continue;
			}

			struct i2c_msg msgs[1];
			uint8_t dst;

			/* Send the address to read from */
			msgs[0].buf = &dst;
			msgs[0].len = 0U;
			msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
			if (i2c_transfer(dev, &msgs[0], 1, i + j) == 0) {
				printf("%02x ", i + j);
				++cnt;
			} else {
				printf("-- ");
			}
		}
		printf("\n");
	}

	printf("%u devices found\n", cnt);

	return 0;
}

int main(void)
{
	const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);

	if (!device_is_ready(i2c_dev)) {
		printk("I2C device is not ready\n");
		return -1;
	}

	i2c_scan(i2c_dev);

	return 0;
}
