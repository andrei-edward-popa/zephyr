/*
 * Copyright (c) 2022 Andrei-Edward Popa
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <stdio.h>

/**
 * @file Sample app using DS3231 through I2C.
 */

#define DS3231_I2C_ADDR		0x68

#define RTC_SEC_REG_ADDR	0x0
#define RTC_MIN_REG_ADDR	0x1
#define RTC_HR_REG_ADDR		0x2
#define RTC_DAY_REG_ADDR	0x3
#define RTC_DATE_REG_ADDR	0x4
#define RTC_MON_REG_ADDR	0x5
#define RTC_YR_REG_ADDR		0x6
#define RTC_CTL_REG_ADDR	0x0e
#define RTC_STAT_REG_ADDR	0x0f

#define RTC_CTL_BIT_RS1		0x8
#define RTC_CTL_BIT_RS2		0x10

#define RTC_STAT_BIT_OSF	0x80

static int ds3231_write_byte(const struct device *dev, uint8_t reg, uint8_t val)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = &val;
	msgs[1].len = 1;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(dev, msgs, 2, DS3231_I2C_ADDR);
}

__attribute__((unused))
static int ds3231_write_bytes(const struct device *dev, uint8_t reg, uint8_t len, uint8_t *data)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = data;
	msgs[1].len = len;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(dev, msgs, 2, DS3231_I2C_ADDR);
}

static int ds3231_read_byte(const struct device *dev, uint8_t reg, uint8_t* data)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = data;
	msgs[1].len = 1;
	msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(dev, msgs, 2, DS3231_I2C_ADDR);
}

__attribute__((unused))
static int ds3231_read_bytes(const struct device *dev, uint8_t reg, uint8_t len, uint8_t* data)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = data;
	msgs[1].len = len;
	msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(dev, msgs, 2, DS3231_I2C_ADDR);
}

static int ds3231_rtc_get(const struct device *dev, struct rtc_time *time)
{
	uint8_t status, tmp_reg, tmp[7];

	ds3231_read_byte(dev, RTC_STAT_REG_ADDR, &status);

	ds3231_read_bytes(dev, RTC_SEC_REG_ADDR, 7, tmp);

	if (status & RTC_STAT_BIT_OSF) {
		printf("Warning: RTC oscillator has stopped\n");
		ds3231_read_byte(dev, RTC_STAT_REG_ADDR, &tmp_reg);
		ds3231_write_byte(dev, RTC_STAT_REG_ADDR, tmp_reg & ~RTC_STAT_BIT_OSF);
		return -EINVAL;
	}

	time->tm_sec   = bcd2bin(tmp[0] & 0x7F);
	time->tm_min   = bcd2bin(tmp[1] & 0x7F);
	time->tm_hour  = bcd2bin(tmp[2] & 0x3F);
	time->tm_mday  = bcd2bin(tmp[4] & 0x3F);
	time->tm_mon   = bcd2bin(tmp[5] & 0x1F);
	time->tm_year  = bcd2bin(tmp[6]) + ((tmp[5] & 0x80) ? 2000 : 1900);
	time->tm_wday  = bcd2bin((tmp[3] - 1) & 0x07);
	time->tm_yday  = 0;
	time->tm_isdst = 0;

	return 0;
}

static int ds3231_rtc_set(const struct device *dev, const struct rtc_time *time)
{
	uint8_t century;

	ds3231_write_byte(dev, RTC_YR_REG_ADDR, bin2bcd(time->tm_year % 100));

	century = (time->tm_year >= 2000) ? 0x80 : 0;
	ds3231_write_byte(dev, RTC_MON_REG_ADDR, bin2bcd(time->tm_mon) | century);

	ds3231_write_byte(dev, RTC_DAY_REG_ADDR, bin2bcd(time->tm_wday + 1));
	ds3231_write_byte(dev, RTC_DATE_REG_ADDR, bin2bcd(time->tm_mday));
	ds3231_write_byte(dev, RTC_HR_REG_ADDR, bin2bcd(time->tm_hour));
	ds3231_write_byte(dev, RTC_MIN_REG_ADDR, bin2bcd(time->tm_min));
	ds3231_write_byte(dev, RTC_SEC_REG_ADDR, bin2bcd(time->tm_sec));

	return 0;
}

static int ds3231_rtc_reset(const struct device *dev)
{
	int ret;

	ret = ds3231_write_byte(dev, RTC_CTL_REG_ADDR, RTC_CTL_BIT_RS1 | RTC_CTL_BIT_RS2);
	if (ret < 0)
		return ret;

	return 0;
}

int main(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	struct rtc_time time = { 0, 43, 20, 27, 3, 2025, 4, 0, 0 };
	int ret;

	if (!device_is_ready(i2c_dev)) {
		printf("I2C: Device is not ready.\n");
		return 0;
	}

	ret = ds3231_rtc_reset(i2c_dev);
	if (ret) {
		printf("Could not reset ds3231\n");
		return ret;
	}

	printf("gg\n");


	ret = ds3231_rtc_set(i2c_dev, &time);
	if (ret) {
		printf("Could not set time for ds3231\n");
		return ret;
	}
	printf("gg\n");

	while (1) {
		ret = ds3231_rtc_get(i2c_dev, &time);
		if (ret) {
			printf("Could not get time for ds3231\n");
			return ret;
		}

		uint8_t data[3];
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 1, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		printf("Seconds: %02d\n", data[0]);
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 2, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		data[1] = bcd2bin(data[1]) & 0x7F;
		printf("Minutes:Seconds: %02d:%02d\n", data[1], data[0]);
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 3, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		data[1] = bcd2bin(data[1]) & 0x7F;
		data[2] = bcd2bin(data[2]) & 0x3F;
		printf("Hour:Minutes:Seconds: %02d:%02d:%02d\n", data[2], data[1], data[0]);
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 2, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		data[1] = bcd2bin(data[1]) & 0x7F;
		printf("Minutes:Seconds: %02d:%02d\n", data[1], data[0]);
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 1, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		printf("Seconds: %02d\n", data[0]);
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 3, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		data[1] = bcd2bin(data[1]) & 0x7F;
		data[2] = bcd2bin(data[2]) & 0x3F;
		printf("Hour:Minutes:Seconds: %02d:%02d:%02d\n", data[2], data[1], data[0]);
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 3, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		data[1] = bcd2bin(data[1]) & 0x7F;
		data[2] = bcd2bin(data[2]) & 0x3F;
		printf("Hour:Minutes:Seconds: %02d:%02d:%02d\n", data[2], data[1], data[0]);
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 1, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		printf("Seconds: %02d\n", data[0]);
		ds3231_read_bytes(i2c_dev, RTC_SEC_REG_ADDR, 2, data);
		data[0] = bcd2bin(data[0]) & 0x7F;
		data[1] = bcd2bin(data[1]) & 0x7F;
		printf("Minutes:Seconds: %02d:%02d\n", data[1], data[0]);
		printf("Date: %4d-%02d-%02d (wday=%d) Time: %2d:%02d:%02d\n",
		       time.tm_year, time.tm_mon, time.tm_mday, time.tm_wday,
		       time.tm_hour, time.tm_min, time.tm_sec);

		//k_msleep(1);
	}

	return 0;
}
