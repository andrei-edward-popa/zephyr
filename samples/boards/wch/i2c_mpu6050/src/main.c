/*
 * Copyright (c) 2022 Andrei-Edward Popa
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/**
 * @file Sample app using MPU6050 through I2C.
 */

#define MPU6050_I2C_ADDR	0x68

#define MPU6050_SMPRT_DIV	0x19
#define MPU6050_WHO_AM_I	0x75
#define MPU6050_CONFIG		0x1A
#define MPU6050_GYRO_CONFIG	0x1B
#define MPU6050_ACCEL_CONFIG	0x1C
#define MPU6050_INT_PIN_CFG	0x37
#define MPU6050_INT_ENABLE	0x38
#define MPU6050_INT_STATUS	0x3A
#define MPU6050_ACCEL_XOUT_H	0x3B
#define MPU6050_ACCEL_XOUT_L	0x3C
#define MPU6050_PWR_MGMT_1	0x6B

static int acc_lsb_sensitivity;
static int gyro_lsb_sensitivity;

static const uint8_t gyro_scale_option = 0;
static const uint8_t acc_scale_option = 0;

static const struct device *i2c_dev;

struct mpu6050 {
	int acc_x;
	int acc_y;
	int acc_z;
	int temperature;
	int gyro_x;
	int gyro_y;
	int gyro_z;
};

static void mpu6050_get_lsb_sensitivity(uint8_t gyro_scale_option, uint8_t acc_scale_option);

static int mpu6050_write_byte(const struct device *dev, uint8_t reg, uint8_t val)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = &val;
	msgs[1].len = 1;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(dev, msgs, 2, MPU6050_I2C_ADDR);
}

__attribute__((unused))
static int mpu6050_write_bytes(const struct device *dev, uint8_t reg, uint8_t len, uint8_t *data)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = data;
	msgs[1].len = len;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(dev, msgs, 2, MPU6050_I2C_ADDR);
}

static int mpu6050_read_byte(const struct device *dev, uint8_t reg, uint8_t* data)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = data;
	msgs[1].len = 1;
	msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(dev, msgs, 2, MPU6050_I2C_ADDR);
}

static int mpu6050_read_bytes(const struct device *dev, uint8_t reg, uint8_t len, uint8_t* data)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = data;
	msgs[1].len = len;
	msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(dev, msgs, 2, MPU6050_I2C_ADDR);
}

static void mpu6050_get_lsb_sensitivity(uint8_t gyro_option, uint8_t acc_option)
{
	switch (gyro_option) {
	case 0:
		gyro_lsb_sensitivity = 131.f;
		break;
	case 1:
		gyro_lsb_sensitivity = 65.5f;
		break;
	case 2:
		gyro_lsb_sensitivity = 32.8f;
		break;
	case 3:
		gyro_lsb_sensitivity = 16.4f;
		break;
	}

	switch (acc_option) {
	case 0:
		acc_lsb_sensitivity = 16384.f;
		break;
	case 1:
		acc_lsb_sensitivity = 8192.f;
		break;
	case 2:
		acc_lsb_sensitivity = 4096.f;
		break;
	case 3:
		acc_lsb_sensitivity = 2048.f;
		break;
	}
}

static void delay(uint32_t t)
{
	for (volatile uint32_t i = 0; i < 10000 * t; i++);
}

static void mpu6050_init(void)
{
	uint8_t who_am_i = 0;

	delay(50);

	printf("Checking MPU6050...\n");

	mpu6050_read_byte(i2c_dev, MPU6050_WHO_AM_I, &who_am_i);
	if (who_am_i == 0x68) {
		printf("MPU6050 who_am_i = 0x%02x...OK\n", who_am_i);
	} else {
		printf("ERROR!\n");
		while(1) {
			delay(100);
		}
	}

	mpu6050_write_byte(i2c_dev, MPU6050_PWR_MGMT_1, 0x1 << 7);
	delay(100);

	mpu6050_write_byte(i2c_dev, MPU6050_PWR_MGMT_1, 0x00);
	delay(50);

	mpu6050_write_byte(i2c_dev, MPU6050_SMPRT_DIV, 39);
	delay(50);

	mpu6050_write_byte(i2c_dev, MPU6050_CONFIG, 0x00);
	delay(50);

	mpu6050_write_byte(i2c_dev, MPU6050_GYRO_CONFIG, gyro_scale_option << 3);
	delay(50);

	mpu6050_write_byte(i2c_dev, MPU6050_ACCEL_CONFIG, acc_scale_option << 3);
	delay(50);

	mpu6050_get_lsb_sensitivity(gyro_scale_option, acc_scale_option);

	mpu6050_write_byte(i2c_dev, MPU6050_INT_PIN_CFG, (0 << 7) | (0 << 5) | (1 << 4));
	delay(50);

	mpu6050_write_byte(i2c_dev, MPU6050_INT_ENABLE, 1);
	delay(50);

	printf("MPU6050 setting is finished\n");
}

static void mpu6050_get_6_axis_data(struct mpu6050 *mpu6050)
{
	uint8_t data[14];
	mpu6050_read_bytes(i2c_dev, MPU6050_ACCEL_XOUT_H, 14, data);

	mpu6050->acc_x = (int)(short)(((short)data[0] << 8) | data[1]) / acc_lsb_sensitivity;
	mpu6050->acc_y = (int)(short)(((short)data[2] << 8) | data[3]) / acc_lsb_sensitivity;
	mpu6050->acc_z = (int)(short)(((short)data[4] << 8) | data[5]) / acc_lsb_sensitivity;

	mpu6050->temperature = (int)(short)(((short)data[6] << 8) | data[7]) / 340 + 36.53f;

	mpu6050->gyro_x = (int)(short)(((short)data[8] << 8) | data[9]) / gyro_lsb_sensitivity;
	mpu6050->gyro_y = (int)(short)(((short)data[10] << 8) | data[11]) / gyro_lsb_sensitivity;
	mpu6050->gyro_z = (int)(short)(((short)data[12] << 8) | data[13]) / gyro_lsb_sensitivity;
}

const struct device *gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa));

static int mpu6050_data_ready(void)
{
	return gpio_pin_get(gpioa, 3);
}

int main(void)
{
	struct mpu6050 mpu6050;

	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (!device_is_ready(i2c_dev)) {
		printf("I2C: Device is not ready.\n");
		return 0;
	}

	mpu6050_init();

	gpio_pin_configure(gpioa, 3, GPIO_INPUT);

	while (1) {
		if (mpu6050_data_ready() == 1) {
			mpu6050_get_6_axis_data(&mpu6050);
			printf("AX: %d, AY: %d, AZ: %d\n", mpu6050.acc_x, mpu6050.acc_y, mpu6050.acc_z);
			printf("GX: %d, GY: %d, GZ: %d\n", mpu6050.gyro_x, mpu6050.gyro_y, mpu6050.gyro_z);
			printf("TEMP: %d\n", mpu6050.temperature);
			//k_msleep(1);
		}
	}

	return 0;
}
