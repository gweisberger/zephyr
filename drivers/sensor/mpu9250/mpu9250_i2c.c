/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_mpu9250

#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "mpu9250.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_DECLARE(MPU9250, CONFIG_SENSOR_LOG_LEVEL);

static int mpu9250_i2c_read(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint16_t len)
{
	const struct mpu9250_config *config = dev->config;

	return i2c_burst_read_dt(&config->i2c, reg_addr | 0x80, value, len);
}

static int mpu9250_i2c_write(const struct device *dev, uint8_t reg_addr, uint8_t *value,
			    uint16_t len)
{
	const struct mpu9250_config *config = dev->config;

	return i2c_burst_write_dt(&config->i2c, reg_addr | 0x80, value, len);
}

mpudev_ctx_t mpu9250_i2c_ctx = {
	.read_reg = (mpudev_read_ptr) mpu9250_i2c_read,
	.write_reg = (mpudev_write_ptr) mpu9250_i2c_write,
};

int mpu9250_i2c_init(const struct device *dev)
{
	struct mpu9250_data *data = dev->data;
	const struct mpu9250_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	data->ctx = &mpu9250_i2c_ctx;
	data->ctx->handle = (void *)dev;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
