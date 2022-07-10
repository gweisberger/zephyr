/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_mpu9250

#include <string.h>
#include "mpu9250.h"
#include <zephyr/logging/log.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#define MPU9250_SPI_READM	(1 << 7)	
#define MPU9250_SPI_WRITEM	(0 << 7)

LOG_MODULE_DECLARE(MPU9250, CONFIG_SENSOR_LOG_LEVEL);

static int mpu9250_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct mpu9250_config *config = dev->config;
	uint8_t buffer_tx[2] = { reg | MPU9250_SPI_READM, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	if (spi_transceive_dt(&config->spi, &tx, &rx)) {
		return -EIO;
	}

	return 0;
}

static int mpu9250_spi_write(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct mpu9250_config *config = dev->config;
	uint8_t buffer_tx[1] = { reg | MPU9250_SPI_WRITEM };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};


	if (spi_write_dt(&config->spi, &tx)) {
		return -EIO;
	}

	return 0;
}

mpudev_ctx_t mpu9250_spi_ctx = {
	.read_reg = (mpudev_read_ptr) mpu9250_spi_read,
	.write_reg = (mpudev_write_ptr) mpu9250_spi_write,
};

int mpu9250_spi_init(const struct device *dev)
{
	struct mpu9250_data *data = dev->data;
	const struct mpu9250_config *config = dev->config;

	if (!spi_is_ready(&config->spi)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	data->ctx = &mpu9250_spi_ctx;
	data->ctx->handle = (void *)dev;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
