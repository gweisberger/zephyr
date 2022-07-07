/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MPU9250_MPU9250_H_
#define ZEPHYR_DRIVERS_SENSOR_MPU9250_MPU9250_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*mpudev_write_ptr)(const void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*mpudev_read_ptr)(const void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  mpudev_write_ptr  write_reg;
  mpudev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  const void *handle;
} mpudev_ctx_t;

struct mpu9250_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_sensitivity_shift;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_sensitivity_x10;

#ifdef CONFIG_MPU9250_MAGN_EN
	int16_t magn_x;
	int16_t magn_scale_x;
	int16_t magn_y;
	int16_t magn_scale_y;
	int16_t magn_z;
	int16_t magn_scale_z;
	uint8_t magn_st2;
#endif

	mpudev_ctx_t *ctx;
#ifdef CONFIG_MPU9250_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_MPU9250_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MPU9250_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_MPU9250_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_MPU9250_TRIGGER */
};

struct mpu9250_config {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif
	uint8_t gyro_sr_div;
	uint8_t gyro_dlpf;
	uint8_t gyro_fs;
	uint8_t accel_fs;
	uint8_t accel_dlpf;
#ifdef CONFIG_MPU9250_TRIGGER
	const struct gpio_dt_spec int_pin;
#endif /* CONFIG_MPU9250_TRIGGER */
};

int mpu9250_i2c_init(const struct device *dev);
int mpu9250_spi_init(const struct device *dev);

int mpu9250_read_byte(const mpudev_ctx_t *ctx, uint8_t reg_addr, uint8_t *value);
int mpu9250_write_byte(const mpudev_ctx_t *ctx, uint8_t reg_addr, uint8_t value);
int mpu9250_update_byte(const mpudev_ctx_t *ctx,
				      uint8_t reg_addr, uint8_t mask,
				      uint8_t value);

#ifdef CONFIG_MPU9250_TRIGGER
int mpu9250_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int mpu9250_init_interrupt(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_MPU9250_MPU9250_H_ */
