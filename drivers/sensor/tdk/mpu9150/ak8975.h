/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 * Copyright (c) 2024, pck
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MPU9150_AK8975_H_
#define ZEPHYR_DRIVERS_SENSOR_MPU9150_AK8975_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <stdint.h>

#define AK8975_I2C_ADDR			0x0C
#define AK8975_WIA		        0x48

#define AK8975_REG_WIA			0x00
#define AK8975_REG_INFO			0x01
#define AK8975_REG_ST1			0x02
#define AK8975_REG_HXL			0x03
#define AK8975_REG_ST2			0x09
#define AK8975_REG_CNTL         0x0A
#define AK8975_REG_ASAX			0x10
#define AK8975_REG_ASAY			0x10
#define AK8975_REG_ASAZ			0x10

#define AK8975_ST1_DRDY_POS	    0
#define AK8975_ST1_DRDY_MASK    (0x1 << AK8975_ST1_DRDY_POS)

#define AK8975_ST2_DERR_POS	    2
#define AK8975_ST2_DERR_MASK    (0x1 << AK8975_ST2_DERR_POS)
#define AK8975_ST2_HOFL_POS	    3
#define AK8975_ST2_HOFL_MASK    (0x1 << AK8975_ST2_HOFL_POS)

#define AK8975_CNTL_MODE_POS	0
#define AK8975_CNTL_MODE_MASK   (0xF << AK8975_CNTL_MODE_POS)

#define AK8975_CNTL_MODE_POWERDOWN              0x0
#define AK8975_CNTL_MODE_SINGLE_MEASUREMENT     0x1
#define AK8975_CNTL_MODE_SELF_TEST              0x8
#define AK8975_CNTL_MODE_ROM_ACCESS             0xF

int ak8975_init(const struct device *dev);
int ak8975_convert_magn(struct sensor_value *val, uint8_t st2, int16_t raw_val, uint8_t adj);

#endif /* ZEPHYR_DRIVERS_SENSOR_MPU9150_AK8975_H_ */
