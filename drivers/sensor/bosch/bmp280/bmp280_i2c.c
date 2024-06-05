/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2024 pck
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for BMP280s accessed via I2C.
 */

#include "bmp280.h"

#if BMP280_BUS_I2C

static int bmp280_bus_check_i2c(const union bmp280_bus *bus)
{
	return i2c_is_ready_dt(&bus->i2c) ? 0 : -ENODEV;
}

static int bmp280_reg_read_i2c(const union bmp280_bus *bus,
			       uint8_t start, uint8_t *buf, int size)
{
	return i2c_burst_read_dt(&bus->i2c, start, buf, size);
}

static int bmp280_reg_write_i2c(const union bmp280_bus *bus,
				uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte_dt(&bus->i2c, reg, val);
}

const struct bmp280_bus_io bmp280_bus_io_i2c = {
	.check = bmp280_bus_check_i2c,
	.read = bmp280_reg_read_i2c,
	.write = bmp280_reg_write_i2c,
};
#endif /* BMP280_BUS_I2C */
