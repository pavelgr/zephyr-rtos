/*
 * Copyright (c) 2024 pck
 * 
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Common fields for BMP280
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMP280_USER_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMP280_USER_H_

#include <zephyr/drivers/sensor.h>

/* Custom ATTR values */
#define BMP280_ATTR_IIR_FILTER      (SENSOR_ATTR_PRIV_START + 1u)
#define BMP280_ATTR_TIME_SB         (SENSOR_ATTR_PRIV_START + 2u)

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMP280_USER_H_ */
