/* Copyright (c) 2020 Facebook, Inc. and its affiliates
 * Copyright (c) 2024 pck
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bosch BMP280 pressure sensor
 * 
 * Sensor:
 * https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/
 * Datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 */

#ifndef __BMP280_H
#define __BMP280_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor/bmp280_user.h>

#define DT_DRV_COMPAT bosch_bmp280

#define BMP280_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define BMP280_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#define BMP280_CHIP_ID                  0x58
#define BME280_CHIP_ID                  0x60

/* registers */
#define BMP280_REG_CHIPID               0xD0
//#define BMP280_REG_VERSION             0xD1
#define BMP280_REG_SOFT_RESET           0xE0
//#define BMP280_REG_CAL26               0xE1 /**< R calibration = 0xE1-0xF0 */
#define BMP280_REG_STATUS               0xF3
#define BMP280_REG_CTRL_MEAS            0xF4
#define BMP280_REG_CONFIG               0xF5
#define BMP280_REG_PRESS_MSB            0xF7
#define BMP280_REG_PRESS_LSB            0xF8
#define BMP280_REG_PRESS_XLSB           0xF9
#define BMP280_REG_TEMP_MSB             0xFA
#define BMP280_REG_TEMP_LSB             0xFB
#define BMP280_REG_TEMP_XLSB            0xFC

#define BMP280_REG_DIG_T1               0x88
#define BMP280_REG_DIG_T2               0x8A
#define BMP280_REG_DIG_T3               0x8C
#define BMP280_REG_DIG_P1               0x8E
#define BMP280_REG_DIG_P2               0x90
#define BMP280_REG_DIG_P3               0x92
#define BMP280_REG_DIG_P4               0x94
#define BMP280_REG_DIG_P5               0x96
#define BMP280_REG_DIG_P6               0x98
#define BMP280_REG_DIG_P7               0x9A
#define BMP280_REG_DIG_P8               0x9C
#define BMP280_REG_DIG_P9               0x9E

/* BMP280_REG_STATUS */
#define BMP280_STATUS_UPDATE            BIT(0)
#define BMP280_STATUS_MEASURING         BIT(3)

/* BMP280_REG_CTRL_MEAS */
#define BMP280_CTRL_MEAS_OSRS_T_POS     5
#define BMP280_CTRL_MEAS_OSRS_T_MASK    (0x07 << BMP280_CTRL_MEAS_OSRS_T_POS)
#define BMP280_CTRL_MEAS_OSRS_P_POS     2
#define BMP280_CTRL_MEAS_OSRS_P_MASK    (0x07 << BMP280_CTRL_MEAS_OSRS_P_POS)
#define BMP280_CTRL_MEAS_MODE_POS       0
#define BMP280_CTRL_MEAS_MODE_MASK      (0x03 << BMP280_CTRL_MEAS_MODE_POS)

/* BMP280_REG_CONFIG */
#define BMP280_CONFIG_TIME_SB_POS       5
#define BMP280_CONFIG_TIME_SB_MASK      (0x07 << BMP280_CONFIG_TIME_SB_POS)
#define BMP280_CONFIG_FILTER_POS        2
#define BMP280_CONFIG_FILTER_MASK       (0x07 << BMP280_CONFIG_FILTER_POS)
#define BMP280_CONFIG_SPI3W_EN_POS      0
#define BMP280_CONFIG_SPI3W_EN_MASK     (0x01 << BMP280_CONFIG_SPI3W_EN_POS)

#define BMP280_SOFT_RESET_CMD           0xB6

#define BMP280_CTRL_MEAS_OSRS_T_SKIP    0x00
#define BMP280_CTRL_MEAS_OSRS_T_1       0x01
#define BMP280_CTRL_MEAS_OSRS_T_2       0x02
#define BMP280_CTRL_MEAS_OSRS_T_4       0x03
#define BMP280_CTRL_MEAS_OSRS_T_8       0x04
#define BMP280_CTRL_MEAS_OSRS_T_16      0x05

#define BMP280_CTRL_MEAS_OSRS_P_SKIP    0x00
#define BMP280_CTRL_MEAS_OSRS_P_1       0x01
#define BMP280_CTRL_MEAS_OSRS_P_2       0x02
#define BMP280_CTRL_MEAS_OSRS_P_4       0x03
#define BMP280_CTRL_MEAS_OSRS_P_8       0x04
#define BMP280_CTRL_MEAS_OSRS_P_16      0x05

#define BMP280_CTRL_MEAS_MODE_SLEEP     0x00
#define BMP280_CTRL_MEAS_MODE_FORCED    0x01
#define BMP280_CTRL_MEAS_MODE_NORMAL    0x03

#define BMP280_CONFIG_FILTER_OFF        0x00
#define BMP280_CONFIG_FILTER_COEFF_2    0x01
#define BMP280_CONFIG_FILTER_COEFF_4    0x02
#define BMP280_CONFIG_FILTER_COEFF_8    0x03
#define BMP280_CONFIG_FILTER_COEFF_16   0x04

#define BMP280_CONFIG_TIME_SB_0000_5MS  0x00
#define BMP280_CONFIG_TIME_SB_0062_5MS  0x01
#define BMP280_CONFIG_TIME_SB_0125_0MS  0x02
#define BMP280_CONFIG_TIME_SB_0250_0MS  0x03
#define BMP280_CONFIG_TIME_SB_0500_0MS  0x04
#define BMP280_CONFIG_TIME_SB_1000_0MS  0x05
#define BMP280_CONFIG_TIME_SB_2000_0MS  0x06
#define BMP280_CONFIG_TIME_SB_4000_0MS  0x07

#define BMP280_CONFIG_SPI3W_ENABLE      0x01
#define BMP280_CONFIG_SPI3W_DISABLE     0x00

#define BMP280_SAMPLE_BUFFER_SIZE       6
#define BMP280_CALIB_BUFFER_SIZE        24

union bmp280_bus {
#if BMP280_BUS_SPI
    struct spi_dt_spec spi;
#endif
#if BMP280_BUS_I2C
    struct i2c_dt_spec i2c;
#endif
};

typedef int (*bmp280_bus_check_fn)(const union bmp280_bus *bus);
typedef int (*bmp280_reg_read_fn)(const union bmp280_bus *bus, uint8_t start, uint8_t *buf, int size);
typedef int (*bmp280_reg_write_fn)(const union bmp280_bus *bus, uint8_t reg, uint8_t val);

struct bmp280_bus_io {
    bmp280_bus_check_fn check;
    bmp280_reg_read_fn read;
    bmp280_reg_write_fn write;
};

#if BMP280_BUS_SPI
#define BMP280_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA) //MODE3
extern const struct bmp280_bus_io bmp280_bus_io_spi;
#endif

#if BMP280_BUS_I2C
extern const struct bmp280_bus_io bmp280_bus_io_i2c;
#endif

struct bmp280_cal_data {
    uint16_t t1;
    int16_t  t2;
    int16_t  t3;
    uint16_t p1;
    int16_t  p2;
    int16_t  p3;
    int16_t  p4;
    int16_t  p5;
    int16_t  p6;
    int16_t  p7;
    int16_t  p8;
    int16_t  p9;
} __packed;

struct bmp280_sample {
    struct sensor_value press;
    struct sensor_value temp;

    uint8_t status;
    int32_t press_raw;
    int32_t temp_raw;
    int32_t t_fine;
};

struct bmp280_config {
    uint8_t spi3w_en;

    union bmp280_bus bus;
    const struct bmp280_bus_io *bus_io;
};

struct bmp280_data {
    uint8_t osr_temp;
    uint8_t osr_press;
    uint8_t iir_filter;
    uint8_t time_sb;

    struct bmp280_cal_data cal;
    struct bmp280_sample sample;
};

#endif /* __BMP280_H */
