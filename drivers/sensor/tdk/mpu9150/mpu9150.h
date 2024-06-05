/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MPU9150_MPU9150_H_
#define ZEPHYR_DRIVERS_SENSOR_MPU9150_MPU9150_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
//#include <zephyr/kernel.h>
//#include <zephyr/sys/util.h>

//#include <stdint.h>

#define DT_DRV_COMPAT invensense_mpu9150

//#define MPU9150_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
//#define MPU9150_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

union mpu9150_bus {
//#if MPU9150_BUS_SPI
    //struct spi_dt_spec spi;
//#endif
//#if MPU9150_BUS_I2C
    struct i2c_dt_spec i2c;
//#endif
};

typedef int (*mpu9150_bus_check_fn)(const union mpu9150_bus *bus);
typedef int (*mpu9150_reg_read_fn)(const union mpu9150_bus *bus, uint8_t start, uint8_t *buf, int size);
typedef int (*mpu9150_reg_write_fn)(const union mpu9150_bus *bus, uint8_t reg, uint8_t val);

struct mpu9150_bus_io {
    mpu9150_bus_check_fn check;
    mpu9150_reg_read_fn read;
    mpu9150_reg_write_fn write;
};

//#if MPU9150_BUS_SPI
//#define MPU9150_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA) //MODE3
//extern const struct mpu9150_bus_io mpu9150_bus_io_spi;
//#endif

//#if MPU9150_BUS_I2C
extern const struct mpu9150_bus_io mpu9150_bus_io_i2c;
//#endif


#define MPU9150_I2C_ADDR1			        0x68
#define MPU9150_I2C_ADDR2			        0x69
#define MPU9150_WHO_AM_I			        0x68

#define MPU9150_REG_SMPLRT_DIV		        0x19
#define MPU9150_REG_CONFIG		            0x1A
#define MPU9150_REG_GYRO_CFG		        0x1B
#define MPU9150_REG_ACCEL_CFG		        0x1C
//#define MPU9150_REG_ACCEL_CFG2		     0x1D
#define MPU9150_REG_FIFO_EN		            0x23
#define MPU9150_REG_I2C_MST_CTRL			0x24
#define MPU9150_REG_I2C_SLV0_ADDR			0x25
#define MPU9150_REG_I2C_SLV0_REG			0x26
#define MPU9150_REG_I2C_SLV0_CTRL			0x27
#define MPU9150_REG_I2C_SLV1_ADDR			0x28
#define MPU9150_REG_I2C_SLV1_REG			0x29
#define MPU9150_REG_I2C_SLV1_CTRL			0x2A
#define MPU9150_REG_I2C_SLV4_ADDR			0x31
#define MPU9150_REG_I2C_SLV4_REG			0x32
#define MPU9150_REG_I2C_SLV4_DO				0x33
#define MPU9150_REG_I2C_SLV4_CTRL			0x34
#define MPU9150_REG_I2C_SLV4_DI				0x35
#define MPU9150_REG_I2C_MST_STATUS			0x36
#define MPU9150_REG_INT_PIN_CFG             0x37
#define MPU9150_REG_INT_ENABLE              0x38
#define MPU9150_REG_INT_STATUS              0x3A
#define MPU9150_REG_ACCEL_XOUT_H            0x3B
#define MPU9150_REG_TEMP_OUT_H              0x41
#define MPU9150_REG_GYRO_XOUT_H             0x43
#define MPU9150_REG_EXT_SENS_DATA_00        0x49
#define MPU9150_REG_I2C_SLV0_DO			    0x63
#define MPU9150_REG_I2C_SLV1_DO			    0x64
#define MPU9150_REG_I2C_MST_DELAY_CTRL      0x67
#define MPU9150_REG_SIGNAL_PATH_RESET       0x68
#define MPU9150_REG_USER_CTRL               0x6A
#define MPU9150_REG_PWR_MGMT_1		        0x6B
#define MPU9150_REG_PWR_MGMT_2              0x6C
#define MPU9150_REG_WHO_AM_I		        0x75


#define MPU9150_CONFIG_DLPF_CFG_POS                 0
#define MPU9150_CONFIG_DLPF_CFG_MASK                (0x7 << MPU9150_CONFIG_DLPF_CFG_POS)
#define MPU9150_CONFIG_EXT_SYNC_SET_POS             3
#define MPU9150_CONFIG_EXT_SYNC_SET_MASK            (0x7 << MPU9150_CONFIG_EXT_SYNC_SET_POS)

#define MPU9150_GYRO_CFG_FS_SEL_POS		            3
#define MPU9150_GYRO_CFG_FS_SEL_MASK		        (0x3 << MPU9150_GYRO_CFG_FS_SEL_POS)
#define MPU9150_GYRO_CFG_ZG_ST_POS		            5
#define MPU9150_GYRO_CFG_ZG_ST_MASK		            (0x1 << MPU9150_GYRO_CFG_ZG_ST_POS)
#define MPU9150_GYRO_CFG_YG_ST_POS		            6
#define MPU9150_GYRO_CFG_YG_ST_MASK		            (0x1 << MPU9150_GYRO_CFG_YG_ST_POS)
#define MPU9150_GYRO_CFG_XG_ST_POS		            7
#define MPU9150_GYRO_CFG_XG_ST_MASK		            (0x1 << MPU9150_GYRO_CFG_XG_ST_POS)

#define MPU9150_ACCEL_CFG_AFS_SEL_POS		        3
#define MPU9150_ACCEL_CFG_AFS_SEL_MASK		        (0x3 << MPU9150_ACCEL_CFG_AFS_SEL_POS)
#define MPU9150_ACCEL_CFG_ZA_ST_POS		            5
#define MPU9150_ACCEL_CFG_ZA_ST_MASK		        (0x1 << MPU9150_ACCEL_CFG_ZA_ST_POS)
#define MPU9150_ACCEL_CFG_YA_ST_POS		            6
#define MPU9150_ACCEL_CFG_YA_ST_MASK		        (0x1 << MPU9150_ACCEL_CFG_YA_ST_POS)
#define MPU9150_ACCEL_CFG_XA_ST_POS		            7
#define MPU9150_ACCEL_CFG_XA_ST_MASK		        (0x1 << MPU9150_ACCEL_CFG_XA_ST_POS)

#define MPU9150_FIFO_EN_SLV0_FIFO_EN_POS            0
#define MPU9150_FIFO_EN_SLV0_FIFO_EN_MASK           (0x1 << MPU9150_FIFO_EN_SLV0_FIFO_EN_POS)
#define MPU9150_FIFO_EN_SLV1_FIFO_EN_POS            1
#define MPU9150_FIFO_EN_SLV1_FIFO_EN_MASK           (0x1 << MPU9150_FIFO_EN_SLV1_FIFO_EN_POS)
#define MPU9150_FIFO_EN_SLV2_FIFO_EN_POS            2
#define MPU9150_FIFO_EN_SLV2_FIFO_EN_MASK           (0x1 << MPU9150_FIFO_EN_SLV2_FIFO_EN_POS)
#define MPU9150_FIFO_EN_ACCEL_FIFO_EN_POS           3
#define MPU9150_FIFO_EN_ACCEL_FIFO_EN_MASK          (0x1 << MPU9150_FIFO_EN_ACCEL_FIFO_EN_POS)
#define MPU9150_FIFO_EN_ZG_FIFO_EN_POS              4
#define MPU9150_FIFO_EN_ZG_FIFO_EN_MASK             (0x1 << MPU9150_FIFO_EN_ZG_FIFO_EN_POS)
#define MPU9150_FIFO_EN_YG_FIFO_EN_POS              5
#define MPU9150_FIFO_EN_YG_FIFO_EN_MASK             (0x1 << MPU9150_FIFO_EN_YG_FIFO_EN_POS)
#define MPU9150_FIFO_EN_XG_FIFO_EN_POS              6
#define MPU9150_FIFO_EN_XG_FIFO_EN_MASK             (0x1 << MPU9150_FIFO_EN_XG_FIFO_EN_POS)
#define MPU9150_FIFO_EN_TEMP_FIFO_EN_POS            7
#define MPU9150_FIFO_EN_TEMP_FIFO_EN_MASK           (0x1 << MPU9150_FIFO_EN_TEMP_FIFO_EN_POS)

#define MPU9150_I2C_MST_CTRL_I2C_MST_CLK_POS        0
#define MPU9150_I2C_MST_CTRL_I2C_MST_CLK_MASK       (0xF << MPU9150_I2C_MST_CTRL_I2C_MST_CLK_POS)
#define MPU9150_I2C_MST_CTRL_I2C_MST_P_NSR_POS      4
#define MPU9150_I2C_MST_CTRL_I2C_MST_P_NSR_MASK     (0x1 << MPU9150_I2C_MST_CTRL_I2C_MST_P_NSR_POS)
#define MPU9150_I2C_MST_CTRL_SLV_3_FIFO_EN_POS      5
#define MPU9150_I2C_MST_CTRL_SLV_3_FIFO_EN_MASK     (0x1 << MPU9150_I2C_MST_CTRL_SLV_3_FIFO_EN_POS)
#define MPU9150_I2C_MST_CTRL_WAIT_FOR_ES_POS        6
#define MPU9150_I2C_MST_CTRL_WAIT_FOR_ES_MASK       (0x1 << MPU9150_I2C_MST_CTRL_WAIT_FOR_ES_POS)
#define MPU9150_I2C_MST_CTRL_MULT_MST_EN_POS        7
#define MPU9150_I2C_MST_CTRL_MULT_MST_EN_MASK       (0x1 << MPU9150_I2C_MST_CTRL_MULT_MST_EN_POS)

#define MPU9150_I2C_SLVx_ADDR_I2C_SLVx_ADDR_POS     0
#define MPU9150_I2C_SLVx_ADDR_I2C_SLVx_ADDR_MASK    (0x7F << MPU9150_I2C_SLVx_ADDR_I2C_SLVx_ADDR_POS)
#define MPU9150_I2C_SLVx_ADDR_I2C_SLVx_RW_POS       7
#define MPU9150_I2C_SLVx_ADDR_I2C_SLVx_RW_MASK      (0x1 << MPU9150_I2C_SLVx_ADDR_I2C_SLVx_RW_POS)

#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_LEN_POS	    0
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_LEN_MASK	    (0xF << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_LEN_POS)
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_GRP_POS	    4
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_GRP_MASK     (0x1 << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_GRP_POS)
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_REG_DIS_POS	5
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_REG_DIS_MASK (0x1 << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_REG_DIS_POS)
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_BYTE_SW_POS	6
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_BYTE_SW_MASK (0x1 << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_BYTE_SW_POS)
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_EN_POS	    7
#define MPU9150_I2C_SLVx_CTRL_I2C_SLVx_EN_MASK      (0x1 << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_EN_POS)

#define MPU9150_I2C_SLV4_ADDR_I2C_SLV4_ADDR_POS     0
#define MPU9150_I2C_SLV4_ADDR_I2C_SLV4_ADDR_MASK    (0x7F << MPU9150_I2C_SLV4_ADDR_I2C_SLV4_ADDR_POS)
#define MPU9150_I2C_SLV4_ADDR_I2C_SLV4_RW_POS       7
#define MPU9150_I2C_SLV4_ADDR_I2C_SLV4_RW_MASK      (0x1 << MPU9150_I2C_SLV4_ADDR_I2C_SLV4_RW_POS)

#define MPU9150_I2C_SLV4_CTRL_I2C_MST_DLY_POS	    0
#define MPU9150_I2C_SLV4_CTRL_I2C_MST_DLY_MASK	    (0x1F << MPU9150_I2C_SLV4_CTRL_I2C_MST_DLY_POS)
#define MPU9150_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_POS	5
#define MPU9150_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_MASK (0x1 << MPU9150_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_POS)
#define MPU9150_I2C_SLV4_CTRL_I2C_SLV4_INT_EN_POS	6
#define MPU9150_I2C_SLV4_CTRL_I2C_SLV4_INT_EN_MASK  (0x1 << MPU9150_I2C_SLV4_CTRL_I2C_SLV4_INT_EN_POS)
#define MPU9150_I2C_SLV4_CTRL_I2C_SLV4_EN_POS	    7
#define MPU9150_I2C_SLV4_CTRL_I2C_SLV4_EN_MASK      (0x1 << MPU9150_I2C_SLV4_CTRL_I2C_SLV4_EN_POS)

#define MPU9150_I2C_MST_STATUS_I2C_SLV0_NACK_POS    0
#define MPU9150_I2C_MST_STATUS_I2C_SLV0_NACK_MASK   (0x1 << MPU9150_I2C_MST_STATUS_I2C_SLV0_NACK_POS)
#define MPU9150_I2C_MST_STATUS_I2C_SLV1_NACK_POS    1
#define MPU9150_I2C_MST_STATUS_I2C_SLV1_NACK_MASK   (0x1 << MPU9150_I2C_MST_STATUS_I2C_SLV1_NACK_POS)
#define MPU9150_I2C_MST_STATUS_I2C_SLV2_NACK_POS    2
#define MPU9150_I2C_MST_STATUS_I2C_SLV2_NACK_MASK   (0x1 << MPU9150_I2C_MST_STATUS_I2C_SLV2_NACK_POS)
#define MPU9150_I2C_MST_STATUS_I2C_SLV3_NACK_POS    3
#define MPU9150_I2C_MST_STATUS_I2C_SLV3_NACK_MASK   (0x1 << MPU9150_I2C_MST_STATUS_I2C_SLV3_NACK_POS)
#define MPU9150_I2C_MST_STATUS_I2C_SLV4_NACK_POS    4
#define MPU9150_I2C_MST_STATUS_I2C_SLV4_NACK_MASK   (0x1 << MPU9150_I2C_MST_STATUS_I2C_SLV4_NACK_POS)
#define MPU9150_I2C_MST_STATUS_I2C_LOST_ARB_POS     5
#define MPU9150_I2C_MST_STATUS_I2C_LOST_ARB_MASK    (0x1 << MPU9150_I2C_MST_STATUS_I2C_LOST_ARB_POS)
#define MPU9150_I2C_MST_STATUS_I2C_SLV4_DONE_POS    6
#define MPU9150_I2C_MST_STATUS_I2C_SLV4_DONE_MASK   (0x1 << MPU9150_I2C_MST_STATUS_I2C_SLV4_DONE_POS)
#define MPU9150_I2C_MST_STATUS_PASS_THROUGH_POS     7
#define MPU9150_I2C_MST_STATUS_PASS_THROUGH_MASK    (0x1 << MPU9150_I2C_MST_STATUS_PASS_THROUGH_POS)

#define MPU9150_INT_PIN_CFG_I2C_BYPASS_EN_POS       1
#define MPU9150_INT_PIN_CFG_I2C_BYPASS_EN_MASK      (0x1 << MPU9150_INT_PIN_CFG_I2C_BYPASS_EN_POS)
#define MPU9150_INT_PIN_CFG_FSYNC_INT_EN_POS        2
#define MPU9150_INT_PIN_CFG_FSYNC_INT_EN_MASK       (0x1 << MPU9150_INT_PIN_CFG_FSYNC_INT_EN_POS)
#define MPU9150_INT_PIN_CFG_FSYNC_INT_LEVEL_POS     3
#define MPU9150_INT_PIN_CFG_FSYNC_INT_LEVEL_MASK    (0x1 << MPU9150_INT_PIN_CFG_FSYNC_INT_LEVEL_POS)
#define MPU9150_INT_PIN_CFG_INT_RD_CLEAR_POS        4
#define MPU9150_INT_PIN_CFG_INT_RD_CLEAR_MASK       (0x1 << MPU9150_INT_PIN_CFG_INT_RD_CLEAR_POS)
#define MPU9150_INT_PIN_CFG_LATCH_INT_EN_POS        5
#define MPU9150_INT_PIN_CFG_LATCH_INT_EN_MASK       (0x1 << MPU9150_INT_PIN_CFG_LATCH_INT_EN_POS)
#define MPU9150_INT_PIN_CFG_INT_OPEN_POS            6
#define MPU9150_INT_PIN_CFG_INT_OPEN_MASK           (0x1 << MPU9150_INT_PIN_CFG_INT_OPEN_POS)
#define MPU9150_INT_PIN_CFG_INT_LEVEL_POS           7
#define MPU9150_INT_PIN_CFG_INT_LEVEL_MASK          (0x1 << MPU9150_INT_PIN_CFG_INT_LEVEL_POS)

#define MPU9150_INT_ENABLE_DATA_RDY_EN_POS          0
#define MPU9150_INT_ENABLE_DATA_RDY_EN_MASK         (0x1 << MPU9150_INT_ENABLE_DATA_RDY_EN_POS)
#define MPU9150_INT_ENABLE_I2C_MST_INT_EN_POS       3
#define MPU9150_INT_ENABLE_I2C_MST_INT_EN_MASK      (0x1 << MPU9150_INT_ENABLE_I2C_MST_INT_EN_POS)
#define MPU9150_INT_ENABLE_FIFO_OFLOW_EN_POS        4
#define MPU9150_INT_ENABLE_FIFO_OFLOW_EN_MASK       (0x1 << MPU9150_INT_ENABLE_FIFO_OFLOW_EN_POS)

#define MPU9150_INT_STATUS_DATA_RDY_POS             0
#define MPU9150_INT_STATUS_DATA_RDY_MASK            (0x1 << MPU9150_INT_STATUS_DATA_RDY_POS)
#define MPU9150_INT_STATUS_I2C_MST_INT_POS          3
#define MPU9150_INT_STATUS_I2C_MST_INT_MASK         (0x1 << MPU9150_INT_STATUS_I2C_MST_INT_POS)
#define MPU9150_INT_STATUS_FIFO_OFLOW_POS           4
#define MPU9150_INT_STATUS_FIFO_OFLOW_MASK          (0x1 << MPU9150_INT_STATUS_FIFO_OFLOW_POS)

#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN_POS  0
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN_MASK (0x1 << MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN_POS)
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN_POS  1
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN_MASK (0x1 << MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN_POS)
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV2_DLY_EN_POS  2
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV2_DLY_EN_MASK (0x1 << MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV2_DLY_EN_POS)
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV3_DLY_EN_POS  3
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV3_DLY_EN_MASK (0x1 << MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV3_DLY_EN_POS)
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN_POS  4
#define MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN_MASK (0x1 << MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN_POS)
#define MPU9150_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW_POS  7
#define MPU9150_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW_MASK (0x1 << MPU9150_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW_POS)

#define MPU9150_SIGNAL_PATH_RESET_TEMP_RESET_POS    0
#define MPU9150_SIGNAL_PATH_RESET_TEMP_RESET_MASK   (0x1 << MPU9150_SIGNAL_PATH_RESET_TEMP_RESET_POS)
#define MPU9150_SIGNAL_PATH_RESET_ACCEL_RESET_POS   1
#define MPU9150_SIGNAL_PATH_RESET_ACCEL_RESET_MASK  (0x1 << MPU9150_SIGNAL_PATH_RESET_ACCEL_RESET_POS)
#define MPU9150_SIGNAL_PATH_RESET_GYRO_RESET_POS    2
#define MPU9150_SIGNAL_PATH_RESET_GYRO_RESET_MASK   (0x1 << MPU9150_SIGNAL_PATH_RESET_GYRO_RESET_POS)

#define MPU9150_USER_CTRL_SIG_COND_RESET_POS        0
#define MPU9150_USER_CTRL_SIG_COND_RESET_MASK       (0x1 << MPU9150_USER_CTRL_SIG_COND_RESET_POS)
#define MPU9150_USER_CTRL_I2C_MST_RESET_POS         1
#define MPU9150_USER_CTRL_I2C_MST_RESET_MASK        (0x1 << MPU9150_USER_CTRL_I2C_MST_RESET_POS)
#define MPU9150_USER_CTRL_FIFO_RESET_POS            2
#define MPU9150_USER_CTRL_FIFO_RESET_MASK           (0x1 << MPU9150_USER_CTRL_FIFO_RESET_POS)
#define MPU9150_USER_CTRL_I2C_IF_DIS_POS            4
#define MPU9150_USER_CTRL_I2C_IF_DIS_MASK           (0x1 << MPU9150_USER_CTRL_I2C_IF_DIS_POS)
#define MPU9150_USER_CTRL_I2C_MST_EN_POS            5
#define MPU9150_USER_CTRL_I2C_MST_EN_MASK           (0x1 << MPU9150_USER_CTRL_I2C_MST_EN_POS)
#define MPU9150_USER_CTRL_FIFO_EN_POS               6
#define MPU9150_USER_CTRL_FIFO_EN_MASK              (0x1 << MPU9150_USER_CTRL_FIFO_EN_POS)

#define MPU9150_PWR_MGMT_1_CLKSEL_POS   		    0
#define MPU9150_PWR_MGMT_1_CLKSEL_MASK   		    (0x7 << MPU9150_PWR_MGMT_1_CLKSEL_POS)
#define MPU9150_PWR_MGMT_1_TEMP_DIS_POS   		    3
#define MPU9150_PWR_MGMT_1_TEMP_DIS_MASK   		    (0x1 << MPU9150_PWR_MGMT_1_TEMP_DIS_POS)
#define MPU9150_PWR_MGMT_1_CYCLE_POS   		        5
#define MPU9150_PWR_MGMT_1_CYCLE_MASK   		    (0x1 << MPU9150_PWR_MGMT_1_CYCLE_POS)
#define MPU9150_PWR_MGMT_1_SLEEP_POS   		        6
#define MPU9150_PWR_MGMT_1_SLEEP_MASK   		    (0x1 << MPU9150_PWR_MGMT_1_SLEEP_POS)
#define MPU9150_PWR_MGMT_1_DEVICE_RESET_POS   		7
#define MPU9150_PWR_MGMT_1_DEVICE_RESET_MASK   		(0x1 << MPU9150_PWR_MGMT_1_DEVICE_RESET_POS)

#define MPU9150_PWR_MGMT_2_STBY_ZG_POS              0
#define MPU9150_PWR_MGMT_2_STBY_ZG_MASK             (0x1 << MPU9150_PWR_MGMT_2_STBY_ZG_POS)
#define MPU9150_PWR_MGMT_2_STBY_YG_POS              1
#define MPU9150_PWR_MGMT_2_STBY_YG_MASK             (0x1 << MPU9150_PWR_MGMT_2_STBY_YG_POS)
#define MPU9150_PWR_MGMT_2_STBY_XG_POS              2
#define MPU9150_PWR_MGMT_2_STBY_XG_MASK             (0x1 << MPU9150_PWR_MGMT_2_STBY_XG_POS)
#define MPU9150_PWR_MGMT_2_STBY_ZA_POS              3
#define MPU9150_PWR_MGMT_2_STBY_ZA_MASK             (0x1 << MPU9150_PWR_MGMT_2_STBY_ZA_POS)
#define MPU9150_PWR_MGMT_2_STBY_YA_POS              4
#define MPU9150_PWR_MGMT_2_STBY_YA_MASK             (0x1 << MPU9150_PWR_MGMT_2_STBY_YA_POS)
#define MPU9150_PWR_MGMT_2_STBY_XA_POS              5
#define MPU9150_PWR_MGMT_2_STBY_XA_MASK             (0x1 << MPU9150_PWR_MGMT_2_STBY_XA_POS)
#define MPU9150_PWR_MGMT_2_LP_WAKE_CTRL_POS         6
#define MPU9150_PWR_MGMT_2_LP_WAKE_CTRL_MASK        (0x3 << MPU9150_PWR_MGMT_2_LP_WAKE_CTRL_POS)

#ifdef CONFIG_MPU9150_MAGN_EN
#define MPU9150_SAMPLE_BUFFER_SIZE 11
#else
#define MPU9150_SAMPLE_BUFFER_SIZE 7
#endif


struct mpu9150_sample {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
    
#ifdef CONFIG_MPU9150_MAGN_EN
	int16_t magn_x;
	int16_t magn_y;
	int16_t magn_z;
	uint8_t magn_st2;
#endif
};

struct mpu9150_config {
	uint8_t fs_sel;
	uint8_t afs_sel;
	uint8_t dlpf_cfg;
	uint8_t smplrt_div;
    uint8_t i2c_mst_dly;
    
    union mpu9150_bus bus;
    const struct mpu9150_bus_io *bus_io;

#ifdef CONFIG_MPU9150_TRIGGER
	const struct gpio_dt_spec drdy_gpio;
#endif /* CONFIG_MPU9150_TRIGGER */
};

struct mpu9150_data {
    uint16_t accel_sensitivity;
	uint16_t gyro_sensitivity;

    struct mpu9150_sample sample;

#ifdef CONFIG_MPU9150_MAGN_EN
	uint8_t magn_asax;
	uint8_t magn_asay;
	uint8_t magn_asaz;
#endif

#ifdef CONFIG_MPU9150_TRIGGER
	struct gpio_callback drdy_callback;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;

#ifdef CONFIG_MPU9150_TRIGGER_OWN_THREAD
	struct k_sem drdy_sem;
#elif defined(CONFIG_MPU9150_TRIGGER_GLOBAL_THREAD)
	struct k_work drdy_work;
#endif

#if defined(CONFIG_MPU9150_TRIGGER_GLOBAL_THREAD) || \
	defined(CONFIG_MPU9150_TRIGGER_DIRECT)
	const struct device *dev;
#endif
#endif /* CONFIG_MPU9150_TRIGGER */
};

int mpu9150_reg_read(const struct device *dev, uint8_t start, uint8_t *buf, int size);
int mpu9150_reg_write(const struct device *dev, uint8_t reg, uint8_t val);
int mpu9150_reg_field_update(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t val);

#ifdef CONFIG_MPU9150_TRIGGER
int mpu9150_trigger_init(const struct device *dev);
int mpu9150_trigger_set(
            const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);
#endif /* CONFIG_MPU9150_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_MPU9150_MPU9150_H_ */
