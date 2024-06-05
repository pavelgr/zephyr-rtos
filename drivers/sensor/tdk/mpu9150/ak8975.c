/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 * Copyright (c) 2021, pck
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "mpu9150.h"
#include "ak8975.h"

LOG_MODULE_REGISTER(AK8975, CONFIG_SENSOR_LOG_LEVEL);

#define MPU9150_I2C_MST_CTRL_I2C_MST_CLK_400KHZ     13
#define AK8975_SET_MODE_DELAY_MS                    1

int ak8975_convert_magn(struct sensor_value *val, uint8_t st2, int16_t raw_val, uint8_t adj)
{
	if ((st2 & AK8975_ST2_DERR_MASK) != 0) {
		LOG_INF("Magnetometer value invalid.");
		return -EINVAL;
	}

	if ((st2 & (AK8975_ST2_HOFL_MASK)) != 0) {
		LOG_INF("Magnetometer value overflow.");
		return -EOVERFLOW;
	}

    /* From datasheet:
     * 1) Hadj = H * (((ASA - 128) * 0.5) /128 +1)
     * 
     * Hadj is 13bit signed value representing +/-1200 uT.
     * Hence sensitivity is: 1200/2^12 = 0.29296875
     * 
     * Zephyr sensor represents magnetic data in Gauss, or uT/100.
     * 
     * We can reorder 1 above and incorporate scaling in order to get the result in Gauss:
     * 2) Hadj = H * (ASA / 256 + 0.5) * 0.29296875 /100
     * 
     * In order to avoid floating point arithmetic, keep reasonable 
     * precision and simplify conversion to Zephyr sensor values,
     * we can multiply both sides by a large integer value, 
     * namely the precision of the sensor's data type definition: 10^6.
     * 
     * A bit of manipulation gives the following:
     * 3) Hadj *10^6 = (ASA *10^6 /265 + 5 *10^5) * H * 29296875 /10^10
     * 
     * Separating the above to sensor's data integer and fractional parts is trivial.
     */ 

    int64_t magn = ((((int64_t)adj *1000000) >> 8) + 500000) * raw_val * 29296875 /10000000000;

	val->val1 = magn / 1000000;
	val->val2 = magn % 1000000;
    
	return 0;
}

static int mpu9150_execute_rw(const struct device *dev, uint8_t reg, bool write)
{
	/* Instruct the MPU9150 to access over its external i2c bus
	 * given device register with given details
	 */
	int ret, count;
	uint8_t val;
	uint8_t mode_bit = 0x00;

	if (!write) {
		mode_bit = (1 << MPU9150_I2C_SLV4_ADDR_I2C_SLV4_RW_POS);
	}

	// Set target i2c address
	ret = mpu9150_reg_write(dev,
            MPU9150_REG_I2C_SLV4_ADDR,
			AK8975_I2C_ADDR | mode_bit);
	if (ret < 0) {
		LOG_ERR("Failed to write slave address.");
		return ret;
	}

	// Set target i2c register
	ret = mpu9150_reg_write(dev,
			MPU9150_REG_I2C_SLV4_REG,
			reg);
	if (ret < 0) {
		LOG_ERR("Failed to write slave register.");
		return ret;
	}

	// Initiate transfer
    val = (1 << MPU9150_I2C_SLV4_CTRL_I2C_SLV4_EN_POS);
    val |= (1 << MPU9150_I2C_SLV4_CTRL_I2C_SLV4_INT_EN_POS);
	ret = mpu9150_reg_field_update(dev,
				    MPU9150_REG_I2C_SLV4_CTRL,
				    (MPU9150_I2C_SLV4_CTRL_I2C_SLV4_EN_MASK | MPU9150_I2C_SLV4_CTRL_I2C_SLV4_INT_EN_MASK),
                    val);
	if (ret < 0) {
		LOG_ERR("Failed to initiate i2c slave transfer.");
		return ret;
	}

	// Wait for a transfer to be ready
    count = 0;
	do {
        k_msleep(1);

        ret = mpu9150_reg_read(dev, MPU9150_REG_I2C_MST_STATUS, &val, 1);
		if (ret < 0) {
			LOG_ERR("Waiting for slave status failed.");
			return ret;
		}
        
        if (count++ >= 10) {
			LOG_ERR("Waiting for slave status failed 2.");
            break;
        }
	} while ((val & MPU9150_I2C_MST_STATUS_I2C_SLV4_DONE_MASK) == 0);

	return 0;
}

static int ak8975_read_reg(const struct device *dev, uint8_t reg, uint8_t *data)
{
	int ret;

	// Execute transfer
	ret = mpu9150_execute_rw(dev, reg, false);
	if (ret < 0) {
		LOG_ERR("Failed to transfer read from slave.");
		return ret;
	}

	// Read the result
    ret = mpu9150_reg_read(dev, MPU9150_REG_I2C_SLV4_DI, data, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read slave data from MPU9150.");
		return ret;
	}

	return 0;
}

static int ak8975_write_reg(const struct device *dev, uint8_t reg, uint8_t data)
{
	int ret;

	// Set the data to write
	ret = mpu9150_reg_write(dev, MPU9150_REG_I2C_SLV4_DO, data);
	if (ret < 0) {
		LOG_ERR("Failed to write slave data to MPU9150.");
		return ret;
	}

	// Execute transfer
	ret = mpu9150_execute_rw(dev, reg, true);
	if (ret < 0) {
		LOG_ERR("Failed to transfer write to slave.");
		return ret;
	}

	return 0;
}

static int mpu9150_init_master(const struct device *dev)
{
	int ret, count;
    uint8_t val;
	const struct mpu9150_config *cfg = dev->config;

	// Reset auxiliary I2C bus and disable master on it.
	ret = mpu9150_reg_field_update(dev,
                MPU9150_REG_USER_CTRL,
                (MPU9150_USER_CTRL_I2C_MST_RESET_MASK | MPU9150_USER_CTRL_I2C_MST_EN_MASK),
                (1 << MPU9150_USER_CTRL_I2C_MST_RESET_POS));
	if (ret < 0) {
		LOG_ERR("Failed to reset master i2c mode.");
		return ret;
	}

	// Wait for reset to complete
    count = 0;
	do {
        k_msleep(1);

        ret = mpu9150_reg_read(dev, MPU9150_REG_USER_CTRL, &val, 1);
		if (ret < 0) {
			LOG_ERR("Waiting for slave ctrl failed.");
			return ret;
		}
        if (count++ >= 10) {
			LOG_ERR("Waiting for slave ctrl failed 2.");
            break;
        }
	} while ((val & MPU9150_USER_CTRL_I2C_MST_RESET_MASK) != 0);

	// Set MPU9150 I2C bus as 400kHz and delay drdy interrupt until magn data is in EXT_SENS_DATA.
    val = (MPU9150_I2C_MST_CTRL_I2C_MST_CLK_400KHZ << MPU9150_I2C_MST_CTRL_I2C_MST_CLK_POS);
    val |= (1 << MPU9150_I2C_MST_CTRL_WAIT_FOR_ES_POS);
    val |= (1 << MPU9150_I2C_MST_CTRL_I2C_MST_P_NSR_POS);
	ret = mpu9150_reg_field_update(dev,
				MPU9150_REG_I2C_MST_CTRL,
                (MPU9150_I2C_MST_CTRL_I2C_MST_CLK_MASK | MPU9150_I2C_MST_CTRL_WAIT_FOR_ES_MASK | MPU9150_I2C_MST_CTRL_I2C_MST_P_NSR_MASK),
				val);
	if (ret < 0) {
		LOG_ERR("Failed to set MPU9150 master i2c speed.");
		return ret;
	}

    // Set I2C master delay, the reduced access rate of I2C slaves relative to the Sample Rate.
	ret = mpu9150_reg_field_update(dev,
				    MPU9150_REG_I2C_SLV4_CTRL,
				    MPU9150_I2C_SLV4_CTRL_I2C_MST_DLY_MASK,
                    (cfg->i2c_mst_dly << MPU9150_I2C_SLV4_CTRL_I2C_MST_DLY_POS));
	if (ret < 0) {
		LOG_ERR("Failed to initiate i2c slave transfer.");
		return ret;
	}

	// Enable auxiliary I2C bus as master.
	ret = mpu9150_reg_field_update(dev,
                MPU9150_REG_USER_CTRL,
                MPU9150_USER_CTRL_I2C_MST_EN_MASK,
                (1 << MPU9150_USER_CTRL_I2C_MST_EN_POS));
	if (ret < 0) {
		LOG_ERR("Failed to enable master i2c mode.");
		return ret;
	}

	return 0;
}

static int ak8975_set_mode(const struct device *dev, uint8_t mode)
{
	int ret;

	ret = ak8975_write_reg(dev, AK8975_REG_CNTL, mode);
	if (ret < 0) {
		LOG_ERR("Failed to set AK8975 mode.");
		return ret;
	}

	// Wait for mode to change
	k_msleep(AK8975_SET_MODE_DELAY_MS);

	return 0;
}

static int ak8975_fetch_adjustment(const struct device *dev)
{
	int ret;
	struct mpu9150_data *data = dev->data;

	// Change to FUSE access mode to access adjustment registers
	ret = ak8975_set_mode(dev, AK8975_CNTL_MODE_ROM_ACCESS);
	if (ret < 0) {
		LOG_ERR("Failed to set chip in fuse access mode.");
		return ret;
	}

	ret = ak8975_read_reg(dev, AK8975_REG_ASAX, &data->magn_asax);
	if (ret < 0) {
		LOG_ERR("Failed to read sensitivity data.");
		return ret;
	}

	ret = ak8975_read_reg(dev, AK8975_REG_ASAY, &data->magn_asay);
	if (ret < 0) {
		LOG_ERR("Failed to read sensitivity data.");
		return ret;
	}

	ret = ak8975_read_reg(dev, AK8975_REG_ASAZ, &data->magn_asaz);
	if (ret < 0) {
		LOG_ERR("Failed to read sensitivity data.");
		return ret;
	}

	// Change back to the powerdown mode
	ret = ak8975_set_mode(dev, AK8975_CNTL_MODE_POWERDOWN);
	if (ret < 0) {
		LOG_ERR("Failed to set chip in power down mode.");
		return ret;
	}

	LOG_DBG("Adjustment values %d %d %d", data->magn_asax,
		data->magn_asay, data->magn_asaz);

	return 0;
}

static int ak8975_init_readout(const struct device *dev)
{
	int ret;
    uint8_t val;

	// Set target i2c address
    val = AK8975_I2C_ADDR;
    val |= (1 << MPU9150_I2C_SLVx_ADDR_I2C_SLVx_RW_POS);
    ret = mpu9150_reg_write(dev, MPU9150_REG_I2C_SLV0_ADDR, val);
	if (ret < 0) {
		LOG_ERR("Failed to write slave0 address.");
		return ret;
	}

    ret = mpu9150_reg_write(dev, MPU9150_REG_I2C_SLV1_ADDR, AK8975_I2C_ADDR);
	if (ret < 0) {
		LOG_ERR("Failed to write slave1 address.");
		return ret;
	}

	// Set target as data registers
    ret = mpu9150_reg_write(dev, MPU9150_REG_I2C_SLV0_REG, AK8975_REG_HXL);
	if (ret < 0) {
		LOG_ERR("Failed to write slave0 register.");
		return ret;
	}

    ret = mpu9150_reg_write(dev, MPU9150_REG_I2C_SLV1_REG, AK8975_REG_CNTL);
	if (ret < 0) {
		LOG_ERR("Failed to write slave1 register.");
		return ret;
	}

    ret = mpu9150_reg_write(dev, MPU9150_REG_I2C_SLV1_DO, AK8975_CNTL_MODE_SINGLE_MEASUREMENT);
	if (ret < 0) {
		LOG_ERR("Failed to write slave1 data.");
		return ret;
	}

    // Enable slave read delay
    val = (1 << MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN_POS);
    val |= (1 << MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN_POS);
    ret = mpu9150_reg_field_update(dev, 
                MPU9150_REG_I2C_MST_DELAY_CTRL, 
                (MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN_MASK | MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN_MASK),
                val);
	if (ret < 0) {
		LOG_ERR("Failed to write slave delay.");
		return ret;
	}

	// Initiate readout at sample rate
    val = (7 << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_LEN_POS);
    val |= (1 << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_EN_POS);
    ret = mpu9150_reg_write(dev, MPU9150_REG_I2C_SLV0_CTRL, val);
	if (ret < 0) {
		LOG_ERR("Failed to write slave0 enable.");
		return ret;
	}

    val = (1 << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_LEN_POS);
    val |= (1 << MPU9150_I2C_SLVx_CTRL_I2C_SLVx_EN_POS);
    ret = mpu9150_reg_write(dev, MPU9150_REG_I2C_SLV1_CTRL, val);
	if (ret < 0) {
		LOG_ERR("Failed to write slave1 enable.");
		return ret;
	}

	return 0;
}

int ak8975_init(const struct device *dev)
{
	int ret;
	uint8_t buf;

	ret = mpu9150_init_master(dev);
	if (ret < 0) {
		LOG_ERR("Initializing MPU9150 master mode failed.");
		return ret;
	}

	ret = ak8975_set_mode(dev, AK8975_CNTL_MODE_POWERDOWN);
	if (ret < 0) {
		LOG_ERR("Resetting AK8975 failed.");
		return ret;
	}

	// First check that the chip says hello
	ret = ak8975_read_reg(dev, AK8975_REG_WIA, &buf);
	if (ret < 0) {
		LOG_ERR("Failed to read AK8975 chip id.");
		return ret;
	}

	if (buf != AK8975_WIA) {
		LOG_ERR("Invalid AK8975 chip id (0x%X).", buf);
		return -ENOTSUP;
	}

	// Fetch calibration data
	ret = ak8975_fetch_adjustment(dev);
	if (ret < 0) {
		LOG_ERR("Failed fetching adjestment values.");
		return ret;
	}

    // Init constant readouts at sample rate
    ret = ak8975_init_readout(dev);
    if (ret < 0) {
        LOG_ERR("Initializing AK8975 readout failed.");
        return ret;
    }

	return 0;
}
