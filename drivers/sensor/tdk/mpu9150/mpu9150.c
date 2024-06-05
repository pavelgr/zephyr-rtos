/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 * Copyright (c) 2024, pck
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/devicetree.h>

#include "mpu9150.h"

#ifdef CONFIG_MPU9150_MAGN_EN
#include "ak8975.h"
#endif

#define MPU9150_TEMP_SENSITIVITY	340
#define MPU9150_TEMP_OFFSET		    35

LOG_MODULE_REGISTER(MPU9150, CONFIG_SENSOR_LOG_LEVEL);

static inline int mpu9150_bus_check(const struct device *dev)
{
    const struct mpu9150_config *cfg = dev->config;
    
    return cfg->bus_io->check(&cfg->bus);
}

inline int mpu9150_reg_read(const struct device *dev, uint8_t start, uint8_t *buf, int size)
{
    const struct mpu9150_config *cfg = dev->config;

    return cfg->bus_io->read(&cfg->bus, start, buf, size);
}

inline int mpu9150_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct mpu9150_config *cfg = dev->config;

    return cfg->bus_io->write(&cfg->bus, reg, val);
}

int mpu9150_reg_field_update(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    int ret;
    uint8_t old_value, new_value;
    const struct mpu9150_config *cfg = dev->config;

    if ((ret = cfg->bus_io->read(&cfg->bus, reg, &old_value, 1)) != 0) {
        return ret;
    }

    new_value = (old_value & ~mask) | (val & mask);
    if (new_value == old_value) {
        return 0;
    }

    return cfg->bus_io->write(&cfg->bus, reg, new_value);
}

/* see "Accelerometer Measurements" section from register map description */
static void mpu9150_convert_accel(struct sensor_value *val, int16_t raw_val, uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
static void mpu9150_convert_gyro(struct sensor_value *val, int16_t raw_val, uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) / (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
static inline void mpu9150_convert_temp(struct sensor_value *val, int16_t raw_val)
{
	/* Temp[*C] = (raw / sensitivity) + offset */
	val->val1 = (raw_val / MPU9150_TEMP_SENSITIVITY) + MPU9150_TEMP_OFFSET;
	val->val2 = (((int64_t)(raw_val % MPU9150_TEMP_SENSITIVITY) * 1000000) / MPU9150_TEMP_SENSITIVITY);

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
        
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int mpu9150_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    #ifdef CONFIG_MPU9150_MAGN_EN
	int ret;
    #endif
	struct mpu9150_data *data = dev->data;
    struct mpu9150_sample *sample = &data->sample;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		mpu9150_convert_accel(val, sample->accel_x, data->accel_sensitivity);
		mpu9150_convert_accel(val+1, sample->accel_y, data->accel_sensitivity);
		mpu9150_convert_accel(val+2, sample->accel_z, data->accel_sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_X:
		mpu9150_convert_accel(val, sample->accel_x, data->accel_sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		mpu9150_convert_accel(val, sample->accel_y, data->accel_sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		mpu9150_convert_accel(val, sample->accel_z, data->accel_sensitivity);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		mpu9150_convert_gyro(val, sample->gyro_x, data->gyro_sensitivity);
		mpu9150_convert_gyro(val+1, sample->gyro_y, data->gyro_sensitivity);
		mpu9150_convert_gyro(val+2, sample->gyro_z, data->gyro_sensitivity);
		break;
	case SENSOR_CHAN_GYRO_X:
		mpu9150_convert_gyro(val, sample->gyro_x, data->gyro_sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Y:
		mpu9150_convert_gyro(val, sample->gyro_y, data->gyro_sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Z:
		mpu9150_convert_gyro(val, sample->gyro_z, data->gyro_sensitivity);
		break;
    case SENSOR_CHAN_DIE_TEMP:
		mpu9150_convert_temp(val, sample->temp);
		break;
#ifdef CONFIG_MPU9150_MAGN_EN
	case SENSOR_CHAN_MAGN_XYZ:
		ret = ak8975_convert_magn(val, sample->magn_st2, sample->magn_x, data->magn_asax);
		if (ret < 0) {
			return ret;
		}
		ret = ak8975_convert_magn(val+1, sample->magn_st2, sample->magn_y, data->magn_asay);
		if (ret < 0) {
			return ret;
		}
		ret = ak8975_convert_magn(val+2, sample->magn_st2, sample->magn_z, data->magn_asaz);
		return ret;
	case SENSOR_CHAN_MAGN_X:
		return ak8975_convert_magn(val, sample->magn_st2, sample->magn_x, data->magn_asax);
	case SENSOR_CHAN_MAGN_Y:
		return ak8975_convert_magn(val, sample->magn_st2, sample->magn_y, data->magn_asay);
	case SENSOR_CHAN_MAGN_Z:
		return ak8975_convert_magn(val, sample->magn_st2, sample->magn_z, data->magn_asaz);
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int mpu9150_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    int ret;
	int16_t buf[MPU9150_SAMPLE_BUFFER_SIZE];
	struct mpu9150_data *data = dev->data;
	struct mpu9150_sample *sample = &data->sample;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

#ifdef CONFIG_PM_DEVICE
    enum pm_device_state state;
    
    (void)pm_device_state_get(dev, &state);
    if (state != PM_DEVICE_STATE_ACTIVE) {
        return -EBUSY;
    }
#endif

    pm_device_busy_set(dev);

    ret = mpu9150_reg_read(dev,
              MPU9150_REG_ACCEL_XOUT_H,
              (uint8_t *)buf,
              sizeof(buf));
	if (ret < 0) {
        LOG_DBG("Failed sample fetch.");
        goto error;
	}

	sample->accel_x = sys_be16_to_cpu(buf[0]);
	sample->accel_y = sys_be16_to_cpu(buf[1]);
	sample->accel_z = sys_be16_to_cpu(buf[2]);
	sample->temp = sys_be16_to_cpu(buf[3]);
	sample->gyro_x = sys_be16_to_cpu(buf[4]);
	sample->gyro_y = sys_be16_to_cpu(buf[5]);
	sample->gyro_z = sys_be16_to_cpu(buf[6]);

#ifdef CONFIG_MPU9150_MAGN_EN
	sample->magn_x = sys_le16_to_cpu(buf[7]);
	sample->magn_y = sys_le16_to_cpu(buf[8]);
	sample->magn_z = sys_le16_to_cpu(buf[9]);
	sample->magn_st2 = ((uint8_t *)buf)[20];
#endif

error:
    pm_device_busy_clear(dev);
    return ret;
}

#ifdef CONFIG_PM_DEVICE
static int mpu9150_pm_action(const struct device *dev,
                enum pm_device_action action)
{
    int ret;
    uint8_t reg_val;

    switch (action) {
    case PM_DEVICE_ACTION_RESUME:
        reg_val = 0;
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        reg_val = 1;
        break;
    default:
        return -ENOTSUP;
    }

    ret = mpu9150_reg_field_update(dev,
                MPU9150_REG_PWR_MGMT_1,
                MPU9150_PWR_MGMT_1_SLEEP_MASK,
                reg_val << MPU9150_PWR_MGMT_1_SLEEP_POS);
    if (ret < 0) {
        LOG_DBG("Failed to set power mode.");
        return ret;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct sensor_driver_api mpu9150_driver_api = {
    //.attr_set = mpu9150_attr_set,
#if CONFIG_MPU9150_TRIGGER
	.trigger_set = mpu9150_trigger_set,
#endif
	.sample_fetch = mpu9150_sample_fetch,
	.channel_get = mpu9150_channel_get,
};

static int mpu9150_init(const struct device *dev)
{
	int ret;
	uint8_t val;
	struct mpu9150_data *data = dev->data;
    const struct mpu9150_config *cfg = dev->config;

    /* measured in degrees/sec x10 to avoid floating point */
    const uint16_t gyro_sensitivity[] = {1310, 655, 328, 164};

	data->gyro_sensitivity = gyro_sensitivity[cfg->fs_sel];
	data->accel_sensitivity = 14 - cfg->afs_sel;

    if ((ret = mpu9150_bus_check(dev)) < 0) {
        LOG_DBG("bus check failed");
		return ret;
	}

	/* check chip ID */
    ret = mpu9150_reg_read(dev, MPU9150_REG_WHO_AM_I, &val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to read chip id.");
		return ret;
	}

	if (val != MPU9150_WHO_AM_I) {
		LOG_ERR("Invalid chip ID.");
		return -ENOTSUP;
	}

	/* wake up chip */
    ret = mpu9150_reg_field_update(dev,
                MPU9150_REG_PWR_MGMT_1,
                MPU9150_PWR_MGMT_1_SLEEP_MASK,
                0 << MPU9150_PWR_MGMT_1_SLEEP_POS);
    if (ret < 0) {
        LOG_DBG("Failed to set power mode.");
        return ret;
    }

    ret = mpu9150_reg_field_update(dev,
                MPU9150_REG_GYRO_CFG,
                MPU9150_GYRO_CFG_FS_SEL_MASK,
                cfg->fs_sel << MPU9150_GYRO_CFG_FS_SEL_POS);
    if (ret < 0) {
        LOG_DBG("Failed to set fs_sel.");
        return ret;
    }

    ret = mpu9150_reg_field_update(dev,
                MPU9150_REG_ACCEL_CFG,
                MPU9150_ACCEL_CFG_AFS_SEL_MASK,
                cfg->afs_sel << MPU9150_ACCEL_CFG_AFS_SEL_POS);
    if (ret < 0) {
        LOG_DBG("Failed to set afs_sel.");
        return ret;
    }

    ret = mpu9150_reg_field_update(dev,
                MPU9150_REG_CONFIG,
                MPU9150_CONFIG_DLPF_CFG_MASK,
                cfg->dlpf_cfg << MPU9150_CONFIG_DLPF_CFG_POS);
    if (ret < 0) {
        LOG_DBG("Failed to set dlpf_cfg.");
        return ret;
    }

    ret = mpu9150_reg_write(dev, MPU9150_REG_SMPLRT_DIV, cfg->smplrt_div);
    if (ret < 0) {
        LOG_DBG("Failed to set smplrt_div.");
        return ret;
    }

#ifdef CONFIG_MPU9150_MAGN_EN
    ret = ak8975_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize AK8975.");
		return ret;
	}
#endif

#ifdef CONFIG_MPU9150_TRIGGER
	if ((ret = mpu9150_trigger_init(dev)) < 0) {
		LOG_ERR("Failed to initialize interrupts.");
		return ret;
	}
#endif

	return 0;
}

/* Initializes a struct mpu9150_config for an instance on a SPI bus. */
//#define MPU9150_CONFIG_SPI(inst)
    //.bus.spi = SPI_DT_SPEC_INST_GET(inst, MPU9150_SPI_OPERATION, 0),
    //.bus_io = &mpu9150_bus_io_spi,

/* Initializes a struct mpu9150_config for an instance on an I2C bus. */
#define MPU9150_CONFIG_I2C(inst)                \
    .bus.i2c = I2C_DT_SPEC_INST_GET(inst),      \
    .bus_io = &mpu9150_bus_io_i2c,

//#define MPU9150_BUS_CFG(inst)
    //COND_CODE_1(DT_INST_ON_BUS(inst, i2c),
            //(MPU9150_CONFIG_I2C(inst)),
            //(MPU9150_CONFIG_SPI(inst)))

#define MPU9150_BUS_CFG(inst) MPU9150_CONFIG_I2C(inst)
            
#if defined(CONFIG_MPU9150_TRIGGER)
#define MPU9150_INT_CFG(inst) \
	.drdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}),
#else
#define MPU9150_INT_CFG(inst)
#endif

#define MPU9150_INST(inst)                                              \
    static struct mpu9150_data mpu9150_data_##inst;                     \
    static const struct mpu9150_config mpu9150_config_##inst = {        \
        MPU9150_BUS_CFG(inst)                                           \
        MPU9150_INT_CFG(inst)                                           \
        .fs_sel = DT_INST_ENUM_IDX(inst, fs_sel),			            \
        .afs_sel = DT_INST_ENUM_IDX(inst, afs_sel),			            \
        .dlpf_cfg = DT_INST_ENUM_IDX(inst, dlpf_cfg),			        \
        .smplrt_div = (uint8_t)DT_INST_PROP(inst, smplrt_div),			\
        .i2c_mst_dly = (uint8_t)DT_INST_PROP(inst, i2c_mst_dly),	    \
    };                                                                  \
    PM_DEVICE_DT_INST_DEFINE(inst, mpu9150_pm_action);                  \
    SENSOR_DEVICE_DT_INST_DEFINE(                                       \
        inst,                                                           \
        mpu9150_init,                                                   \
        PM_DEVICE_DT_INST_GET(inst),                                    \
        &mpu9150_data_##inst,                                           \
        &mpu9150_config_##inst,                                         \
        POST_KERNEL,                                                    \
        CONFIG_SENSOR_INIT_PRIORITY,                                    \
        &mpu9150_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MPU9150_INST)
