/* Copyright (c) 2020 Facebook, Inc. and its affiliates
 * Copyright (c) 2024 pck
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bosch BMP280 pressure sensor
 *
 * Datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/pm/device.h>

#include "bmp280.h"

#define BMP280_MIN_TEMP_INT (int32_t)-4000
#define BMP280_MAX_TEMP_INT (int32_t)8500
#define BMP280_MIN_PRESS_64INT (uint64_t)(30000 * 256)
#define BMP280_MAX_PRESS_64INT (uint64_t)(110000 * 256)

LOG_MODULE_REGISTER(BMP280, CONFIG_SENSOR_LOG_LEVEL);

static inline int bmp280_bus_check(const struct device *dev)
{
    const struct bmp280_config *cfg = dev->config;

    return cfg->bus_io->check(&cfg->bus);
}

static inline int bmp280_reg_read(const struct device *dev,
                  uint8_t start, uint8_t *buf, int size)
{
    const struct bmp280_config *cfg = dev->config;

    return cfg->bus_io->read(&cfg->bus, start, buf, size);
}

static inline int bmp280_reg_write(const struct device *dev, uint8_t reg,
                   uint8_t val)
{
    const struct bmp280_config *cfg = dev->config;

    return cfg->bus_io->write(&cfg->bus, reg, val);
}

int bmp280_reg_field_update(const struct device *dev,
                uint8_t reg,
                uint8_t mask,
                uint8_t val)
{
    int rc = 0;
    uint8_t old_value, new_value;
    const struct bmp280_config *cfg = dev->config;

    rc = cfg->bus_io->read(&cfg->bus, reg, &old_value, 1);
    if (rc != 0) {
        return rc;
    }

    new_value = (old_value & ~mask) | (val & mask);
    if (new_value == old_value) {
        return 0;
    }

    return cfg->bus_io->write(&cfg->bus, reg, new_value);
}

static int bmp280_attr_set_oversampling(const struct device *dev,
                    enum sensor_channel chan,
                    int32_t val)
{
    uint8_t reg_val = 0;
    uint32_t pos, mask;
    int err;
    struct bmp280_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_PRESS:
        pos = BMP280_CTRL_MEAS_OSRS_P_POS;
        mask = BMP280_CTRL_MEAS_OSRS_P_MASK;
        break;

    case SENSOR_CHAN_AMBIENT_TEMP:
        pos = BMP280_CTRL_MEAS_OSRS_T_POS;
        mask = BMP280_CTRL_MEAS_OSRS_T_MASK;
        break;

    default:
        LOG_DBG("Channel not supported.");
        return -EINVAL;
    }

    /* Value must be a positive power of 2 <= 16 or 0. */
    if ((val < 0) || (val > 16) || ((val & (val - 1)) != 0)) {
        return -EINVAL;
    }

    /* Determine exponent: this corresponds to register setting. */
    if (val > 0) {
        reg_val = 1;
        while ((val % 2) == 0) {
            val >>= 1;
            ++reg_val;
        }
    }

    err = bmp280_reg_field_update(dev,
                      BMP280_REG_CTRL_MEAS,
                      mask,
                      reg_val << pos);
    if (err < 0) {
        return err;
    }

    switch (chan) {
    case SENSOR_CHAN_PRESS:
        data->osr_press = reg_val;
        break;

    case SENSOR_CHAN_AMBIENT_TEMP:
        data->osr_temp = reg_val;
        break;

    default:
        break;
    }

    return err;
}

static int bmp280_attr_set_iir_filter(const struct device *dev, int32_t val)
{
    uint8_t reg_val = 0;
    int err;
    struct bmp280_data *data = dev->data;

    /* Value must be a positive power of 2 <= 16 or 0, and not 1. */
    if ((val < 0) || (val == 1) || (val > 16) || ((val & (val - 1)) != 0)) {
        return -EINVAL;
    }

    /* Determine exponent: this corresponds to register setting. */
    if (val > 0) {
        while ((val % 2) == 0) {
            val >>= 1;
            ++reg_val;
        }
    }

    err = bmp280_reg_field_update(dev,
                      BMP280_REG_CONFIG,
                      BMP280_CONFIG_FILTER_MASK,
                      reg_val << BMP280_CONFIG_FILTER_POS);
    if (err < 0) {
        return err;
    }

    data->iir_filter = reg_val;

    return err;
}

static int bmp280_attr_set_time_sb(const struct device *dev, int32_t val)
{
    uint8_t reg_val = 0;
    int err;
    struct bmp280_data *data = dev->data;

    //* Value must be 5, or a power of two multiplied by 625 between 625 and 40000. */
    if ((val != 5) && ((val < 625) || (val > 40000) || (val%625 != 0) || ((val/625 & (val/625 - 1)) != 0))) {
        return -EINVAL;
    }

    //* Determine exponent: this corresponds to register setting. */
    if (val != 5) {
        val /= 625;
        ++reg_val;
        while ((val % 2) == 0) {
            val >>= 1;
            ++reg_val;
        }
    }

    err = bmp280_reg_field_update(dev,
                      BMP280_REG_CONFIG,
                      BMP280_CONFIG_TIME_SB_MASK,
                      reg_val << BMP280_CONFIG_TIME_SB_POS);
    if (err < 0) {
        return err;
    }

    data->time_sb = reg_val;

    return err;
}

static int bmp280_attr_set(const struct device *dev,
               enum sensor_channel chan,
               enum sensor_attribute attr,
               const struct sensor_value *val)
{
    int ret;

#ifdef CONFIG_PM_DEVICE
    enum pm_device_state state;

    (void)pm_device_state_get(dev, &state);
    if (state != PM_DEVICE_STATE_ACTIVE) {
        return -EBUSY;
    }
#endif

    switch ((int)attr) {
    case SENSOR_ATTR_OVERSAMPLING:
        ret = bmp280_attr_set_oversampling(dev, chan, val->val1);
        break;

    case BMP280_ATTR_IIR_FILTER:
        ret = bmp280_attr_set_iir_filter(dev, val->val1);
        break;

    case BMP280_ATTR_TIME_SB:
        ret = bmp280_attr_set_time_sb(dev, val->val1);
        break;

    default:
        ret = -EINVAL;
    }

    return ret;
}

static int bmp280_wait_until_ready(const struct device *dev)
{
    uint8_t status = 0;
    int ret;

    /* Wait for NVM to copy and and measurement to be completed */
    do {
        k_sleep(K_MSEC(3));
        ret = bmp280_reg_read(dev, BMP280_REG_STATUS, &status, 1);
        if (ret < 0) {
            return ret;
        }
    } while (status & (BMP280_STATUS_MEASURING | BMP280_STATUS_UPDATE));

    return 0;
}

static int bmp280_sample_fetch(const struct device *dev,
                   enum sensor_channel chan)
{
    struct bmp280_data *bmp280 = dev->data;
    uint8_t raw[BMP280_SAMPLE_BUFFER_SIZE];
    int ret = 0;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

#ifdef CONFIG_PM_DEVICE
    enum pm_device_state state;

    (void)pm_device_state_get(dev, &state);
    if (state != PM_DEVICE_STATE_ACTIVE) {
        return -EBUSY;
    }
#endif

    pm_device_busy_set(dev);

    ret = bmp280_reg_read(dev,
              BMP280_REG_PRESS_MSB,
              raw,
              BMP280_SAMPLE_BUFFER_SIZE);
    if (ret < 0) {
        LOG_DBG("Failed sample fetch.");
        goto error;
    }

    /* convert samples to 20bit values */
    bmp280->sample.press_raw = 
                   (raw[0] << 12) |
                   (raw[1] << 4) |
                   (raw[2] >> 4);
    bmp280->sample.temp_raw = 
                   (raw[3] << 12) |
                   (raw[4] << 4) |
                   (raw[5] >> 4);

    bmp280->sample.status = 0;
    pm_device_busy_clear(dev);
    return 0;

error:
    bmp280->sample.status = 0x4;
    pm_device_busy_clear(dev);
    return ret;
}

static void bmp280_compensate_temp(struct bmp280_data *data)
{
    /* Adapted from:
     * https://github.com/boschsensortec/BMP2_SensorAPI/blob/master/bmp2.c
     */

    int32_t var1, var2, temp;
    struct bmp280_cal_data *cal = &data->cal;
    struct bmp280_sample *sample = &data->sample;

    var1 = (((sample->temp_raw >> 3) - ((int32_t)cal->t1 << 1))) * ((int32_t)cal->t2) >> 11;
    var2 = (((((sample->temp_raw >> 4) - ((int32_t)cal->t1)) * ((sample->temp_raw >> 4) - ((int32_t)cal->t1))) >> 12) * ((int32_t)cal->t3)) >> 14;
    temp = ((var1 + var2) * 5 + 128) >> 8;
    
    if (temp > BMP280_MAX_TEMP_INT) {
        temp = BMP280_MAX_TEMP_INT;
    }

    if (temp < BMP280_MIN_TEMP_INT) {
        temp = BMP280_MIN_TEMP_INT;
    }

    sample->t_fine = var1 + var2;
    
    /* From the datasheet, p22:
     * Returns temperature in DegC, resolution is 0.01 DegC. 
     * Output value of "5123" equals 51.23 DegC. */
    sample->temp.val1 = temp / 100;
    sample->temp.val2 = (temp % 100) * 10000;
    sample->status |= 0x1;
}

static int bmp280_temp_channel_get(const struct device *dev,
                   struct sensor_value *val)
{
    struct bmp280_data *data = dev->data;
    struct bmp280_sample *sample = &data->sample;

    if ((sample->status & 0x4) != 0) {
        return -EINVAL;
    }

    if ((sample->status & 0x1) == 0) {
        bmp280_compensate_temp(data);
    }

    val->val1 = sample->temp.val1;
    val->val2 = sample->temp.val2;

    return 0;
}

static void bmp280_compensate_press(struct bmp280_data *data)
{
    /* Adapted from:
     * https://github.com/boschsensortec/BMP2_SensorAPI/blob/master/bmp2.c
     */

    struct bmp280_cal_data *cal = &data->cal;
    struct bmp280_sample *sample = &data->sample;

    int64_t var1, var2, press;

    var1 = ((int64_t)sample->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)cal->p6;
    var2 = var2 + ((var1 * (int64_t)cal->p5) << 17);
    var2 = var2 + (((int64_t)cal->p4) << 35);
    var1 = ((var1 * var1 * (int64_t)cal->p3) >> 8) + ((var1 * (int64_t)cal->p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)cal->p1) >> 33;

    /* Avoid exception caused by division by zero. */
    if (var1 == 0) {
        LOG_DBG("Invalid pressure converion.");
        sample->status |= 0x4;
        return;
    }

    press = 1048576 - sample->press_raw;
    press = (((press << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)cal->p9) * (press >> 13) * (press >> 13)) >> 25;
    var2 = (((int64_t)cal->p8) * press) >> 19;
    press = ((press + var1 + var2) >> 8) + (((int64_t)cal->p7) << 4);

    if (press > BMP280_MAX_PRESS_64INT) {
        press = BMP280_MAX_PRESS_64INT;
    }

    if (press < BMP280_MIN_PRESS_64INT) {
        press = BMP280_MIN_PRESS_64INT;
    }

    /* From the datasheet, p22:
     * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits). 
     * Output value of "24674867" represents 24674867/256 = 96386.2 Pa. */
    sample->press.val1 = (((uint32_t)press) >> 8) / 1000;
    sample->press.val2 = ((((uint32_t)press) >> 8) % 1000) * 1000 + (((((uint32_t)press) & 0xff) * 1000) >> 8);
    sample->status |= 0x2;
}

static int bmp280_press_channel_get(const struct device *dev,
                    struct sensor_value *val)
{
    struct bmp280_data *data = dev->data;
    struct bmp280_sample *sample = &data->sample;

    if ((sample->status & 0x4) != 0) {
        return -EINVAL;
    }

    if ((sample->status & 0x1) == 0) {
        bmp280_compensate_temp(data);
    }

    if ((sample->status & 0x2) == 0) {
        bmp280_compensate_press(data);
    }
    
    if ((sample->status & 0x4) != 0) {
        return -EINVAL;
    }
    
    val->val1 = sample->press.val1;
    val->val2 = sample->press.val2;

    return 0;
}

static int bmp280_channel_get(const struct device *dev,
                  enum sensor_channel chan,
                  struct sensor_value *val)
{
    switch (chan) {
    case SENSOR_CHAN_PRESS:
        bmp280_press_channel_get(dev, val);
        break;

    case SENSOR_CHAN_AMBIENT_TEMP:
        bmp280_temp_channel_get(dev, val);
        break;

    default:
        LOG_DBG("Channel not supported.");
        return -ENOTSUP;
    }

    return 0;
}

static int bmp280_get_calibration_data(const struct device *dev)
{
    struct bmp280_data *data = dev->data;
    struct bmp280_cal_data *cal = &data->cal;
    uint16_t buf[12];
    
    if (bmp280_reg_read(dev, BMP280_REG_DIG_T1, (uint8_t*)buf, sizeof(buf)) < 0) {
        return -EIO;
    }

    cal->t1 = sys_le16_to_cpu(buf[0]);
    cal->t2 = sys_le16_to_cpu(buf[1]);
    cal->t3 = sys_le16_to_cpu(buf[2]);
    cal->p1 = sys_le16_to_cpu(buf[3]);
    cal->p2 = sys_le16_to_cpu(buf[4]);
    cal->p3 = sys_le16_to_cpu(buf[5]);
    cal->p4 = sys_le16_to_cpu(buf[6]);
    cal->p5 = sys_le16_to_cpu(buf[7]);
    cal->p6 = sys_le16_to_cpu(buf[8]);
    cal->p7 = sys_le16_to_cpu(buf[9]);
    cal->p8 = sys_le16_to_cpu(buf[10]);
    cal->p9 = sys_le16_to_cpu(buf[11]);

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int bmp280_pm_action(const struct device *dev,
                enum pm_device_action action)
{
    uint8_t reg_val;

    switch (action) {
    case PM_DEVICE_ACTION_RESUME:
        reg_val = BMP280_CTRL_MEAS_MODE_NORMAL;
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        reg_val = BMP280_CTRL_MEAS_MODE_SLEEP;
        break;
    default:
        return -ENOTSUP;
    }

    if (bmp280_reg_field_update(dev,
                    BMP280_REG_CTRL_MEAS,
                    BMP280_CTRL_MEAS_MODE_MASK,
                    reg_val << BMP280_CTRL_MEAS_MODE_POS) < 0) {
        LOG_DBG("Failed to set power mode.");
        return -EIO;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct sensor_driver_api bmp280_api = {
    .attr_set = bmp280_attr_set,
    .sample_fetch = bmp280_sample_fetch,
    .channel_get = bmp280_channel_get,
};

static int bmp280_init(const struct device *dev)
{
    struct bmp280_data *data = dev->data;
    const struct bmp280_config *cfg = dev->config;
    uint8_t val = 0U;

    if (bmp280_bus_check(dev) < 0) {
        LOG_DBG("bus check failed");
        return -ENODEV;
    }

//#ifdef CONFIG_BMP280_REBOOT_ON_INIT
    /* Soft reboot the chip */
    if (bmp280_reg_write(dev, BMP280_REG_SOFT_RESET, BMP280_SOFT_RESET_CMD) < 0) {
        LOG_ERR("Cannot reboot chip.");
        return -EIO;
    }

    if (bmp280_wait_until_ready(dev) < 0) {
        LOG_ERR("Failed waiting to get ready.");        
        return -EIO;
    }
//#else
    ///* Transition to sleep mode */
    //if (bmp280_reg_field_update(dev,
                    //BMP280_REG_CTRL_MEAS,
                    //BMP280_CTRL_MEAS_MODE_MASK,
                    //BMP280_CTRL_MEAS_MODE_SLEEP << BMP280_CTRL_MEAS_MODE_POS) < 0) {
        //LOG_DBG("Failed to set sleep mode.");
        //return -EIO;
    //}
//#endif

    if (bmp280_reg_read(dev, BMP280_REG_CHIPID, &val, 1) < 0) {
        LOG_ERR("Failed to read chip id.");
        return -EIO;
    }

    if ((val != BMP280_CHIP_ID) && (val != BME280_CHIP_ID)) {
        LOG_ERR("Unsupported chip detected (0x%x)!", val);
        return -ENODEV;
    }

    /* Read calibration data */
    if (bmp280_get_calibration_data(dev) < 0) {
        LOG_ERR("Failed to read calibration data.");
        return -EIO;
    }

    /* Set oversampling mode */
    val = (data->osr_temp << BMP280_CTRL_MEAS_OSRS_T_POS) & BMP280_CTRL_MEAS_OSRS_T_MASK;
    val |= (data->osr_press << BMP280_CTRL_MEAS_OSRS_P_POS) & BMP280_CTRL_MEAS_OSRS_P_MASK;
    if (bmp280_reg_field_update(dev,
                    BMP280_REG_CTRL_MEAS,
                    (BMP280_CTRL_MEAS_OSRS_T_MASK | BMP280_CTRL_MEAS_OSRS_P_MASK),
                    val) < 0) {
        LOG_DBG("Failed to set oversampling mode.");
        return -EIO;
    }

    /* Set standby, IIR filter coefficient and SPI 3-wire mode */
    val = (data->iir_filter << BMP280_CONFIG_FILTER_POS) & BMP280_CONFIG_FILTER_MASK;
    val |= (data->time_sb << BMP280_CONFIG_TIME_SB_POS) & BMP280_CONFIG_TIME_SB_MASK;
    val |= (cfg->spi3w_en << BMP280_CONFIG_SPI3W_EN_POS) & BMP280_CONFIG_SPI3W_EN_MASK;
    if (bmp280_reg_write(dev, BMP280_REG_CONFIG, val) < 0) {
        LOG_ERR("Failed to set standby, IIR filter coefficient and SPI 3-wire mode.");
        return -EIO;
    }

    /* Transition to normal mode */
    if (bmp280_reg_field_update(dev,
                    BMP280_REG_CTRL_MEAS,
                    BMP280_CTRL_MEAS_MODE_MASK,
                    BMP280_CTRL_MEAS_MODE_NORMAL << BMP280_CTRL_MEAS_MODE_POS) < 0) {
        LOG_DBG("Failed to set normal mode.");
        return -EIO;
    }

    return 0;
}

/* Initializes a struct bmp280_config for an instance on a SPI bus. */
#define BMP280_CONFIG_SPI(inst)             \
    .bus.spi = SPI_DT_SPEC_INST_GET(inst, BMP280_SPI_OPERATION, 0), \
    .bus_io = &bmp280_bus_io_spi,

/* Initializes a struct bmp280_config for an instance on an I2C bus. */
#define BMP280_CONFIG_I2C(inst)                \
    .bus.i2c = I2C_DT_SPEC_INST_GET(inst),         \
    .bus_io = &bmp280_bus_io_i2c,

#define BMP280_BUS_CFG(inst)            \
    COND_CODE_1(DT_INST_ON_BUS(inst, i2c),  \
            (BMP280_CONFIG_I2C(inst)),  \
            (BMP280_CONFIG_SPI(inst)))

#define BMP280_INST(inst)                          \
    static struct bmp280_data bmp280_data_##inst = {           \
        .osr_temp = DT_INST_ENUM_IDX(inst, osr_temp),          \
        .osr_press = DT_INST_ENUM_IDX(inst, osr_press),     \
        .iir_filter = DT_INST_ENUM_IDX(inst, iir_filter),      \
        .time_sb = DT_INST_ENUM_IDX(inst, time_sb),      \
    };                                 \
    static const struct bmp280_config bmp280_config_##inst = {     \
        BMP280_BUS_CFG(inst)                       \
        .spi3w_en = DT_INST_PROP(inst, spi3w_en),      \
    };                                 \
    PM_DEVICE_DT_INST_DEFINE(inst, bmp280_pm_action);          \
    SENSOR_DEVICE_DT_INST_DEFINE(                      \
        inst,                              \
        bmp280_init,                           \
        PM_DEVICE_DT_INST_GET(inst),                   \
        &bmp280_data_##inst,                       \
        &bmp280_config_##inst,                     \
        POST_KERNEL,                           \
        CONFIG_SENSOR_INIT_PRIORITY,                   \
        &bmp280_api);

DT_INST_FOREACH_STATUS_OKAY(BMP280_INST)
