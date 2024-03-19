#define DT_DRV_COMPAT st_bm_lsm6dsox

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bm_lsm6dsox.h"

LOG_MODULE_REGISTER(bm_lsm6dsox, CONFIG_BM_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static inline int lsm6dsox_reboot(const struct device* dev)
{
    const struct lsm6dsox_config* config = dev->config;

    if (i2c_reg_update_byte_dt(&config->i2c, LSM6DSOX_REG_CTRL_REG3_C,
            LSM6DSOX_MASK_CTRL_REG3_C_BOOT,
            LSM6DSOX_SHIFT_CTRL_REG3_C_BOOT)
        < 0) {
        return -EIO;
    }

    // Wait sensor turn-on time as per datasheet (Ton in Table 3 on p.13)
    k_msleep(35);

    return 0;
}

static inline int lsm6dsox_accel_set_ctrl_regs(const struct device* dev)
{
    const struct lsm6dsox_config* config = dev->config;

    // Accelerometer power mode set (ctrl reg 5) : ultra-low-power mode disables
    if (i2c_reg_update_byte_dt(&config->i2c, LSM6DSOX_REG_CTRL_REG5_C,
            LSM6DSOX_MASK_CTRL_REG5_C_XL_ULP_EN,
            LSM6DSOX_SHIFT_CTRL_REG5_C_XL_ULP_EN)
        < 0) {
        LOG_ERR("failed to set ctrl 5 - acc");
        return -EIO;
    }

    // Accelerometer power mode set (ctrl reg 6) : High performance, not low power
    if (i2c_reg_update_byte_dt(&config->i2c, LSM6DSOX_REG_CTRL_REG6_C,
            LSM6DSOX_MASK_CTRL_REG6_C_XL_HM_MODE,
            LSM6DSOX_SHIFT_CTRL_REG6_C_XL_HM_MODE)
        < 0) {
        LOG_ERR("failed to set ctrl 6 - acc");
        return -EIO;
    }

    // Something about accelerometer range here maybe ?? It is default +- 2g which is definitely enough

    // Accelerometer data rate selection/ODR raw (ctrl reg 1): 12.5 Hz
    if (i2c_reg_update_byte_dt(&config->i2c, LSM6DSOX_REG_CTRL_REG1_XL,
            LSM6DSOX_MASK_CTRL_REG1_XL_ODR_XL,
            LSM6DSOX_SHIFT_CTRL_REG1_XL_ODR_XL)
        < 0) {
        LOG_ERR("failed to set ctrl 1 - acc");
        return -EIO;
    }

    return 0;
}

static inline int lsm6dsox_gyro_set_ctrl_regs(const struct device* dev)
{
    const struct lsm6dsox_config* config = dev->config;

    // Power mode set (ctrl reg 7) : high-performance operating mode for gyroscope enabled
    if (i2c_reg_update_byte_dt(&config->i2c, LSM6DSOX_REG_CTRL_REG7_G,
            LSM6DSOX_MASK_CTRL_REG7_G_G_HM_MODE,
            LSM6DSOX_SHIFT_CTRL_CTRL_REG7_G_G_HM_MODE)
        < 0) {
        LOG_ERR("failed to set ctrl 7 - gyro");
        return -EIO;
    }

    // Something about gyroscope range here maybe ?? It is default +-250 dps which is definitely enough

    // Data rate selection/ODR raw (ctrl reg 2): 12.5 Hz
    if (i2c_reg_update_byte_dt(&config->i2c, LSM6DSOX_REG_CTRL_REG2_G,
            LSM6DSOX_MASK_CTRL_REG2_G_ODR_G,
            LSM6DSOX_SHIFT_CTRL_REG2_G_ODR_G)
        < 0) {
        LOG_ERR("failed to set ctrl 2 - gyro");
        return -EIO;
    }

    return 0;
}

// Initialize the registers on the chip
static int lsm6dsox_init_chip(const struct device* dev)
{
    const struct lsm6dsox_config* config = dev->config;
    uint8_t                       chip_id;

    if (lsm6dsox_reboot(dev) < 0) {
        LOG_ERR("failed to reboot device");
        return -EIO;
    }

    if (i2c_reg_read_byte_dt(&config->i2c, LSM6DSOX_REG_WHO_AM_I, &chip_id) < 0) {
        LOG_ERR("failed reading chip id");
        return -EIO;
    }
    if (chip_id != LSM6DSOX_VAL_WHO_AM_I) {
        LOG_ERR("Wrong register value");
        return -EIO;
    }

    if (lsm6dsox_accel_set_ctrl_regs(dev) < 0) {
        LOG_ERR("failed to set control registers for accelerometer");
        return -EIO;
    }

    if (lsm6dsox_gyro_set_ctrl_regs(dev) < 0) {
        LOG_ERR("failed to set control registers for gyroscope");
        return -EIO;
    }
    return 0;
}

static int lsm6dsox_init(const struct device* dev)
{
    const struct lsm6dsox_config* config = dev->config;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device not ready");
        return -ENODEV;
    }

    // Initialize the registers on the chip
    if (lsm6dsox_init_chip(dev) < 0) {
        LOG_ERR("failed to initialize chip");
        return -EIO;
    }
    LOG_INF("%s ready", dev->name);
    return 0;
}

static int lsm6dsox_sample_fetch_accel(const struct device* dev)
{
    struct lsm6dsox_data*         data   = dev->data;
    const struct lsm6dsox_config* config = dev->config;
    uint8_t                       buf[6];

    if (i2c_burst_read_dt(&config->i2c, LSM6DSOX_REG_OUT_X_L_A, buf, sizeof(buf)) < 0) {
        LOG_ERR("failed to read sample");
        return -EIO;
    }

    // Linear acceleration data (raw get)
    data->acc_x = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));
    data->acc_y = (int16_t)((uint16_t)(buf[2]) | ((uint16_t)(buf[3]) << 8));
    data->acc_z = (int16_t)((uint16_t)(buf[4]) | ((uint16_t)(buf[5]) << 8));
    return 0;
}

static int lsm6dsox_sample_fetch_gyro(const struct device* dev)
{
    struct lsm6dsox_data*         data   = dev->data;
    const struct lsm6dsox_config* config = dev->config;
    uint8_t                       buf[6];

    if (i2c_burst_read_dt(&config->i2c, LSM6DSOX_REG_OUT_X_L_G, buf, sizeof(buf)) < 0) {
        LOG_ERR("failed to read sample");
        return -EIO;
    }

    // Angular rate data (raw get)
    data->gyro_x = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));
    data->gyro_y = (int16_t)((uint16_t)(buf[2]) | ((uint16_t)(buf[3]) << 8));
    data->gyro_z = (int16_t)((uint16_t)(buf[4]) | ((uint16_t)(buf[5]) << 8));
    return 0;
}

static int lsm6dsox_sample_fetch(const struct device* dev, enum sensor_channel chan)
{
    switch (chan) {
    case SENSOR_CHAN_ACCEL_XYZ:
        lsm6dsox_sample_fetch_accel(dev);
        break;
    case SENSOR_CHAN_GYRO_XYZ:
        lsm6dsox_sample_fetch_gyro(dev);
        break;
    case SENSOR_CHAN_ALL:
        lsm6dsox_sample_fetch_accel(dev);
        lsm6dsox_sample_fetch_gyro(dev);
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static void lsm6dsox_accel_convert(struct sensor_value* val, int16_t raw_val)
{
    double acc; // ms^-2
    acc       = (double)(raw_val) * (double)(LSM6DSOX_SENSITIVITY_ACC_FS_2_G * LSM6DSOX_G * LSM6DSOX_MILLI_FLOAT); // ms^-2 = LSB * mg/LSB * ms^-2/g * g/mg
    val->val1 = (int32_t)acc;
    val->val2 = ((int32_t)(acc * LSM6DSOX_MEGA_INT)) % LSM6DSOX_MEGA_INT;
}

static void lsm6dsox_gyro_convert(struct sensor_value* val, int16_t raw_val)
{
    double rps; // rad/s
    // TODO: Validate these conversions/mappings
    rps       = (double)(raw_val) * (double)(LSM6DSOX_SENSITIVITY_ACC_FS_250_dps * LSM6DSOX_DEG_TO_RAD * LSM6DSOX_MILLI_FLOAT); // rad/s = LSB *  millidegrees s^-1/LSB  * radian/degree * degree/millidegree
    val->val1 = (int32_t)rps;
    val->val2 = ((int32_t)(rps * LSM6DSOX_MEGA_INT)) % LSM6DSOX_MEGA_INT;
}

static int lsm6dsox_channel_get(const struct device* dev, enum sensor_channel chan, struct sensor_value* val)
{
    struct lsm6dsox_data* data = dev->data;
    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        lsm6dsox_accel_convert(val, data->acc_x);
        break;
    case SENSOR_CHAN_ACCEL_Y:
        lsm6dsox_accel_convert(val, data->acc_y);
        break;
    case SENSOR_CHAN_ACCEL_Z:
        lsm6dsox_accel_convert(val, data->acc_z);
        break;
    case SENSOR_CHAN_ACCEL_XYZ:
        lsm6dsox_accel_convert(val, data->acc_x);
        lsm6dsox_accel_convert(val + 1, data->acc_y);
        lsm6dsox_accel_convert(val + 2, data->acc_z);
        break;
    case SENSOR_CHAN_GYRO_X:
        lsm6dsox_gyro_convert(val, data->gyro_x);
        break;
    case SENSOR_CHAN_GYRO_Y:
        lsm6dsox_gyro_convert(val, data->gyro_y);
        break;
    case SENSOR_CHAN_GYRO_Z:
        lsm6dsox_gyro_convert(val, data->gyro_z);
        break;
    case SENSOR_CHAN_GYRO_XYZ:
        lsm6dsox_gyro_convert(val, data->gyro_x);
        lsm6dsox_gyro_convert(val + 1, data->gyro_y);
        lsm6dsox_gyro_convert(val + 2, data->gyro_z);
        break;
    case SENSOR_CHAN_ALL:
        lsm6dsox_accel_convert(val, data->acc_x);
        lsm6dsox_accel_convert(val + 1, data->acc_y);
        lsm6dsox_accel_convert(val + 2, data->acc_z);
        lsm6dsox_gyro_convert(val + 3, data->gyro_x);
        lsm6dsox_gyro_convert(val + 4, data->gyro_y);
        lsm6dsox_gyro_convert(val + 5, data->gyro_z);
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static struct lsm6dsox_data   bm_lsm6dsox_data;
static struct lsm6dsox_config bm_lsm6dsox_config = {
    .i2c = I2C_DT_SPEC_INST_GET(LSM6DSOX_INSTANCE_NUMBER),
};

static const struct sensor_driver_api lsm6dsox_api_funcs = {
    .sample_fetch = lsm6dsox_sample_fetch,
    .channel_get  = lsm6dsox_channel_get,
};

SENSOR_DEVICE_DT_INST_DEFINE(LSM6DSOX_INSTANCE_NUMBER,
    &lsm6dsox_init,
    NULL,
    &bm_lsm6dsox_data,
    &bm_lsm6dsox_config,
    APPLICATION,
    CONFIG_SENSOR_INIT_PRIORITY,
    &lsm6dsox_api_funcs);

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)