#define DT_DRV_COMPAT sensirion_bm_sgp40 // Checks the compatible string that should match the device in the device tree

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "sgp40_air_quality.h"

LOG_MODULE_REGISTER(bm_spg40, CONFIG_BM_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#ifdef CONFIG_CRC
static uint8_t sgp40_compute_crc(uint8_t* value)
{
    return crc8(value, 2, SGP40_CRC_POLY, SGP40_CRC_INIT, false);
}
#endif /* CONFIG_CRC */

/**
 * @brief  Write commands over I2C
 * @param  Command  An SGP_40 command.
 */
static int sgp40_write_cmd(const struct device* dev, uint16_t cmd)
{
    const struct sgp40_config* config = dev->config;
    uint8_t                    tx_buf[2];

    tx_buf[0] = (uint8_t)(cmd >> 8);
    tx_buf[1] = (uint8_t)(cmd);
    int ret   = i2c_write_dt(&config->i2c, tx_buf, sizeof(tx_buf));
    return ret;
}

/**
 * @brief  Soft Reset, the SGP40 restarts and enter the idle mode.
 * @note TODO: Currently not working correctly.
 * @note This command is a general call resetting all devices connected to
            the same I2C bus. The first byte refers to the general call address
            and the second byte refers to the reset command. After calling
            this command, the SGP40 will restart entering the idle mode.
 */
static int sgp40_soft_reset(const struct device* dev)
{
    uint16_t cmd = ((uint16_t)SGP40_CMD_SOFT_RESET_H << 8) | ((uint16_t)SGP40_CMD_SOFT_RESET_L);

    if (sgp40_write_cmd(dev, cmd)) {
        LOG_ERR("Sensor failed to initiate reset.");
        return -EIO;
    }
    return 0;
}

/**
 * @brief  Read the sensor serial number.
 * @note Total 9 bytes serial number. Byte 2,5,8 CRC for the two preceding bytes.
 */
static int sgp40_get_serial_number(const struct device* dev)
{
    const struct sgp40_config* config = dev->config;

    uint16_t cmd = ((uint16_t)SGP40_CMD_GET_SERIAL_NUMBER_H << 8) | ((uint16_t)SGP40_CMD_GET_SERIAL_NUMBER_L);
    if (sgp40_write_cmd(dev, cmd)) {
        LOG_ERR("Failed to send read serial number cmd.");
        return -EIO;
    }

    k_msleep(SGP40_READ_WAIT_MS);

    uint8_t rx_buf[9];
    if (i2c_read_dt(&config->i2c, rx_buf, sizeof(rx_buf))) {
        LOG_ERR("Failed to read serial number.");
        return -EIO;
    }

    if (sgp40_compute_crc(&rx_buf[0]) != rx_buf[2]) {
        LOG_ERR("Failed CRC check.");
        return -EIO;
    }
    if (sgp40_compute_crc(&rx_buf[3]) != rx_buf[5]) {
        LOG_ERR("Failed CRC check.");
        return -EIO;
    }
    if (sgp40_compute_crc(&rx_buf[6]) != rx_buf[8]) {
        LOG_ERR("Failed CRC check.");
        return -EIO;
    }

    uint8_t  serial_number_array[6] = { rx_buf[0], rx_buf[1], rx_buf[3], rx_buf[4], rx_buf[6], rx_buf[7] };
    uint64_t serial_number          = sys_get_be48(serial_number_array);
    __ASSERT(serial_number == SGP40_SERIAL_NUMBER, "Serial number not a match.");
    return 0;
}

/**
 * @brief  Sensor self-test
 * @note Testing integrity of both hotplates.
 * @note Return 2 bytes, ignore last 8bit of byte one.
 * @note 0xD4XX if all tests passed. 0x4BXX if one or more tests have failed.
 */
static int sgp40_perform_self_test(const struct device* dev)
{
    const struct sgp40_config* config = dev->config;

    uint16_t cmd = ((uint16_t)SGP40_CMD_EXECTUE_SELF_TEST_H << 8) | ((uint16_t)SGP40_CMD_EXECTUE_SELF_TEST_L);
    if (sgp40_write_cmd(dev, cmd)) {
        LOG_ERR("Failed to start self test.");
        return -EIO;
    }

    k_msleep(SGP40_TEST_WAIT_MS);

    uint8_t rx_buf[3];
    if (i2c_read_dt(&config->i2c, rx_buf, sizeof(rx_buf))) {
        LOG_ERR("Failed to read test results.");
        return -EIO;
    }

    if (sgp40_compute_crc(rx_buf) != rx_buf[2]) {
        LOG_ERR("Failed CRC check.");
        return -EIO;
    }

    LOG_HEXDUMP_DBG(rx_buf, sizeof(rx_buf), "Self test results: ");

    if (rx_buf[0] == SPG40_SELF_TEST_PASSED) {
        LOG_DBG("All tests passed successfully.");
    } else if (rx_buf[0] == SPG40_SELF_TEST_FAILED) {
        LOG_ERR("One or more tests have failed.");
        return SPG40_SELF_TEST_FAILED_ERROR;
    }

    return 0;
}

/**
 * @brief  Initialization function
 * @note Reset device, check serial number and perform self test.
 * @note Define default relative humidity and temperature.
 */
static int sgp40_init(const struct device* dev)
{
    const struct sgp40_config* config = dev->config;
    int                        ret    = 0;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device not ready");
        return -ENODEV;
    }

    ret = sgp40_get_serial_number(dev);
    if (ret != 0) {
        return ret;
    }

    ret = sgp40_perform_self_test(dev);
    if (ret != 0) {
        return ret;
    }

    LOG_INF("%s successfully initated", dev->name);
    return 0;
}

/**
 * @brief  Get raw VOC data.
 * @note Reads 3 bytes. Bytes 1 and 2 is u16 unsigned integer directly providing the raw signal SRAW_VOC in ticks. Byte 3 is CRC.
 * @note SRAW_VOC is proportional to the logarithm of the resistance of the sensing element.
 * @return SRAW_VOC
 */
static int sgp40_read_raw_data(const struct device* dev, enum sensor_channel chan)
{
    if (chan != SENSOR_CHAN_GAS_RES) { //&& chan != SENSOR_CHAN_ALL
        return -ENOTSUP;
    }

    const struct sgp40_config* config = dev->config;
    struct sgp40_data*         data   = dev->data;

    uint8_t  tx_buf[8];
    uint16_t cmd = ((uint16_t)SGP40_CMD_MEASURE_RAW_SIGNAL_H << 8) | ((uint16_t)SGP40_CMD_MEASURE_RAW_SIGNAL_L);
    sys_put_be16(cmd, &tx_buf[0]);
    sys_put_be24(sys_get_be24(config->humidity), &tx_buf[2]);
    sys_put_be24(sys_get_be24(config->temperature), &tx_buf[5]);

    if (i2c_write_dt(&config->i2c, tx_buf, sizeof(tx_buf))) {
        LOG_ERR("Failed to send read raw data cmd.");
        return -EIO;
    }

    k_msleep(SGP40_MEASURE_WAIT_MS);

    uint8_t rx_buf[3];
    if (i2c_read_dt(&config->i2c, rx_buf, sizeof(rx_buf))) {
        LOG_ERR("Failed to read raw data from sensor.");
        return -EIO;
    }
    if (sgp40_compute_crc(&rx_buf[0]) != rx_buf[2]) {
        LOG_ERR("Failed CRC check.");
        return -EIO;
    }

    LOG_HEXDUMP_DBG(rx_buf, sizeof(rx_buf), "Raw signal data in ticks: ");

    uint16_t raw_data         = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    data->raw_signal_in_ticks = raw_data;

    LOG_DBG("Successfully fetched data from sensor.");
    return 0;
}

static int sgp40_channel_get(const struct device* dev, enum sensor_channel chan, struct sensor_value* val)
{
    if (chan != SENSOR_CHAN_GAS_RES) {
        return -ENOTSUP;
    }

    const struct sgp40_data* data = dev->data;

    val->val1 = (int32_t)data->raw_signal_in_ticks; // Ticks in range [0, 65'535].
    val->val2 = 0;

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int sgp40_pm_action(const struct device* dev,
    enum pm_device_action                       action)
{
    uint16_t cmd;

    switch (action) {
    case PM_DEVICE_ACTION_RESUME:
        /* activate the hotplate by sending a measure command.  */
        cmd = ((uint16_t)SGP40_CMD_MEASURE_RAW_SIGNAL_H << 8) | ((uint16_t)SGP40_CMD_MEASURE_RAW_SIGNAL_L);
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        /* turns the hotplate off and stops the measurement. Subsequently, the sensor enters the idle mode. */
        cmd = ((uint16_t)SGP40_CMD_TURN_HEATER_OFF_H << 8) | ((uint16_t)SGP40_CMD_TURN_HEATER_OFF_L);
        break;
    default:
        return -ENOTSUP;
    }

    return sgp40_write_cmd(dev, cmd);
}
#endif /* CONFIG_PM_DEVICE */

static struct sgp40_data   bm_sgp40_data;
static struct sgp40_config bm_sgp40_config = {
    .i2c         = I2C_DT_SPEC_INST_GET(SGP40_INSTANCE_NUMBER),
    .temperature = DT_INST_PROP(SGP40_INSTANCE_NUMBER, temperature),
    .humidity    = DT_INST_PROP(SGP40_INSTANCE_NUMBER, relative_humidity),
};

static const struct sensor_driver_api sgp40_api_funcs = {
    .sample_fetch = sgp40_read_raw_data,
    .channel_get  = sgp40_channel_get,
};

// Adds function for power managment (PM)
PM_DEVICE_DT_DEFINE(bm_sgp40, sgp40_pm_action);

SENSOR_DEVICE_DT_INST_DEFINE(SGP40_INSTANCE_NUMBER,
    &sgp40_init,
    PM_DEVICE_DT_GET(bm_sgp40),
    &bm_sgp40_data,
    &bm_sgp40_config,
    POST_KERNEL,
    CONFIG_SENSOR_INIT_PRIORITY,
    &sgp40_api_funcs);

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)