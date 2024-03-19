#ifndef DRIVERS_ZEPHYR_INCLUDE_SGP40_AIR_QUALITY_H_
#define DRIVERS_ZEPHYR_INCLUDE_SGP40_AIR_QUALITY_H_

#include <zephyr/drivers/i2c.h>

#define SGP40_INSTANCE_NUMBER 0
#define SGP40_SERIAL_NUMBER   81899109 // 0x000004e1ae65
#define SGP40_I2C_ADDRESS     0x59 // Ch. 4.4 datasheet
#define SGP40_TEST_WAIT_MS    320 // Ch. 3.3 datasheet
#define SGP40_MEASURE_WAIT_MS 30
#define SGP40_READ_WAIT_MS    1

/* Self test */
#define SPG40_SELF_TEST_PASSED       0xD4
#define SPG40_SELF_TEST_FAILED       0x4B
#define SPG40_SELF_TEST_FAILED_ERROR 99

#define SGP40_CMD_MEASURE_RAW_SIGNAL_H 0x26
#define SGP40_CMD_MEASURE_RAW_SIGNAL_L 0x0F
#define SGP40_CMD_EXECTUE_SELF_TEST_H  0x28
#define SGP40_CMD_EXECTUE_SELF_TEST_L  0x0E
#define SGP40_CMD_TURN_HEATER_OFF_H    0x36
#define SGP40_CMD_TURN_HEATER_OFF_L    0x15
#define SGP40_CMD_GET_SERIAL_NUMBER_H  0x36
#define SGP40_CMD_GET_SERIAL_NUMBER_L  0x82
#define SGP40_CMD_SOFT_RESET_H         0x00
#define SGP40_CMD_SOFT_RESET_L         0x06

/*
 * CRC parameters were taken from the
 * "Checksum Calculation" section of the datasheet.
 */
#define SGP40_CRC_POLY 0x31
#define SGP40_CRC_INIT 0xFF

/**
 * @brief  Set the temperature and humidity
 * @param  relative_humidity  Current environmental relative humidity value, range 0-100, unit: %RH
 * @param  temperature_in_celcius  Current ambient temperature, range -10~50, unit: °C
 */
void sgp40_set_rht(const struct device* dev, float relative_humidity, float temperature_in_celcius); // TODO: Currently using default rh = 50, T = 25

struct sgp40_config {
    struct i2c_dt_spec i2c;
    uint8_t            temperature[3];
    uint8_t            humidity[3];
};

struct sgp40_data {
    uint16_t raw_signal_in_ticks;
};

#endif // DRIVERS_ZEPHYR_INCLUDE_SGP40_AIR_QUALITY_H_