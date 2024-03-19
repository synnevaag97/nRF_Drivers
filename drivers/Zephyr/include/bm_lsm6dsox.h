#ifndef DRIVERS_ZEPHYR_INCLUDE_BM_LSM6DSOX_H_
#define DRIVERS_ZEPHYR_INCLUDE_BM_LSM6DSOX_H_

#include <zephyr/drivers/i2c.h>

#define BIT(n)      (1UL << (n))
#define BIT_ZERO(n) (0 << (n)) // Just used for readibility

// Instance number and WHO_AM_I
#define LSM6DSOX_INSTANCE_NUMBER 0
#define LSM6DSOX_REG_WHO_AM_I    0x0F
#define LSM6DSOX_VAL_WHO_AM_I    0x6C

// Conversions of data
#define LSM6DSOX_G           (SENSOR_G / 1000000.0)
#define LSM6DSOX_DEG_TO_RAD  (3.14159265359 / 180.0)
#define LSM6DSOX_MILLI_FLOAT (1 / 1000.0)
#define LSM6DSOX_MEGA_INT    1000000

// Sensitivities as per datasheet (Table 2 on p.10)
#define LSM6DSOX_SENSITIVITY_ACC_FS_2_G     0.061 // Linear acceleration sensitivity is 0.061 (in mg/LSB) since FS=+-2 g --> FS[1:0]_XL = 00 in CTRL1_XL (10h)
#define LSM6DSOX_SENSITIVITY_ACC_FS_250_dps 8.75 // Angular rate sensitivity is 8.75 (in mdps/LSB) since FS=+-250 dps --> FS[1:0]_G = 00 and FS_125 = 0 in CTRL2_G (11h)

// Reboot register
#define LSM6DSOX_REG_CTRL_REG3_C        0x12
#define LSM6DSOX_MASK_CTRL_REG3_C_BOOT  BIT(7)
#define LSM6DSOX_SHIFT_CTRL_REG3_C_BOOT BIT(7)

// Out data
#define LSM6DSOX_REG_OUT_X_L_A 0x28 // Accelerometer
#define LSM6DSOX_REG_OUT_X_L_G 0x22 // Gyroscope

// Accelerometer control register 1
#define LSM6DSOX_REG_CTRL_REG1_XL          0x10
#define LSM6DSOX_MASK_CTRL_REG1_XL_ODR_XL  (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define LSM6DSOX_SHIFT_CTRL_REG1_XL_ODR_XL BIT(4) // ODR_XL0 =1 : 12.5 Hz

// Gyroscope control register 2
#define LSM6DSOX_REG_CTRL_REG2_G         0x11
#define LSM6DSOX_MASK_CTRL_REG2_G_ODR_G  (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define LSM6DSOX_SHIFT_CTRL_REG2_G_ODR_G BIT(4) // ODR_G0 =1 : 12.5 Hz

// Accelerometer power mode set (ctrl reg 5) : ultra-low-power mode disables
#define LSM6DSOX_REG_CTRL_REG5_C             0x14
#define LSM6DSOX_MASK_CTRL_REG5_C_XL_ULP_EN  BIT(7) // XL_ULP_EN in [XL_ULP_EN,...,...]
#define LSM6DSOX_SHIFT_CTRL_REG5_C_XL_ULP_EN BIT_ZERO(7) // XL_ULP_EN = 0 : Accelerometer ultra-low-power mode disables

// Accelerometer power mode set (ctrl reg 6) : high performance, not low power
#define LSM6DSOX_REG_CTRL_REG6_C              0x15
#define LSM6DSOX_MASK_CTRL_REG6_C_XL_HM_MODE  BIT(4) // XL_HM_MODE in [TRIG_EN, LVL1_EN, LVL2_EN, XL_HM_MODE, USR_OFF_W, FTYPE_2, FTYPE_1, FTYPE_0]
#define LSM6DSOX_SHIFT_CTRL_REG6_C_XL_HM_MODE BIT_ZERO(4) // XL_HM_MODE = 0 : High performance

// Gyroscope power mode set (ctrl reg 7) : high-performance operating mode enabled
#define LSM6DSOX_REG_CTRL_REG7_G                  0x16
#define LSM6DSOX_MASK_CTRL_REG7_G_G_HM_MODE       BIT(7) // G_HM_MODE in [G_HM_MODE, ... , ...]
#define LSM6DSOX_SHIFT_CTRL_CTRL_REG7_G_G_HM_MODE BIT_ZERO(7) // G_HM_MODE = 0 : High performance

struct lsm6dsox_data {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

struct lsm6dsox_config {
    struct i2c_dt_spec i2c;
};

#endif // DRIVERS_ZEPHYR_INCLUDE_BM_LSM6DSOX_H_