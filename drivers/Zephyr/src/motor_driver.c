#define DT_DRV_COMPAT bm_wheel_driver

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "motor_driver.h"

LOG_MODULE_REGISTER(bm_motor, CONFIG_BM_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

const struct gpio_dt_spec gpio_stby = GPIO_DT_SPEC_GET(MOTOR_NODE, stby_gpios);

struct wheel_config {
    const struct gpio_dt_spec standby;
    const struct gpio_dt_spec xi1_gpio;
    const struct gpio_dt_spec xi2_gpio;
    const struct pwm_dt_spec  pwm;
};

struct dir_val dir_forward = {
    .xi1  = GPIO_LOW,
    .xi2  = GPIO_HIGH,
    .stby = GPIO_HIGH,
};

struct dir_val dir_backward = {
    .xi1  = GPIO_HIGH,
    .xi2  = GPIO_LOW,
    .stby = GPIO_HIGH,
};

struct dir_val dir_stop = {
    .xi1  = GPIO_LOW,
    .xi2  = GPIO_LOW,
    .stby = GPIO_LOW,
};

static int wheel_init(const struct device* dev)
{
    int                        err    = 0;
    const struct wheel_config* config = dev->config;

    if (!device_is_ready(config->xi1_gpio.port)) {
        LOG_ERR("%s: GPIO device not ready", dev->name);
        err++;
    }
    if (!device_is_ready(config->xi2_gpio.port)) {
        LOG_ERR("%s: xi2_gpio not ready", dev->name);
        err++;
    }
    if (!device_is_ready(config->standby.port)) {
        LOG_ERR("%s:GPIO Standby not ready", dev->name);
        err++;
    }
    if (!device_is_ready(config->pwm.dev)) {
        LOG_ERR("%s: PWM not ready", dev->name);
        err++;
    }
    err += gpio_pin_configure_dt(&config->xi1_gpio, GPIO_OUTPUT_LOW);
    err += gpio_pin_configure_dt(&config->xi2_gpio, GPIO_OUTPUT_LOW);
    err += gpio_pin_configure_dt(&config->standby, GPIO_OUTPUT_HIGH);
    if (err != 0) {
        LOG_ERR("Motor not initiated, GPIO not configured");
        return 1;
    }
    LOG_INF("%s ready", dev->name);
    return 0;
}

/*
Set motor's angular velocity by adjusting PWM signal according to an input voltage.
Voltage levels between 0% and 100% of VM input.
*/
static int set_motor_speed(const struct device* dev, uint8_t speed_percentage)
{
    const struct wheel_config* config = dev->config;
    if (speed_percentage < MIN_VOLTAGE_PERCENTAGE && speed_percentage > MAX_VOLTAGE_PERCENTAGE) {
        LOG_ERR("Invalid speed value, speed not set to %d", speed_percentage);
        return 1;
    }
    pwm_set_dt(&config->pwm, PWM_MSEC(PERIOD), PWM_MSEC(PERIOD) * speed_percentage * DUTY_CYCLE_TO_PULSE_WIDTH);
    return 0;
}

/*
Set the motor's direction of rotation by setting specific GPIO pins high and low.
 */
static int set_motor_dir(const struct device* dev, const struct dir_val rotation_dir)
{
    const struct wheel_config* config = dev->config;
    uint8_t                    err    = 0;
    err += gpio_pin_set_dt(&config->xi1_gpio, rotation_dir.xi1);
    err += gpio_pin_set_dt(&config->xi2_gpio, rotation_dir.xi2);
    err += gpio_pin_set_dt(&config->standby, rotation_dir.stby);
    if (err != 0) {
        LOG_ERR("%s: Command not set", dev->name);
        return 1;
    }
    return 0;
}

/*
Rotate wheel "forward" with speed_percentage [0,100]%
*/
static int rotate_wheel_forward(const struct device* dev, uint8_t speed_percentage)
{
    uint8_t err = 0;
    err += set_motor_dir(dev, dir_forward);
    err += set_motor_speed(dev, speed_percentage);
    if (err != 0) {
        LOG_ERR("%s: Forward command not set", dev->name);
        return 1;
    }
    return 0;
}

/*
Rotate wheel "backward" with speed_percentage [0,100]%
*/
static int rotate_wheel_backward(const struct device* dev, uint8_t speed_percentage)
{
    uint8_t err = 0;
    err += set_motor_dir(dev, dir_backward);
    err += set_motor_speed(dev, speed_percentage);
    if (err != 0) {
        LOG_ERR("%s: Backward command not set", dev->name);
        return 1;
    }
    return 0;
}

/*
Stop rotation
*/
static int stop_wheel(const struct device* dev)
{
    uint8_t err = 0;
    err += set_motor_dir(dev, dir_stop);
    err += set_motor_speed(dev, 0);
    if (err != 0) {
        LOG_ERR("%s: Stop command not set", dev->name);
        return 1;
    }
    return 0;
}

#define WHEEL_DRIVER_DEVICE(i)                                     \
                                                                   \
    static const struct wheel_config wheel_config_##i = {          \
        .standby  = gpio_stby,                                     \
        .xi1_gpio = GPIO_DT_SPEC_INST_GET_OR(i, xi1_gpios, { 0 }), \
        .xi2_gpio = GPIO_DT_SPEC_INST_GET_OR(i, xi2_gpios, { 0 }), \
        .pwm      = PWM_DT_SPEC_INST_GET(i),                       \
    };                                                             \
    static const struct wheel_api api_##i = {                      \
        .forward  = rotate_wheel_forward,                          \
        .backward = rotate_wheel_backward,                         \
        .stop     = stop_wheel,                                    \
    };                                                             \
                                                                   \
    DEVICE_DT_INST_DEFINE(i, &wheel_init, NULL,                    \
        NULL, &wheel_config_##i,                                   \
        APPLICATION, CONFIG_SENSOR_INIT_PRIORITY,                  \
        &api_##i);

DT_INST_FOREACH_STATUS_OKAY(WHEEL_DRIVER_DEVICE)

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)