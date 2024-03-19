#ifndef DRIVERS_ZEPHYR_INCLUDE_MOTOR_DRIVER_H_
#define DRIVERS_ZEPHYR_INCLUDE_MOTOR_DRIVER_H_

#include <zephyr/device.h>

#define PWM_NODE   DT_NODELABEL(pwms)
#define MOTOR_NODE DT_NODELABEL(motor_node_label)

// Motors specifications.
#define MOTOR_INPUT_VOLTAGE 12.0 // Assumed input voltage motor driver. TODO: Get actual voltage level in runtime.
#define MIN_VOLTAGE         3.0 // Min input voltage motor
#define MAX_VOLTAGE         9.0 // Max input voltage motor

// PWM speifications
#define PERIOD                    20.0 // [ms]
#define DUTY_CYCLE_TO_PULSE_WIDTH 1.0 / 100.0
#define MIN_VOLTAGE_PERCENTAGE    100 * MIN_VOLTAGE / MOTOR_INPUT_VOLTAGE // [%] Min Duty Cycle
#define MAX_VOLTAGE_PERCENTAGE    100 * MAX_VOLTAGE / MOTOR_INPUT_VOLTAGE // [%] Max Duty Cycle

#define GPIO_HIGH 1
#define GPIO_LOW  0

struct dir_val {
    bool xi1;
    bool xi2;
    bool stby;
};

/**
 * @typedef api_forward()
 * @brief drive wheel forward
 *
 * @see wheel_dir_forward() for argument descriptions.
 */
typedef int (*wheel_forward_api)(const struct device* dev, uint8_t speed_percentage);

/**
 * @typedef api_backward()
 * @brief drive wheel backward
 *
 * @see wheel_dir_backward() for argument descriptions.
 */
typedef int (*wheel_backward_api)(const struct device* dev, uint8_t speed_percentage);

/**
 * @typedef api_stop()
 * @brief stop wheel
 *
 * @see wheel_dir_stop() for argument descriptions.
 */
typedef int (*wheel_stop_api)(const struct device* dev);

struct wheel_api {
    wheel_forward_api  forward;
    wheel_backward_api backward;
    wheel_stop_api     stop;
};

#endif /* DRIVERS_ZEPHYR_INCLUDE_MOTOR_DRIVER_H_*/