#ifndef MOTOR_INTERNAL_H
#define MOTOR_INTERNAL_H

#include <zephyr/device.h>

/**
 * @brief Initialize motor hardware (PWM/Feedback)
 * @param dev Motor device instance
 */
void motor_start(const struct device *dev);

/**
 * @brief Stop motor hardware (PWM/Feedback)
 * @param dev Motor device instance 
 */
void motor_stop(const struct device *dev);

/**
 * @brief Disable three-phase output
 * @param dev Motor device instance
 */
void motor_set_threephase_disable(const struct device *dev);

/**
 * @brief Enable three-phase output 
 * @param dev Motor device instance
 */
void motor_set_threephase_enable(const struct device *dev);

#endif /* MOTOR_INTERNAL_H */