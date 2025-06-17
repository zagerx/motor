  /**
 * @file motor_internal.c
 * @brief BLDC motor hardware control implementation
  *
 * Implements low-level hardware operations:
 * - PWM control
 * - Feedback device management
 * - Three-phase bridge control
 *
 * Copyright (c) 2023 Your Company
 * SPDX-License-Identifier: Apache-2.0
  */
  #include "algorithmlib/pid.h"
  #include "zephyr/device.h"
  #include <statemachine/statemachine.h>
   #include <lib/bldcmotor/motor.h>
   #include <lib/foc/foc.h>//TODO
   #include <drivers/currsmp.h>
   #include <drivers/pwm.h>
   #include <drivers/feedback.h>
   #include <zephyr/logging/log.h>
   
   /* Module logging setup */
   LOG_MODULE_REGISTER(motor_internal, LOG_LEVEL_DBG);
  
/**
 * motor_start - Initialize motor hardware peripherals
 * @dev: Motor device instance
 *
 * Caller must ensure device is ready before calling. Initializes:
 * - Feedback device (encoder/hall sensors)
 * - PWM outputs
 */
  void motor_start(const struct device *dev)
  {
    if (!device_is_ready(dev)) {
      LOG_ERR("motor_start: device not ready");
      return;
    }
    
    const struct motor_config *cfg = dev->config;
    
    /* Start feedback device */
    const struct device *dev_f = cfg->feedback;
    feedback_start(dev_f);
    
    /* Start PWM outputs */
    const struct device *devc = cfg->pwm;
    pwm_start(devc);
  }
 
/**
 * motor_stop - Deactivate motor hardware
 * @dev: Motor device instance
 *
 * Safely stops PWM outputs while maintaining configuration.
 */
  void motor_stop(const struct device *dev)
  {
    if (!device_is_ready(dev)) {
      LOG_ERR("motor_stop: device not ready");
      return;
    }
    
    const struct motor_config *cfg = dev->config;
    
    /* Stop PWM outputs */
    const struct device *devc = cfg->pwm;
    pwm_stop(devc);
  } 
 
/**
 * motor_set_threephase_disable - Disable three-phase bridge
 * @dev: Motor device instance
 *
 * Turns off all phase outputs while maintaining PWM configuration.
 */
  void motor_set_threephase_disable(const struct device *dev)
  {
   if (!device_is_ready(dev)) {
     LOG_ERR("motor_set_threephase_disable: device not ready");
     return;
   }
   const struct motor_config *cfg = dev->config;
    
   /* Stop PWM outputs */
   const struct device *devc = cfg->pwm;
   svpwm_set_phase_state(devc,0);  
  }
 
/**
 * motor_set_threephase_enable - Enable three-phase bridge
 * @dev: Motor device instance
 *
 * Re-enables phase outputs using current PWM configuration.
 */
  void motor_set_threephase_enable(const struct device *dev)
  {
   if (!device_is_ready(dev)) {
     LOG_ERR("motor_set_threephase_enable: device not ready");
     return;
   }
   const struct motor_config *cfg = dev->config;
    
   /* Stop PWM outputs */
   const struct device *devc = cfg->pwm;
   svpwm_set_phase_state(devc,1);  
  }