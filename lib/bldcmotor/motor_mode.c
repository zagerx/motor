/**
 * @file motor_mode.c
 * @brief BLDC motor control mode implementations
 *
 * Contains state machine implementations for:
 * - Open loop control
 * - Speed control
 * - Position control
 * - Torque control
 *
 * Copyright (c) 2023 Your Company
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <statemachine.h>
 #include <lib/bldcmotor/motor.h>
 #include <lib/foc/foc.h>//TODO
 #include <drivers/currsmp.h>
 #include <drivers/pwm.h>
 #include <drivers/feedback.h>
 #include <zephyr/logging/log.h>
 
 /* Module logging setup */
 LOG_MODULE_REGISTER(motor_mode, LOG_LEVEL_DBG);
 
 /* Forward declaration */
 static void motor_start(const struct device *dev);
 
 /**
  * @brief Open loop control mode state machine
  * @param obj State machine control block
  * @return fsm_rt_cpl when complete
  *
  * States:
  * 1. ENTER: Initialize open loop mode
  * 2. IDLE: Main operational state
  * 3. EXIT: Cleanup when exiting mode
  */
 fsm_rt_t motor_open_loop_mode(fsm_cb_t *obj)
 {
   switch (obj->chState) {
     case ENTER:
       LOG_INF("Enter %s loop mode",obj->name);
       motor_start(obj->pdata);
       obj->chState = MOTOR_STATE_IDLE;
       break;
       
     case MOTOR_STATE_IDLE:
       /* Main operational state - handled by FOC */
       break;
       
     case EXIT:
       LOG_INF("Exit loop mode");
       break;
       
     default:
       break;    
   }
   return fsm_rt_cpl;
 }
 
 /**
  * @brief Speed control mode state machine
  * @param obj State machine control block
  * @return fsm_rt_cpl when complete
  *
  * States:
  * 1. ENTER: Initialize speed control
  * 2. IDLE: Main operational state
  * 3. EXIT: Cleanup when exiting mode
  */
 fsm_rt_t motor_speed_control_mode(fsm_cb_t *obj)
 {  
   switch (obj->chState) {
     case ENTER:
       LOG_INF("Enter speed mode");
       obj->chState = MOTOR_STATE_IDLE;
       break;
       
     case MOTOR_STATE_IDLE:
       /* Main operational state - handled by FOC */
       break;
       
     case EXIT:
       LOG_INF("Exit speed mode");
       break;
       
     default:
       break;    
   }
   return fsm_rt_cpl;
 }
 
 /**
  * @brief Position control mode (stub)
  * @param obj State machine control block
  * @return fsm_rt_cpl when complete
  *
  * TODO: Implement position control logic
  */
 fsm_rt_t motor_position_control_mode(fsm_cb_t *obj)
 {
   return fsm_rt_cpl;
 }
 
 /**
  * @brief Torque control mode (stub)
  * @param obj State machine control block
  * @return fsm_rt_cpl when complete
  *
  * TODO: Implement torque control logic
  */
 fsm_rt_t motor_torque_control_mode(fsm_cb_t *obj)
 {
   return fsm_rt_cpl;
 }
 
 /**
  * @brief Start motor hardware
  * @param dev Motor device instance
  *
  * Initializes:
  * 1. Feedback device (encoder/hall)
  * 2. PWM outputs
  */
 static void motor_start(const struct device *dev)
 {
   if (!device_is_ready(dev)) {
     LOG_ERR("PWM motor1 device not ready");
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