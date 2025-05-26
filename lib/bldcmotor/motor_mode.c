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
 LOG_MODULE_REGISTER(motor_mode, LOG_LEVEL_DBG);
 
 /* Forward declaration */
 static void motor_start(const struct device *dev);
 static void motor_stop(const struct device *dev);
 static void motor_set_threephase_enable(const struct device *dev);
 static void motor_set_threephase_disable(const struct device *dev);
 
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
  const struct device* motor = obj->p1;

  struct motor_data *m_data = motor->data;  
  const struct device* foc = ((const struct motor_config*)motor->config)->foc_dev;
  struct foc_data* data = foc->data;

  statemachine_updatestatus(obj,obj->sig); 

   switch (obj->chState) {
     case ENTER:
       LOG_INF("Enter %s loop mode",obj->name);
       motor_start(motor);
       obj->chState = MOTOR_STATE_IDLE;
       break;
     case MOTOR_STATE_INIT:
      m_data->statue = MOTOR_STATE_INIT;
       LOG_INF("motor status: MOTOR_STATE_INIT");
       motor_set_threephase_enable(motor);
       data->iq_ref = 0.0f;
       obj->chState = MOTOR_STATE_CLOSED_LOOP;

       break;
     case MOTOR_STATE_CLOSED_LOOP://Runing
       m_data->statue = MOTOR_STATE_CLOSED_LOOP;
       break; 
     case MOTOR_STATE_PARAM_UPDATE:
       m_data->statue = MOTOR_STATE_PARAM_UPDATE;
       LOG_INF("motor status: MOTOR_STATE_PARAM_UPDATE");
       motor_set_threephase_disable(motor);

       float kp,ki;
       float *param = (float *)obj->p2;
       kp = param[0];ki = param[1];
       pid_init(&(data->id_pid), kp, ki, 0.50f,12.0f,-12.0f);
       pid_init(&(data->iq_pid), kp, ki, 0.50f,12.0f,-12.0f);
       LOG_INF("pid param %f,%f",(double)kp,(double)ki);
       param[0] = 0.0f;param[1] = 0.0f;
       obj->chState = MOTOR_STATE_IDLE;
       break;
    case MOTOR_STATE_IDLE:
       m_data->statue = MOTOR_STATE_IDLE;
       /* Main operational state - handled by FOC */
       break;
     case MOTOR_STATE_STOP:
       m_data->statue = MOTOR_STATE_STOP;
       LOG_INF("motor status: MOTOR_STATE_STOP");
       data->iq_ref = 0.0f; 
       pid_reset(&(data->iq_pid));
       motor_set_threephase_disable(motor);
       obj->chState = MOTOR_STATE_IDLE;
       break;

     case EXIT:
       LOG_INF("Exit loop mode");
       motor_stop(motor);
       break;
       
     default:
       break;    
   }
   obj->sig = NULL_USE_SING;
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

 static void motor_stop(const struct device *dev)
 {
   if (!device_is_ready(dev)) {
     LOG_ERR("PWM motor1 device not ready");
     return;
   }
   
   const struct motor_config *cfg = dev->config;
   
   /* Start feedback device */
  //  const struct device *dev_f = cfg->feedback;
  //  feedback_start(dev_f);
   
   /* Stop PWM outputs */
   const struct device *devc = cfg->pwm;
   pwm_stop(devc);
 } 

 static void motor_set_threephase_disable(const struct device *dev)
 {
  if (!device_is_ready(dev)) {
    LOG_ERR("PWM motor1 device not ready");
    return;
  }
  const struct motor_config *cfg = dev->config;
   
  /* Stop PWM outputs */
  const struct device *devc = cfg->pwm;
  svpwm_set_phase_state(devc,0);  
 }

 static void motor_set_threephase_enable(const struct device *dev)
 {
  if (!device_is_ready(dev)) {
    LOG_ERR("PWM motor1 device not ready");
    return;
  }
  const struct motor_config *cfg = dev->config;
   
  /* Stop PWM outputs */
  const struct device *devc = cfg->pwm;
  svpwm_set_phase_state(devc,1);  
 }