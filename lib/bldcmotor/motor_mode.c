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
#include <drivers/currsmp.h>
#include <drivers/feedback.h>
#include <drivers/pwm.h>
#include <lib/bldcmotor/motor.h>
#include <lib/bldcmotor/motor_internal.h>
#include <lib/foc/foc.h> //TODO
#include <statemachine/statemachine.h>
#include <zephyr/logging/log.h>
#include "stm32h7xx_ll_gpio.h"

/* Module logging setup */
LOG_MODULE_REGISTER(motor_mode, LOG_LEVEL_DBG);

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
fsm_rt_t motor_torque_control_mode(fsm_cb_t *obj) {
  const struct device *motor = obj->p1;

  struct motor_data *m_data = motor->data;
  const struct device *foc =
      ((const struct motor_config *)motor->config)->foc_dev;
  struct foc_data *data = foc->data;
  LL_GPIO_SetOutputPin(GPIOE, GPIO_PIN_1);

  float bus_vol = currsmp_get_busvol();

  LL_GPIO_ResetOutputPin(GPIOE, GPIO_PIN_1);

  foc_write_data(foc, FOC_PARAM_BUSVOL, &bus_vol);

  statemachine_updatestatus(obj, obj->sig);

  switch (obj->chState) {
  case ENTER:
    LOG_INF("Enter %s loop mode", obj->name);
    m_data->mode = MOTOR_MODE_TORQUE;
    motor_start(motor);
    pid_init(&(data->id_pid), 0.016f, 0.008f, 0.5f, 12.0f, -12.0f);//0.076000  0.080000
    pid_init(&(data->iq_pid), 0.016f, 0.008f, 0.5f, 12.0f, -12.0f);
    obj->chState = MOTOR_STATE_IDLE;
    break;
  case MOTOR_STATE_INIT:
    m_data->statue = MOTOR_STATE_INIT;
    LOG_INF("motor status: MOTOR_STATE_INIT");
    motor_set_threephase_enable(motor);
    data->iq_ref = 0.0f;
    obj->chState = MOTOR_STATE_IDLE;
    break;
  case MOTOR_STATE_CLOSED_LOOP: // Runing
    m_data->statue = MOTOR_STATE_CLOSED_LOOP;
    break;
  case MOTOR_STATE_PARAM_UPDATE:
    m_data->statue = MOTOR_STATE_PARAM_UPDATE;
    LOG_INF("motor status: MOTOR_STATE_PARAM_UPDATE");
    motor_set_threephase_disable(motor);

    float kp, ki;
    float *param = (float *)obj->p2;
    kp = param[0];
    ki = param[1];
    pid_init(&(data->id_pid), kp, ki, 0.50f, 12.0f, -12.0f);
    pid_init(&(data->iq_pid), kp, ki, 0.50f, 12.0f, -12.0f);
    LOG_INF("pid param %f,%f", (double)kp, (double)ki);
    param[0] = 0.0f;
    param[1] = 0.0f;
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
    pid_reset(&(data->id_pid));
    pid_reset(&(data->iq_pid));
    motor_set_threephase_disable(motor);
    obj->chState = MOTOR_STATE_IDLE;
    break;

  case EXIT:
    LOG_INF("Exit loop mode");
    motor_set_threephase_disable(motor);
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
fsm_rt_t motor_speed_control_mode(fsm_cb_t *obj) {
  const struct device *motor = obj->p1;

  struct motor_data *m_data = motor->data;
  const struct device *foc =
      ((const struct motor_config *)motor->config)->foc_dev;
  struct foc_data *f_data = foc->data;

  float bus_vol = currsmp_get_busvol();
  foc_write_data(foc, FOC_PARAM_BUSVOL, &bus_vol);
  statemachine_updatestatus(obj, obj->sig);
  switch (obj->chState) {
  case ENTER:
    m_data->mode = MOTOR_MODE_SPEED;
    LOG_INF("Enter %s speed mode", obj->name);
    pid_init(&(f_data->id_pid), 0.076f, 0.08f, 0.5f, 24.0f, -24.0f);
    pid_init(&(f_data->iq_pid), 0.076f, 0.08f, 0.5f, 24.0f, -24.0f);
    pid_init(&(f_data->speed_pid), 0.0085f, 0.00135f, 0.5f, 48.0f, -48.0f);
    motor_start(motor);
    obj->chState = MOTOR_STATE_IDLE;
    break;
    /*-----------INIT-------------*/
  case MOTOR_STATE_INIT:
    m_data->statue = MOTOR_STATE_INIT;
    LOG_INF("motor status: MOTOR_STATE_INIT");
    motor_set_threephase_enable(motor);
    f_data->speed_ref = 0.0f;
    obj->chState = MOTOR_STATE_IDLE;
    break;
    /*---------RUNING-------------*/
  case MOTOR_STATE_CLOSED_LOOP: // Runing
    m_data->statue = MOTOR_STATE_CLOSED_LOOP;
    float cur_speed;
    cur_speed = f_data->speed_real;
    f_data->id_ref = 0.0f;
    f_data->iq_ref =
        pid_contrl(&f_data->speed_pid, f_data->speed_ref, cur_speed);
    break;
    /*-------PARAM UPDATE----------*/
  case MOTOR_STATE_PARAM_UPDATE:
    m_data->statue = MOTOR_STATE_PARAM_UPDATE;
    LOG_INF("motor status: MOTOR_STATE_PARAM_UPDATE");
    motor_set_threephase_disable(motor);
    float kp, ki;
    float *param = (float *)obj->p2;
    kp = param[0];
    ki = param[1];
    pid_init(&(f_data->speed_pid), kp, ki, 0.50f, 48.0f, -48.0f);
    LOG_INF("pid param %f,%f", (double)kp, (double)ki);
    param[0] = 0.0f;
    param[1] = 0.0f;
    obj->chState = MOTOR_STATE_IDLE;
    break;
    /*-----------IDLE-------------*/
  case MOTOR_STATE_IDLE:
    m_data->statue = MOTOR_STATE_IDLE;
    /* Main operational state - handled by FOC */
    break;
    /*-----------STOP-------------*/
  case MOTOR_STATE_STOP:
    m_data->statue = MOTOR_STATE_STOP;
    LOG_INF("motor status: MOTOR_STATE_STOP");
    f_data->iq_ref = 0.0f;
    f_data->speed_ref = 0.0f;
    pid_reset(&(f_data->speed_pid));
    pid_reset(&(f_data->id_pid));
    pid_reset(&(f_data->iq_pid));
    motor_set_threephase_disable(motor);
    obj->chState = MOTOR_STATE_IDLE;
    break;

  case EXIT:
    LOG_INF("Exit speed mode");
    motor_set_threephase_disable(motor);
    motor_stop(motor);
    break;

  default:
    break;
  }
  obj->sig = NULL_USE_SING;
  return fsm_rt_cpl;
}

/**
 * @brief Position control mode (stub)
 * @param obj State machine control block
 * @return fsm_rt_cpl when complete
 *
 * TODO: Implement position control logic
 */
fsm_rt_t motor_position_control_mode(fsm_cb_t *obj) { return fsm_rt_cpl; }
