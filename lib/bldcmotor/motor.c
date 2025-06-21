/**
 * @file motor.c
 * @brief BLDC motor control thread implementation
 *
 * This module implements:
 * 1. Hardware initialization (GPIO/PWM etc)
 * 2. FOC control algorithm
 * 3. Watchdog timer operation
 *
 * Copyright (c) 2023 Your Company
 * SPDX-License-Identifier: Apache-2.0
 */

/* System includes */
#include "algorithmlib/pid.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_gpio.h"

#include "zephyr/device.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Local includes */
#include <lib/focutils/utils/focutils.h>
#include <lib/bldcmotor/motor.h>
#include <lib/foc/foc.h>//TODO
#include <drivers/currsmp.h>
#include <drivers/pwm.h>
#include <drivers/feedback.h>
#include <statemachine/statemachine.h>

/* Device tree compatibility string */
#define DT_DRV_COMPAT motor_bldc

/* Module logging setup */
LOG_MODULE_REGISTER(motor, LOG_LEVEL_DBG);

/* External FSM state handlers */
extern fsm_rt_t motor_torque_control_mode(fsm_cb_t *obj);
extern fsm_rt_t motor_speed_control_mode(fsm_cb_t *obj);
extern fsm_rt_t motor_position_control_mode(fsm_cb_t *obj);
extern  fsm_rt_t motor_test_control_mode(fsm_cb_t *obj);
/**
 * motor_state_map - Motor FSM signal-to-state transition mapping
 * @signal: Trigger signal
 * @target_state: Resulting state after signal processing
 */
static struct state_transition_map motor_state_map[] = {
	/* Reserved signals */
  {.signal = NULL_USE_SING,            .target_state = -1},
	/* Command signals */
  {.signal = MOTOR_CMD_SET_ENABLE,     .target_state = MOTOR_STATE_INIT},
  {.signal = MOTOR_CMD_SET_DISABLE,    .target_state = MOTOR_STATE_STOP},
  {.signal = MOTOR_CMD_SET_PIDPARAM,   .target_state = MOTOR_STATE_PARAM_UPDATE},
  {.signal = MOTOR_CMD_SET_SPEED,      .target_state = MOTOR_STATE_CLOSED_LOOP},
  {.signal = MOTOR_CMD_SET_START,      .target_state = MOTOR_STATE_CLOSED_LOOP},

};

/**
 * @brief FOC current regulator callback
 * @param ctx Device context pointer
 *
 * Implements the FOC current control loop:
 * 1. Gets current measurements
 * 2. Performs Park/Inverse Park transforms
 * 3. Generates PWM outputs via SVM
 */
static void foc_curr_regulator(void *ctx)
{    
    struct device *dev = (struct device*)ctx;
    struct motor_config *cfg = (struct motor_config *)dev->config;
    struct device *currsmp = (struct device *)cfg->currsmp;

    const struct device *foc = cfg->foc_dev;
    struct foc_data *data = foc->data;
    struct currsmp_curr current_now;

    struct motor_data *m_data = dev->data;
    svm_t *svm = data->svm_handle;
    /* Get current measurements */
    currsmp_get_currents(currsmp, &current_now);
    data->i_a = current_now.i_a;data->i_b = current_now.i_b;data->i_c = current_now.i_c;
    data->eangle = feedback_get_eangle(cfg->feedback);
    data->pos_real = feedback_get_position(cfg->feedback);
    foc_calculate_speed(foc,feedback_get_rads(cfg->feedback)*95493.0f*0.2f);

    /* Generate test signals for open loop */
    float alph, beta, sin_the, cos_the;
    sin_cos_f32(((data->eangle - 90.0f)), &sin_the, &cos_the);
    
    clarke_f32(current_now.i_a,current_now.i_b,&(data->i_alpha),&(data->i_beta));
    park_f32((data->i_alpha),(data->i_beta),&(data->i_d),&(data->i_q),sin_the,cos_the);
    
    /* Update rotor angle */
    float d_out,q_out;
    if(m_data->statue == MOTOR_STATE_CLOSED_LOOP)
    {
      d_out = pid_contrl((pid_cb_t *)(&data->id_pid), 0.0f, data->i_d);
      // d_out = 0.0f;
      q_out = pid_contrl((pid_cb_t *)(&data->iq_pid), data->iq_ref, data->i_q);
      // q_out = data->iq_ref; 
      // q_out = -0.02f;
      svm_apply_voltage_limiting(foc,&d_out, &q_out,data->bus_vol);

      sin_cos_f32(((data->eangle)), &sin_the, &cos_the);
      inv_park_f32(d_out, q_out, &alph, &beta, sin_the, cos_the);
    }else if(m_data->statue == MOTOR_STATE_ALIGN){
      d_out = 0.0f;
      q_out = 0.02;
      svm_apply_voltage_limiting(foc,&d_out, &q_out,data->bus_vol);
      sin_cos_f32(((0.0f) * 57.2957795131f), &sin_the, &cos_the);
      inv_park_f32(d_out, q_out, &alph, &beta, sin_the, cos_the);      
    }else{
      d_out = 0.0f;
      q_out = 0.0f;
      sin_cos_f32(((data->eangle)* 57.2957795131f), &sin_the, &cos_the);
      inv_park_f32(d_out, q_out, &alph, &beta, sin_the, cos_the);
    }
    foc_modulate(foc,alph,beta);
    pwm_set_phase_voltages(cfg->pwm, svm->duties.a, svm->duties.b, svm->duties.c);
}

void motor_set_mode(const struct device* motor,enum motor_mode mode)
{
  const struct motor_config* mcfg = motor->config;
  fsm_cb_t* mfsm = mcfg->fsm;
  if(mode == MOTOR_MODE_TORQUE)
  {
    TRAN_STATE(mfsm, motor_torque_control_mode);
  }else if(mode == MOTOR_MODE_SPEED){
    TRAN_STATE(mfsm, motor_speed_control_mode);
  }else if(mode == MOTOR_MODE_POSI){
    TRAN_STATE(mfsm, motor_position_control_mode);
  }
}

enum motor_mode motor_get_mode(const struct device* motor)
{
  const struct motor_data *mdata = motor->data;
  return mdata->mode; 
}

void motor_set_state(const struct device* motor,enum motor_cmd sig)
{
  struct motor_config* mcfg = (struct motor_config*)motor->config;
  statemachine_setsig((fsm_cb_t*)mcfg->fsm,sig);
}

enum motor_state motor_get_state(const struct device *motor)
{
  const struct motor_data *mdata = motor->data;
  return mdata->statue;
}

void motor_set_target(const struct device* motor,float target)
{
  const struct motor_data *mdata = motor->data;
  const struct motor_config *mcfg = motor->config;
  const struct device *devfoc = mcfg->foc_dev;
  if(mdata->mode == MOTOR_MODE_SPEED){
    foc_write_data(devfoc,FOC_PARAM_SPEED_REF,(float *)&target);                
  }else if(mdata->mode == MOTOR_MODE_TORQUE){
    foc_write_data(devfoc,FOC_PARAM_DQ_REF,(float *)&target);
  }else if(mdata->mode == MOTOR_MODE_POSI){
    // foc_write_data(devfoc, FOC_PARAM_POSI_REF,(float *)&target);
    foc_write_data(devfoc, FOC_PARAM_POSI_PLANNING,(float *)&target);
  }
}

float motor_get_curposi(const struct device* motor)
{
  const struct motor_config *mcfg = motor->config;
  const struct device *devfoc = mcfg->foc_dev;  
  const struct foc_data *fdata = devfoc->data;
  return fdata->pos_real;
}
/**
 * @brief Motor device initialization

 * @param dev Motor device instance
 * @return 0 on success, negative errno on failure
 *
 * Sets up:
 * 1. Current sampling callback
 * 2. Initial FSM state
 */
static int motor_init(const struct device *dev)
{
    const struct motor_config *cfg = dev->config;
    const struct device *currsmp = cfg->currsmp;
    
    /* Configure current sampling */
    currsmp_configure(currsmp, foc_curr_regulator, (void *)dev);
    LOG_INF("foc_init name: %s", dev->name);   
   
    /* Initialize state machine */
    statemachine_init(cfg->fsm, dev->name, motor_position_control_mode, (void *)dev,motor_state_map,ARRAY_SIZE(motor_state_map));
    // motor_set_mode(dev, MOTOR_MODE_POSI);
    return 0;
}
/* Device tree instantiation macros */

#define MOTOR_INIT(n) \
    fsm_cb_t fsm_##n;\
    static const struct motor_config motor_cfg_##n = { \
        .foc_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, control_algorithm)), \
        .pwm = DEVICE_DT_GET(DT_INST_PHANDLE(n, pwm)), \
        .currsmp = DEVICE_DT_GET(DT_INST_PHANDLE(n, currsmp)), \
        .feedback = DEVICE_DT_GET(DT_INST_PHANDLE(n, feedback)), \
        .fsm = &fsm_##n,\
    }; \
    static struct motor_data motor_data_##n; \
    DEVICE_DT_INST_DEFINE(n, motor_init, NULL, \
                         &motor_data_##n, \
                         &motor_cfg_##n, \
                         POST_KERNEL, \
                         CONFIG_MOTOR_INIT_PRIORITY, \
                         NULL);

/* Create device instances for all enabled nodes */
DT_INST_FOREACH_STATUS_OKAY(MOTOR_INIT)