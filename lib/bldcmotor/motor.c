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
// #include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_ll_gpio.h"

#include "zephyr/device.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Local includes */
#include <lib/focutils/utils/utils.h>
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
extern fsm_rt_t motor_open_loop_mode(fsm_cb_t *obj);
extern fsm_rt_t motor_speed_control_mode(fsm_cb_t *obj);

/**
 * @brief Set motor operating mode
 * @param mode Requested mode (MOTOR_CMD_SET_SPEED_MODE/MOTOR_CMD_SET_LOOP_MODE)
 *
 * Iterates through all configured motors and sets their command mode.
 * Skips motors that aren't ready.
 */
void motor_set_mode(int16_t mode)
{
    /* Get motor devices from device tree */
    const struct device *motors[] = {
    #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor0), okay)
            DEVICE_DT_GET(DT_NODELABEL(motor0)),
    #endif
    #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor1), okay)
            DEVICE_DT_GET(DT_NODELABEL(motor1))
    #endif
    };

    const struct device *motor;
    struct motor_data *data;

    /* Process each motor */
    for(uint8_t i = 0;i < ARRAY_SIZE(motors);i++)
    {
      if (!device_is_ready(motors[i])) 
      {
        LOG_ERR("Motor %d not ready", i);
        continue;
      }
      
      motor = motors[i];
      data = motor->data;

      /* Set requested mode */
      if(mode == MOTOR_CMD_SET_SPEED_MODE)
      {
        data->cmd = MOTOR_CMD_SET_SPEED_MODE;
      }else if(mode == MOTOR_CMD_SET_LOOP_MODE){
        data->cmd = MOTOR_CMD_SET_LOOP_MODE;
      }
    }
}
void motor_set_status(int16_t status)
{
    /* Get motor devices from device tree */
    const struct device *motors[] = {
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor0), okay)
                DEVICE_DT_GET(DT_NODELABEL(motor0)),
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor1), okay)
                DEVICE_DT_GET(DT_NODELABEL(motor1))
        #endif
        };
    
        const struct device *motor;
        struct motor_data *data;
        fsm_cb_t *fsm;
        const struct motor_config *cfg;
        /* Process each motor */
        for(uint8_t i = 0;i < ARRAY_SIZE(motors);i++)
        {
          if (!device_is_ready(motors[i])) 
          {
            LOG_ERR("Motor %d not ready", i);
            continue;
          }
          
          motor = motors[i];
          data = motor->data;
          cfg = motor->config;
          fsm = cfg->fsm;
          /* Set requested mode */
          if(status == MOTOR_STATE_STOP)
          {
            data->statue = MOTOR_STATE_STOP;
            fsm->chState = MOTOR_STATE_STOP;
          }else if(status == MOTOR_STATE_CLOSED_LOOP){
            data->statue = MOTOR_STATE_CLOSED_LOOP;
          }else if(status == MOTOR_STATE_INIT){
            data->statue =  MOTOR_STATE_INIT;
            fsm->chState = MOTOR_STATE_INIT;
          }
        }    
}
void motor_set_pid_param(float kp,float ki,float kc,float kd)
{
    /* Get motor devices from device tree */
    const struct device *motors[] = {
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor0), okay)
                DEVICE_DT_GET(DT_NODELABEL(motor0)),
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor1), okay)
                DEVICE_DT_GET(DT_NODELABEL(motor1))
        #endif
        };

        const struct device *motor;
        struct motor_data *data;
        const struct motor_config *cfg;
        const struct device *foc;        
        struct foc_data *f_data;
        pid_cb_t *pid_h;
        fsm_cb_t *fsm;
        /* Process each motor */
        for(uint8_t i = 0;i < ARRAY_SIZE(motors);i++)
        {
          if (!device_is_ready(motors[i])) 
          {
            LOG_ERR("Motor %d not ready", i);
            continue;
          }
          motor = motors[i];

          cfg = motor->config;
          foc = cfg->foc_dev;
          f_data = foc->data;          
          pid_h = &(f_data->iq_pid);      
          pid_init(pid_h, kp, ki, 1.0f,12.0f,-12.0f);
          data = motor->data;
          data->statue = MOTOR_STATE_PARAM_UPDATE;
          fsm = cfg->fsm;
          fsm->chState = MOTOR_STATE_PARAM_UPDATE;
          /* Set requested mode */
        }    
}

void motor_set_ref_param(int8_t flag, float current_ref,float speed_ref)
{
    /* Get motor devices from device tree */
    const struct device *motors[] = {
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor0), okay)
                DEVICE_DT_GET(DT_NODELABEL(motor0)),
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor1), okay)
                DEVICE_DT_GET(DT_NODELABEL(motor1))
        #endif
        };

        const struct device *motor;
        const struct motor_config *cfg;
        const struct device *foc;        
        struct foc_data *f_data;

        /* Process each motor */
        for(uint8_t i = 0;i < ARRAY_SIZE(motors);i++)
        {
          if (!device_is_ready(motors[i])) 
          {
            LOG_ERR("Motor %d not ready", i);
            continue;
          }
          motor = motors[i];

          cfg = motor->config;
          foc = cfg->foc_dev;
          f_data = foc->data;
          
          f_data->iq_ref = current_ref;
          f_data->speed_ref = speed_ref;
        }    
}


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
    // const struct motor_data* m_data;
    // if(data->iq_ref > -0.01f && data->iq_ref < 0.01f)
    // {
    //   return;
    // } 
    /* Get current measurements */
    LL_GPIO_SetOutputPin(GPIOE, GPIO_PIN_1);
    currsmp_get_currents(currsmp, &current_now);
    data->eangle = feedback_get_eangle(cfg->feedback);
    /* Generate test signals for open loop */
    float alph, beta, sin_the, cos_the;
    sin_cos_f32(((data->eangle - _PI_2_) * 57.2957795131f), &sin_the, &cos_the);

    clarke_f32(current_now.i_a,current_now.i_b,&(data->v_alpha),&(data->v_beta));
    park_f32((data->v_alpha),(data->v_beta),&(data->i_d),&(data->i_q),sin_the,cos_the);
    
    /* Update rotor angle */
    data->self_theta += 0.0008f;
    if (data->self_theta > 6.28f) {
        data->self_theta = 0.0f;
    }
    float d_out,q_out;
    d_out = pid_contrl((pid_cb_t *)(&data->id_pid), 0.0f, data->i_d);
    // d_out = 0.0f;
    q_out = pid_contrl((pid_cb_t *)(&data->iq_pid), data->iq_ref, data->i_q);
    q_out = -0.02f;

    /*

     */
     const float Vmax = 12.0f; // 根据系统参数设置
     float mag_sq = d_out*d_out + q_out*q_out;
     if (mag_sq > Vmax*Vmax) {
        float temp ;
        sqrt_f32(mag_sq,&temp);
         float scale = Vmax / temp;
         d_out *= scale;
         q_out *= scale;
     }

    /* Perform inverse Park transform */
    sin_cos_f32((data->eangle * 57.2957795131f), &sin_the, &cos_the);
    inv_park_f32(d_out, q_out, &alph, &beta, sin_the, cos_the);
    
    /* Generate PWM outputs */
    foc_modulate(foc,alph,beta);
    svm_t *svm = data->svm_handle;
    pwm_set_phase_voltages(cfg->pwm, svm->duties.a, svm->duties.b, svm->duties.c);
    LL_GPIO_ResetOutputPin(GPIOE, GPIO_PIN_1);

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
   
    // pid_cb_t *id_pid,*iq_pid;
    const struct device *foc = cfg->foc_dev;
    struct foc_data *data = foc->data;
    
    pid_init(&(data->id_pid),0.006f,0.0001f,1.0f,12.0f,-12.0f);
    pid_init(&(data->iq_pid),0.006f,0.0001f,1.0f,12.0f,-12.0f);
    /* Initialize state machine */
    statemachine_init(cfg->fsm, dev->name, motor_open_loop_mode, (void *)dev) ;
    return 0;
}

/**
 * @brief Main motor control task
 * @param obj Unused parameter
 *
 * Handles:
 * 1. Mode switching between control states
 * 2. FSM dispatching
 * Runs periodically to update motor control.
 */
void motor_task(void *obj)
{
    /* Get motor devices from device tree */
    const struct device *motors[] = {
    #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor0), okay)
            DEVICE_DT_GET(DT_NODELABEL(motor0)),
    #endif
    #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor1), okay)
            DEVICE_DT_GET(DT_NODELABEL(motor1))
    #endif
    };

    const struct device *motor;
    struct motor_data *data;

    /* Process each motor */
    for(uint8_t i = 0;i < ARRAY_SIZE(motors);i++)
    {
        if (!device_is_ready(motors[i])) 
        {
            LOG_ERR("Motor %d not ready", i);
            continue;
        }

        motor = motors[i];
        const struct motor_config *cfg = motor->config;
        data = motor->data;

        /* Handle mode change requests */
        switch (data->cmd) {
            case MOTOR_CMD_SET_SPEED_MODE:
                TRAN_STATE(cfg->fsm, motor_speed_control_mode);
                data->cmd = MOTOR_CMD_UNSED;
            break;
            case MOTOR_CMD_SET_LOOP_MODE:
                TRAN_STATE(cfg->fsm, motor_open_loop_mode);
                data->cmd = MOTOR_CMD_UNSED;
            break;
            case MOTOR_CMD_SET_TORQUE_MODE:
            break;
            default:
            break;
        }
        
        /* Run state machine */
        DISPATCH_FSM(cfg->fsm);
    }
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