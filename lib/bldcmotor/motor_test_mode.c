#include "algorithmlib/pid.h"
#include "zephyr/device.h"
#include <drivers/currsmp.h>
#include <drivers/feedback.h>
#include <drivers/pwm.h>
#include <lib/bldcmotor/motor.h>
#include <lib/bldcmotor/motor_internal.h>
#include <lib/foc/foc.h> //TODO
#include <statemachine/statemachine.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
#include "stm32h7xx_ll_gpio.h"
#include <algorithmlib/s_posi_planning.h>
#include <lib/focutils/utils/focutils.h>
#include <lib/bldcmotor/motor.h>
#include <lib/foc/foc.h>//TODO
#include <drivers/currsmp.h>
#include <drivers/pwm.h>
#include <drivers/feedback.h>
LOG_MODULE_REGISTER(motor_test_mode, LOG_LEVEL_DBG);

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
 fsm_rt_t motor_test_control_mode(fsm_cb_t *obj) {
    const struct device *motor = obj->p1;
  
    struct motor_data *m_data = motor->data;
    const struct device *foc =
        ((const struct motor_config *)motor->config)->foc_dev;
    struct foc_data *data = foc->data;

    const struct motor_config *mcfg = motor->config;
    LL_GPIO_SetOutputPin(GPIOE, GPIO_PIN_1);
  
    float bus_vol = currsmp_get_busvol();
  
    LL_GPIO_ResetOutputPin(GPIOE, GPIO_PIN_1);
  
    foc_write_data(foc, FOC_PARAM_BUSVOL, &bus_vol);
  
    statemachine_updatestatus(obj, obj->sig);
  
    switch (obj->chState) {
    case ENTER:
      LOG_INF("Enter %s loop mode", obj->name);
      m_data->mode = MOTOR_MODE_SELFCHECK;
      motor_start(motor);
      pid_init(&(data->id_pid), 0.08f, 0.006f, 0.5f, 12.0f, -12.0f);
      pid_init(&(data->iq_pid), 0.08f, 0.006f, 0.5f, 12.0f, -12.0f);
      obj->chState = MOTOR_STATE_INIT;
      break;
    case MOTOR_STATE_INIT:
      m_data->statue = MOTOR_STATE_INIT;
      LOG_INF("motor status: MOTOR_STATE_INIT");
      motor_set_threephase_enable(motor);
      data->iq_ref = 0.0f;
      feedback_calibration(mcfg->feedback);
      obj->chState = MOTOR_STATE_ALIGN;
      break;
    case MOTOR_STATE_ALIGN:
      m_data->statue = MOTOR_STATE_ALIGN;
      if(obj->count++>2000)
      {
        obj->count = 0;
        obj->chState = MOTOR_STATE_CLOSED_LOOP;
      }
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

 void eloop_test(void *ctx)
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
    foc_calculate_speed(foc,feedback_get_rads(cfg->feedback)*95493.0f*0.2f/57.2957795131f);

    /* Generate test signals for open loop */
    float alph, beta, sin_the, cos_the;
    sin_cos_f32(((data->eangle ) - 90.0f), &sin_the, &cos_the);
    
    float temp_ia,temp_ib;
    temp_ia = current_now.i_a;
    temp_ib = current_now.i_b;
    clarke_f32(temp_ia,temp_ib,&(data->i_alpha),&(data->i_beta));
    park_f32((data->i_alpha),(data->i_beta),&(data->i_d),&(data->i_q),sin_the,cos_the);
    
    /* Update rotor angle */
    float d_out,q_out;
    static uint8_t cnt = 0;
    if(m_data->statue == MOTOR_STATE_CLOSED_LOOP)
    {
      data->self_theta += (0.004f*57.2957795131f);
      if (data->self_theta > (360.0f)) {
          data->self_theta = 0.0f;
          cnt++;
      }
      d_out = pid_contrl((pid_cb_t *)(&data->id_pid), 0.0f, data->i_d);
      d_out = 0.00f;
      q_out = pid_contrl((pid_cb_t *)(&data->iq_pid), data->iq_ref, data->i_q);
      q_out = -0.02f; 

      // if(cnt>=5){
      //   d_out = 0.0f;
      // }
      svm_apply_voltage_limiting(foc,&d_out, &q_out,data->bus_vol);
      // sin_cos_f32(((data->self_theta)), &sin_the, &cos_the);
      sin_cos_f32(((data->eangle)), &sin_the, &cos_the);
      inv_park_f32(d_out, q_out, &alph, &beta, sin_the, cos_the);
    }else if(m_data->statue == MOTOR_STATE_ALIGN){
      d_out = 0.02f;
      q_out = 0.00;
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