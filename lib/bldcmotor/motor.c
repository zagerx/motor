/**
 * @file motor.c
 * @brief BLDC电机控制线程实现
 * 
 * 功能:
 * 1. 初始化电机控制相关硬件(GPIO/PWM等)
 * 2. 启动FOC控制算法
 * 3. 运行看门狗定时器
 */

#include "zephyr/device.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <lib/foc/foc.h>//TODO
#include <drivers/currsmp.h>
#include <drivers/pwm.h>
#include <drivers/feedback.h>

#include <lib/focutils/utils/utils.h>

#include <statemachine.h>
#define DT_DRV_COMPAT motor_bldc

/* 日志模块注册 */
LOG_MODULE_REGISTER(motor, LOG_LEVEL_DBG);

#define MOTOR0_NODE DT_NODELABEL(motor1)
#define MOTOR1_NODE DT_NODELABEL(motor2)

struct motor_config {
    const struct device *foc_dev;  // FOC控制算法设备
    const struct device *currsmp;
    const struct device *pwm;
    const struct device *feedback;
    fsm_cb_t *fsm;
};


/**
 * @brief FOC电机控制状态枚举
 */
 typedef enum {
  MOTOR_STATE_IDLE,        // 空闲状态，未使能
  MOTOR_STATE_INIT,        // 初始化状态
  MOTOR_STATE_ALIGN,       // 电机对齐状态(初始位置校准)
  MOTOR_STATE_OPEN_LOOP,   // 开环运行状态
  MOTOR_STATE_CLOSED_LOOP, // 闭环运行状态
  MOTOR_STATE_FAULT,       // 故障状态
  MOTOR_STATE_CALIBRATION, // 校准状态(参数辨识)
  MOTOR_STATE_STOP,        // 受控停止状态
  MOTOR_STATE_EMERGENCY    // 紧急停止状态
} motor_state_t;


struct motor_data {
  motor_state_t statue;
  uint8_t test;
};


fsm_rt_t motor_open_loop_mode(fsm_cb_t *obj)
{
  return fsm_rt_cpl;
}

fsm_rt_t motor_speed_control_mode(fsm_cb_t *obj)
{
  return fsm_rt_cpl;
}

fsm_rt_t motor_position_control_mode(fsm_cb_t *obj)
{
  return fsm_rt_cpl;
}
fsm_rt_t motor_torque_control_mode(fsm_cb_t *obj)
{
  return fsm_rt_cpl;
}


 /*
  * Current regulator callback
  * Implements FOC current control loop
  */
  static void foc_curr_regulator(void *ctx)
  {    
      struct device *dev = (struct device*)ctx;
      struct motor_config *cfg = (struct motor_config *)dev->config;
      struct device *currsmp = (struct device *)cfg->currsmp;

      const struct device *foc = cfg->foc_dev;
      
      struct foc_data *data = foc->data;
  
      struct currsmp_curr current_now;
    //   LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_1);
 
      /* Get current measurements */
      currsmp_get_currents(currsmp, &current_now);
      data->eangle = feedback_get_eangle(cfg->feedback);
  
      /* Open loop test generation */
      float alph, beta, sin_the, cos_the;
     //  sin_cos_f32((data->self_theta * 57.2957795131f), &sin_the, &cos_the);
      sin_cos_f32((data->eangle * 57.2957795131f), &sin_the, &cos_the);
      /* Update theta */
      data->self_theta += 0.0008f;
      if (data->self_theta > 6.28f) {
          data->self_theta = 0.0f;
      }
  
      /* Inverse Park transform */
      inv_park_f32(0.0f, 0.02f, &alph, &beta, sin_the, cos_the);
      
      /* Space Vector Modulation */
      foc_modulate(foc,alph,beta);
      /* Set PWM duty cycles */
      svm_t *svm = data->svm_handle;
      pwm_set_phase_voltages(cfg->pwm, svm->duties.a, svm->duties.b, svm->duties.c);
      data->test_a = svm->duties.a * 10000.0f;
    //   LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_1);
  }
  
static int motor_init(const struct device *dev)
{
    const struct motor_config *cfg = dev->config;
    const struct device *currsmp = cfg->currsmp;
    currsmp_configure(currsmp, foc_curr_regulator, (void *)dev);
    LOG_INF("foc_init name: %s", dev->name);   
    
    fsm_cb_t *fsm_x = cfg->fsm;
    fsm_x->fsm = motor_open_loop_mode;
    return 0;
}

void motor_task(void *obj)
{
  const struct device *motor0 = DEVICE_DT_GET(MOTOR0_NODE);
  const struct motor_config *cfg0  = motor0->config;
  fsm_cb_t *fsm0 = cfg0->fsm;
  DISPATCH_FSM(fsm0);

  const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
  const struct motor_config *cfg1  = motor1->config;
  fsm_cb_t *fsm1 = cfg1->fsm;
  DISPATCH_FSM(fsm1);
}
void motor_start(void)
{
     /* 电机0初始化 */
     const struct device *motor0 = DEVICE_DT_GET(MOTOR0_NODE);
     if (!device_is_ready(motor0)) {
         LOG_ERR("PWM motor1 device not ready");
         return;
     }
     const struct motor_config *cfg  = motor0->config;
     const struct device *dev_f = cfg->feedback;
     feedback_start(dev_f);
     const struct device *devc = cfg->pwm;
     pwm_start(devc);

     /* 电机1初始化 */
     const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
     if (!device_is_ready(motor1)) {
         LOG_ERR("PWM motor1 device not ready");
         return;
     }
     cfg  = motor1->config;
     dev_f = cfg->feedback;
     feedback_start(dev_f);
     devc = cfg->pwm;
     pwm_start(devc);
}
#define MOTOR_INIT(n) \
    fsm_cb_t fsm_##n;\
    static const struct motor_config motor_cfg_##n = { \
        .foc_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(n), control_algorithm)), \
        .pwm = DEVICE_DT_GET(DT_INST(n, st_stm32_pwm_custom)), \
        .currsmp = DEVICE_DT_GET(DT_INST(n, st_stm32_currsmp_shunt)), \
        .feedback = DEVICE_DT_GET(DT_INST(n, st_stm32_abz_hall)), \
        .fsm = &fsm_##n,\
    }; \
    static struct motor_data motor_data_##n; \
    DEVICE_DT_INST_DEFINE(n, motor_init, NULL, \
                         &motor_data_##n, \
                         &motor_cfg_##n, \
                         POST_KERNEL, \
                         CONFIG_MOTOR_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_INIT)

