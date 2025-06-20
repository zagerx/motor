#ifndef __MOTOR__H
#define __MOTOR__H
#include "zephyr/device.h"
#include <statemachine/statemachine.h>


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
enum motor_state{
    MOTOR_STATE_IDLE = USER_STATUS,        // 空闲状态，未使能
    MOTOR_STATE_PARAM_UPDATE,
    MOTOR_STATE_INIT,        // 初始化状态
    MOTOR_STATE_ALIGN,       // 电机对齐状态(初始位置校准)
    MOTOR_STATE_OPEN_LOOP,   // 开环运行状态
    MOTOR_STATE_CLOSED_LOOP, // 闭环运行状态
    MOTOR_STATE_FAULT,       // 故障状态
    MOTOR_STATE_CALIBRATION, // 校准状态(参数辨识)
    MOTOR_STATE_STOP,        // 受控停止状态
    MOTOR_STATE_EMERGENCY    // 紧急停止状态
    } ;
    
    enum motor_mode{
    // MOTOR_MODE_OPEN_LOOP,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSI,
    MOTOR_MODE_TORQUE,
    MOTOR_MODE_SELFCHECK
    };
    
    enum motor_cmd{
      MOTOR_CMD_SET_SPEED_MODE = USER_SIG,// 进入速度模式
      MOTOR_CMD_SET_LOOP_MODE_1111111111,
      MOTOR_CMD_SET_TORQUE_MODE,
      MOTOR_CMD_SET_ENABLE,
      MOTOR_CMD_SET_DISABLE,
      MOTOR_CMD_SET_SPEED,//设置速度
      MOTOR_CMD_SET_PIDPARAM,
    };
    struct motor_data {
      enum motor_mode mode;
      enum motor_state statue;
      enum motor_cmd cmd;
    };

    
    extern void motor_set_mode(int16_t mode);
    extern void motor_set_ref_param(int8_t flag, float current_ref,float speed_ref);
    void motor_cmd_set(int16_t cmd,void *pdata,int8_t datalen);


#endif
