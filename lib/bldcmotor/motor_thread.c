/**
 * @file motor_thread.c
 * @brief BLDC motor control thread implementation
 *
 * Handles:
 * - Motor control thread creation
 * - Hardware initialization (GPIOs, watchdog)
 * - Main motor control loop
 *
 * Copyright (c) 2023 Your Company
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <stdint.h>
#include <zephyr/kernel.h>
#include "statemachine/statemachine.h"
 #include "zephyr/device.h"
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/logging/log.h>
 #include <lib/bldcmotor/motor.h>
 /* Module logging setup */
 LOG_MODULE_REGISTER(motor_thread, LOG_LEVEL_DBG);
 
 /* Thread stack definition */
 K_THREAD_STACK_DEFINE(motor_thread_stack, 2048);
 
 /* Device tree node aliases */
 #define LED0_NODE DT_ALIAS(led0)
 #define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)
 #define ENCODER_VCC DT_NODELABEL(encoder_vcc)
 #define W_DOG DT_NODELABEL(wdog)
 #define P_SWITCH DT_NODELABEL(proximity_switch)
 /* GPIO device specification */
 const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
 
 /* External motor control function */
 extern void motor_task(void *obj);
 void super_elevator_task(void* obj);
 extern fsm_rt_t motor_torque_control_mode(fsm_cb_t *obj);
 extern fsm_rt_t motor_speed_control_mode(fsm_cb_t *obj);
 extern fsm_rt_t motor_position_control_mode(fsm_cb_t *obj);
  
 /**
  * @struct motor_thread_data
  * @brief Motor thread control structure
  */
 struct motor_thread_data {
     const struct device *motor_dev;  ///< Motor device pointer
     struct k_thread thread;         ///< Thread control block
 };
 
 /**
  * @brief Motor control thread entry function
  * @param p1 Unused parameter
  * @param p2 Unused parameter
  * @param p3 Unused parameter
  *
  * Initializes hardware and runs main control loop:
  * 1. Configures all GPIO devices
  * 2. Starts motor control tasks
  * 3. Maintains watchdog timer
  */
 static void motor_thread_entry(void *p1, void *p2, void *p3)
 {
 #if defined(CONFIG_BOARD_ZGM_002)
     /* Initialize LED indicator */
     if (!device_is_ready(led.port)) {
         LOG_ERR("LED device not ready");
         return;
     }
     k_msleep(1000);
     int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
     if (ret < 0) {
         LOG_ERR("Failed to configure LED (err %d)", ret);
     }
 
     /* Initialize brake pin */
     const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
     ret = gpio_pin_configure_dt(&mot12_brk, GPIO_OUTPUT_ACTIVE);
     if (ret < 0) {
         LOG_ERR("Failed to configure brake pin (err %d)", ret);
     }
 
     /* Initialize watchdog pin */
     const struct gpio_dt_spec w_dog = GPIO_DT_SPEC_GET(W_DOG, gpios);
     ret = gpio_pin_configure_dt(&w_dog, GPIO_OUTPUT_ACTIVE);
     if (ret < 0) {
         LOG_ERR("Failed to configure watchdog pin (err %d)", ret);
     }
 
     /* Initialize encoder power */
     const struct gpio_dt_spec encoder_vcc = GPIO_DT_SPEC_GET(ENCODER_VCC, gpios);
     ret = gpio_pin_configure_dt(&encoder_vcc, GPIO_OUTPUT_ACTIVE);
     if (ret < 0) {
         LOG_ERR("Failed to configure encoder power (err %d)", ret);
     }

     const struct gpio_dt_spec prx_switch = GPIO_DT_SPEC_GET(P_SWITCH, gpios);
     ret = gpio_pin_configure_dt(&prx_switch, GPIO_INPUT);
     if (ret < 0) {
        LOG_ERR("Failed to configure proximity switch (err %d)", ret);
     } else {
        LOG_INF("Proximity switch configured");
     }
          
 #endif
 
     /* Initial delay for hardware stabilization */
     k_msleep(1000);
 
     /* Main control loop */
     while (1) {
 #if defined(CONFIG_BOARD_ZGM_002)
         /* Toggle watchdog */
         gpio_pin_toggle_dt(&w_dog);
 #endif
         /* Run motor control tasks */
         motor_task(NULL);
         super_elevator_task(NULL);
         k_msleep(1);
     }
 }
 
 /**
  * @brief Create motor control thread
  * @param dev Unused device pointer
  *
  * Creates high-priority cooperative thread for motor control.
  */
 void creat_motor_thread(const struct device *dev)
 {
     static struct motor_thread_data thread_data __aligned(4);
     
     k_thread_create(&thread_data.thread,
                    motor_thread_stack,
                    K_THREAD_STACK_SIZEOF(motor_thread_stack),
                    motor_thread_entry,
                    NULL,
                    NULL,
                    NULL,
                    K_PRIO_COOP(5),  // High priority cooperative thread
                    0,
                    K_NO_WAIT);
 }
 
static fsm_cb_t elevator_handle;
uint8_t conctrl_cmd = 0;
void super_elevator_task(void* obj)
{
    enum{
        ELEVATOR_INIT = USER_STATUS,
        ELEVATOR_FINDZERO,
        ELEVATOR_ZERO,
        ELEVATOR_TEMP0,
        ELEVATOR_TEMP1,
        ELEVATOR_ISZERO,
        ELEVATOR_END,
        ELEVATOR_MID,

    };
    fsm_cb_t* elevator_fsm = &elevator_handle;
    const struct gpio_dt_spec prx_switch = GPIO_DT_SPEC_GET(P_SWITCH, gpios);

    const struct device *motor = (DEVICE_DT_GET(DT_NODELABEL(motor0)));
    struct motor_data *data;
    // const struct motor_config *cfg = motor->config;
    data = motor->data;

    int switch_state;
    switch_state = gpio_pin_get_dt(&prx_switch);
    elevator_fsm->p1 = (void *)motor;
    switch (elevator_fsm->chState) {
        case ENTER:

        case ELEVATOR_INIT:
            {
                
                if (switch_state < 0) {
                    // elevator_fsm->chState = ELEVATOR_ZERO;
                    LOG_WRN("Failed to read proximity switch");
                } else {//电机正转 找零点
                    if(switch_state == 0)
                    {
                        if(data->mode != MOTOR_MODE_SPEED)
                        {
                            motor_cmd_set(MOTOR_CMD_SET_SPEED_MODE ,NULL,0);
                        }
                        if(data->mode == MOTOR_MODE_SPEED)
                        {
                            motor_cmd_set(MOTOR_CMD_SET_ENABLE,NULL,0);
                            elevator_fsm->chState = ELEVATOR_FINDZERO; 
                        }
                    }else{
                        elevator_fsm->chState = ELEVATOR_ZERO;
                    }
                    LOG_DBG("Proximity switch state: %d", switch_state);
                }
            }
        break;
        case ELEVATOR_FINDZERO:
            {
                float speed = 150.0f;
                motor_cmd_set(MOTOR_CMD_SET_SPEED,&speed,1);
                if(switch_state == 1)
                {
                    motor_cmd_set(MOTOR_CMD_SET_POSTION_MODE ,NULL,0);
                    elevator_fsm->chState = ELEVATOR_ZERO;
                }
            }
        break;
        case ELEVATOR_ZERO :
            {
                if(conctrl_cmd == 2){//开始升起
                    motor_cmd_set(MOTOR_CMD_SET_ENABLE,NULL,0);
                    elevator_fsm->chState = ELEVATOR_TEMP0;
                    conctrl_cmd = 255;                    
                }
            }
            break;
        case ELEVATOR_TEMP0:
                {
                    float posi = -4000.0f;
                    motor_cmd_set(MOTOR_CMD_SET_SPEED,&posi,1);
                    elevator_fsm->chState = ELEVATOR_END;
                }
            break;
        case ELEVATOR_MID:
            //读取当前位置

            //如果到达终点
            // elevator_fsm->chState = ELEVATOR_END;
            //如果到达零点
            // elevator_fsm->chState = ELEVATOR_ZERO

            //根据实际情况，是否需要失能电机及落下报闸
            break;
        case ELEVATOR_END:
            {
                if(conctrl_cmd == 1){//回零点
                   //设置目标位置 
                   conctrl_cmd = 255;
                   motor_cmd_set(MOTOR_CMD_SET_ENABLE,NULL,0);
                   elevator_fsm->chState = ELEVATOR_TEMP1;
                }
            }
            break;
        case ELEVATOR_TEMP1:
        {
            float posi = 4000.0f;
            motor_cmd_set(MOTOR_CMD_SET_SPEED,&posi,1);
            elevator_fsm->chState = ELEVATOR_ISZERO;
        }
        break;
        case ELEVATOR_ISZERO:
            if(switch_state == 1)
            {
                motor_cmd_set(MOTOR_CMD_SET_DISABLE,NULL,0);
                elevator_fsm->chState = ELEVATOR_ZERO;
            }
        break;
        case EXIT:
        break;
    }    

}
