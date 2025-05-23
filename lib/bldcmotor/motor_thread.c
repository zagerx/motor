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

 #include <zephyr/kernel.h>
 #include "zephyr/device.h"
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/logging/log.h>
 
 /* Module logging setup */
 LOG_MODULE_REGISTER(motor_thread, LOG_LEVEL_DBG);
 
 /* Thread stack definition */
 K_THREAD_STACK_DEFINE(motor_thread_stack, 1024);
 
 /* Device tree node aliases */
 #define LED0_NODE DT_ALIAS(led0)
 #define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)
 #define ENCODER_VCC DT_NODELABEL(encoder_vcc)
 #define W_DOG DT_NODELABEL(wdog)
 
 /* GPIO device specification */
 const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
 
 /* External motor control function */
 extern void motor_task(void *obj);
 
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