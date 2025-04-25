/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include <stdio.h>
#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <lib/foc/foc.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)
#define ENCODER_VCC DT_NODELABEL(encoder_vcc)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define MOTOR0_NODE DT_INST(0, foc_ctrl_algo)
#define MOTOR1_NODE DT_INST(1, foc_ctrl_algo)
int32_t get_tim3_encoder_count(void);
int32_t get_tim4_encoder_count(void);
int main(void)
{
	// if (!gpio_is_ready_dt(&led)) {
	// 	return 0;
	// }

	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
    int ret = gpio_pin_configure_dt(&mot12_brk, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: Failed to configure brake pin\n", ret);
        // return;
    }
	const struct gpio_dt_spec encoder_vcc = GPIO_DT_SPEC_GET(ENCODER_VCC, gpios);
    ret = gpio_pin_configure_dt(&encoder_vcc, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: Failed to configure brake pin\n", ret);
        // return;
    }
	LOG_INF("main statr");                    // 信息级
	const struct device *motor0 = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor0)) {
        LOG_ERR("PWM motor1 device not ready");
        return -ENODEV;
    }
	foc_start(motor0);

	const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
    if (!device_is_ready(motor1)) {
        LOG_ERR("PWM motor1 device not ready");
        return -ENODEV;
    }
	foc_start(motor1);	

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);      
	while (1) {
		int32_t pos3 = get_tim3_encoder_count();
		int32_t pos4 = get_tim4_encoder_count();
	
		printf("TIM3: %d, TIM4: %d\n", pos3, pos4);
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
