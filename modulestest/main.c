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
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   200

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
extern void tim1_enable(void);
extern void tim1_set_pwm(float _a, float _b, float _c);
struct pwm_driver_api {
	void (*start)(const struct device *dev);
	void (*stop)(const struct device *dev);
	void (*set_phase_voltages)(const struct device *dev,
				   float ua, float ub, float uc);
};
struct motor_config{
    const struct device *pwm;
	const struct device *ctrl_algo;
};
struct foc_api{
    int (*posloop)(const struct device*);
    int (*currloop)(const struct device*);
    int (*opencloop)(const struct device*);
};
#define MOTOR0_NODE DT_INST(0, bldc_motor)
#define MOTOR1_NODE DT_INST(1, bldc_motor)

int main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
	LOG_INF("main statr");                    // 信息级
    const struct device *motor0 = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor0)) {
        LOG_ERR("PWM motor1 device not ready");
        return -ENODEV;
    }
    const struct motor_config *cfg = motor0->config;
	const struct device *pwm0 = cfg->pwm;
	const struct pwm_driver_api *api = pwm0->api;
	api->start(pwm0);
	api->set_phase_voltages(pwm0,0.5f,0.5f,0.5f);

    const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
    if (!device_is_ready(motor1)) {
        LOG_ERR("PWM motor1 device not ready");
        return -ENODEV;
    }
    cfg = motor1->config;
	pwm0 = cfg->pwm;
	api = pwm0->api;
	api->start(pwm0);
	api->set_phase_voltages(pwm0,0.5f,0.5f,0.5f);
	
	const struct device *ctrl_algo = cfg->ctrl_algo;
	const struct foc_api *foc_api = ctrl_algo->api;
	foc_api->opencloop(motor0);

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);      
	while (1) {
		// gpio_pin_toggle_dt(&led);
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
