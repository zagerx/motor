/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include <sys/_stdint.h>
#define DT_DRV_COMPAT st_stm32_abz_hall

 #include <zephyr/drivers/clock_control/stm32_clock_control.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/drivers/pinctrl.h>
 #include <zephyr/irq.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/irq.h>
 #include <drivers/feedback.h>
 #include <stm32_ll_tim.h>
 
 LOG_MODULE_REGISTER(abz_hall_stm32, LOG_LEVEL_DBG);
 
 /*******************************************************************************
  * Private
  ******************************************************************************/
 
 struct abz_hall_stm32_config {
	 uint32_t lines;
	 uint32_t pole_pairs;
	 TIM_TypeDef *timer;
	 struct stm32_pclken pclken;
	 const struct pinctrl_dev_config *pcfg;
	 struct gpio_dt_spec hu_gpio;
     struct gpio_dt_spec hv_gpio;
     struct gpio_dt_spec hw_gpio;
 };
 
 struct abz_hall_stm32_data {
	 int overflow;
	 float mangle_ratio;
	 float eangle_ratio;
     struct gpio_callback gpio_cb;
     const struct device *dev;  // 添加设备指针
	 uint8_t cur_sect;
	 uint8_t pre_sect;
 };
 /*******************************************************************************
  * API
  ******************************************************************************/
  static float abz_stm32_get_eangle(const struct device *dev)
  {
	  return 0.0f; 
  }
  
  static float abz_stm32_get_mangle(const struct device *dev)
  {
	  return 0.0f;
  }

  static float abz_stm32_get_rads(const struct device *dev)
  {
	  /* fixme: */
	  return 0.0f;
  }
  
  static float abz_stm32_get_position(const struct device *dev)
  {
	  return 0.0f; 
  }

  static const struct feedback_driver_api driver_feedback = {
	  .get_rads = abz_stm32_get_rads,
	  .get_eangle = abz_stm32_get_eangle,
	  .get_mangle = abz_stm32_get_mangle,
	  .get_position = abz_stm32_get_position
  };
  
 /*******************************************************************************
  * Init
  ******************************************************************************/
  static void hall_gpio_callback(const struct device *port,
                              struct gpio_callback *cb,
                              uint32_t pins)
 {
     struct abz_hall_stm32_data *data = CONTAINER_OF(cb, struct abz_hall_stm32_data, gpio_cb);
 
     ARG_UNUSED(port);
     ARG_UNUSED(pins);

     const struct device *dev = data->dev;
     const struct abz_hall_stm32_config *cfg = dev->config;

     int hu_state = gpio_pin_get_dt(&cfg->hu_gpio);
     int hv_state = gpio_pin_get_dt(&cfg->hv_gpio);
     int hw_state = gpio_pin_get_dt(&cfg->hw_gpio);
     LOG_DBG("Hall states - HALL_VAL:%d", hu_state<<2|hv_state<<1|hw_state);

     data->pre_sect = data->cur_sect;
     data->cur_sect = hu_state<<2|hv_state<<1|hw_state;
 }


 static int abz_stm32_init(const struct device *dev)
 {
	 const struct abz_hall_stm32_config *config = dev->config;
 
	 int ret;
	 const struct device *clk;
	 ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	 if (ret < 0) {
		 LOG_ERR("pinctrl setup failed (%d)", ret);
		 return ret;
	 }
 
	 /* enable timer clock */
	 clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
 
	 ret = clock_control_on(clk, (clock_control_subsys_t *)&config->pclken);
	 if (ret < 0) {
		 LOG_ERR("Could not turn on timer clock (%d)", ret);
		 return ret;
	 }
 
	 /* initialize timer */
	 LL_TIM_InitTypeDef TIM_InitStruct = {0};
	 TIM_InitStruct.Prescaler = 0;
	 TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	 TIM_InitStruct.Autoreload = config->lines;
	 TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	 LL_TIM_Init(config->timer, &TIM_InitStruct);
	 LL_TIM_DisableARRPreload(config->timer);
	 LL_TIM_SetEncoderMode(config->timer, LL_TIM_ENCODERMODE_X2_TI1);
	 LL_TIM_IC_SetActiveInput(config->timer, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	 LL_TIM_IC_SetPrescaler(config->timer, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	 LL_TIM_IC_SetFilter(config->timer, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
	 LL_TIM_IC_SetPolarity(config->timer, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
	 LL_TIM_IC_SetActiveInput(config->timer, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
	 LL_TIM_IC_SetPrescaler(config->timer, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
	 LL_TIM_IC_SetFilter(config->timer, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
	 LL_TIM_IC_SetPolarity(config->timer, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
	 LL_TIM_SetTriggerOutput(config->timer, LL_TIM_TRGO_RESET);
	 LL_TIM_DisableMasterSlaveMode(config->timer);
	//  LL_TIM_EnableCounter(config->timer);
	 return 0;
}
  static void abz_hall_stm32_enable(const struct device *dev)
  {
	const struct abz_hall_stm32_config *cfg = dev;
	uint8_t ret;
	LL_TIM_EnableCounter(cfg->timer);

     ret = gpio_pin_interrupt_configure_dt(&cfg->hu_gpio, GPIO_INT_EDGE_BOTH);
     ret |= gpio_pin_interrupt_configure_dt(&cfg->hv_gpio, GPIO_INT_EDGE_BOTH);
     ret |= gpio_pin_interrupt_configure_dt(&cfg->hw_gpio, GPIO_INT_EDGE_BOTH);

     if (ret < 0) {
         LOG_ERR("Failed to configure interrupts");
     }
  }

static int hall_stm32_init(const struct device *dev)
{
     const struct abz_hall_stm32_config *cfg = dev->config;
     struct abz_hall_stm32_data *data = dev->data;
     int ret;
     uint32_t pin_mask = BIT(cfg->hu_gpio.pin) | BIT(cfg->hv_gpio.pin) | BIT(cfg->hw_gpio.pin);
 
     /* 存储设备指针 */
     data->dev = dev;
 
     /* Configure GPIO pins */
     ret = gpio_pin_configure_dt(&cfg->hu_gpio, GPIO_INPUT);
     ret |= gpio_pin_configure_dt(&cfg->hv_gpio, GPIO_INPUT);
     ret |= gpio_pin_configure_dt(&cfg->hw_gpio, GPIO_INPUT);
     if (ret < 0) {
         LOG_ERR("Failed to configure GPIO pins");
         return ret;
     }
 
     /* Initialize and add callback */
     gpio_init_callback(&data->gpio_cb, hall_gpio_callback, pin_mask);
 
     ret = gpio_add_callback(cfg->hu_gpio.port, &data->gpio_cb);
     ret |= gpio_add_callback(cfg->hv_gpio.port, &data->gpio_cb);
     ret |= gpio_add_callback(cfg->hw_gpio.port, &data->gpio_cb);
     if (ret < 0) {
         LOG_ERR("Failed to add callback");
         return ret;
     }
 
     /* Configure interrupts */
    //  ret = gpio_pin_interrupt_configure_dt(&cfg->hu_gpio, GPIO_INT_EDGE_BOTH);
    //  ret |= gpio_pin_interrupt_configure_dt(&cfg->hv_gpio, GPIO_INT_EDGE_BOTH);
    //  ret |= gpio_pin_interrupt_configure_dt(&cfg->hw_gpio, GPIO_INT_EDGE_BOTH);
    //  if (ret < 0) {
    //      LOG_ERR("Failed to configure interrupts");
    //      return ret;
    //  }
     return 0;
}
static int stm32_abz_hall_init(const struct device *dev)
{
	abz_stm32_init(dev);
    hall_stm32_init(dev);
    LOG_INF("stm32_abz_hall_init Finish");
	return 0;
}

 #define ABZ_HALL_STM32_INIT(n)						\
	 PINCTRL_DT_INST_DEFINE(n);					\
	 static const struct abz_hall_stm32_config abz_hall_stm32_cfg_##n = {	\
		 .timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)), \
		 .pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)),	\
		 .lines = DT_INST_PROP(n, lines),			\
		 .pole_pairs = DT_INST_PROP(n, pole_pairs),		\
		 .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		 .hu_gpio = GPIO_DT_SPEC_INST_GET(n, hu_gpios), \
		 .hv_gpio = GPIO_DT_SPEC_INST_GET(n, hv_gpios), \
		 .hw_gpio = GPIO_DT_SPEC_INST_GET(n, hw_gpios), \
	 };								\
	 static struct abz_hall_stm32_data abz_hall_stm32_data_##n = {};		\
									 \
	DEVICE_DT_INST_DEFINE(n, stm32_abz_hall_init, NULL,			\
				   &abz_hall_stm32_data_##n,			\
				   &abz_hall_stm32_cfg_##n,			\
				   PRE_KERNEL_1, 86, \
				   &driver_feedback			\
		 );
 
 DT_INST_FOREACH_STATUS_OKAY(ABZ_HALL_STM32_INIT)
 