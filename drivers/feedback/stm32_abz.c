/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_abz

 #include <zephyr/drivers/clock_control/stm32_clock_control.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/drivers/pinctrl.h>
 #include <zephyr/irq.h>
 #include <zephyr/logging/log.h>
 
 #include <stm32_ll_tim.h>
 
 LOG_MODULE_REGISTER(abz_stm32, LOG_LEVEL_DBG);
 
 /*******************************************************************************
  * Private
  ******************************************************************************/
 
 struct abz_stm32_config {
	 uint32_t lines;
	 uint32_t pole_pairs;
	 TIM_TypeDef *timer;
	 struct stm32_pclken pclken;
	 const struct pinctrl_dev_config *pcfg;
 };
 
 struct abz_stm32_data {
	 int overflow;
	 float mangle_ratio;
	 float eangle_ratio;
 };
 /*******************************************************************************
  * API
  ******************************************************************************/
 
 static uint16_t abz_stm32_get_count(const struct device *dev)
 {
	 const struct abz_stm32_config *config = dev->config;
	 uint16_t cnt = LL_TIM_GetCounter(config->timer);
	 return (cnt);
 }
 
struct stm32_abz_driver_api {
	uint16_t (*get_count)(const struct device *dev);
};
 
 static const struct stm32_abz_driver_api abz_stm32_driver_api = {
	.get_count=  abz_stm32_get_count,
 };
 
 /*******************************************************************************
  * Init
  ******************************************************************************/
 
 static int abz_stm32_init(const struct device *dev)
 {
	 const struct abz_stm32_config *config = dev->config;
 
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
	 LL_TIM_EnableCounter(config->timer);
	 return 0;
 }
 int32_t get_tim3_encoder_count(void)
{
    static uint16_t last_cnt = 0;
    static int32_t total_cnt = 0;
    
    uint16_t current_cnt = LL_TIM_GetCounter(TIM3);
    int16_t delta = (int16_t)(current_cnt - last_cnt);
    total_cnt += delta;
    last_cnt = current_cnt;
    
    return total_cnt;
}
int32_t get_tim4_encoder_count(void)
{
    static uint16_t last_cnt = 0;
    static int32_t total_cnt = 0;
    
    uint16_t current_cnt = LL_TIM_GetCounter(TIM4);
    int16_t delta = (int16_t)(current_cnt - last_cnt);
    total_cnt += delta;
    last_cnt = current_cnt;
    
    return total_cnt;
}
 #define ABZ_STM32_INIT(n)						\
	 PINCTRL_DT_INST_DEFINE(n);					\
	 static const struct abz_stm32_config abz_stm32_cfg_##n = {	\
		 .timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)), \
		 .pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)),	\
		 .lines = DT_INST_PROP(n, lines),			\
		 .pole_pairs = DT_INST_PROP(n, pole_pairs),		\
		 .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
	 };								\
									 \
	 static struct abz_stm32_data abz_stm32_data_##n = {};		\
									 \
	 DEVICE_DT_INST_DEFINE(n, abz_stm32_init, NULL,			\
				   &abz_stm32_data_##n,			\
				   &abz_stm32_cfg_##n,			\
				   POST_KERNEL, 86, \
				   &abz_stm32_driver_api			\
		 );
 
 DT_INST_FOREACH_STATUS_OKAY(ABZ_STM32_INIT)
 