/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT st_stm32_svpwm

#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h>

#include <drivers/svpwm.h>
#include <svm/svm.h>
#include <utils/stm32_tim.h>

LOG_MODULE_REGISTER(svpwm_stm32, CONFIG_SPINNER_SVPWM_LOG_LEVEL);

/*******************************************************************************
 * Private
 ******************************************************************************/

struct svpwm_stm32_config {
	TIM_TypeDef *timer;
	struct stm32_pclken pclken;
	bool enable_comp_outputs;
	uint32_t t_dead;
	uint32_t t_rise;
	const struct gpio_dt_spec *enable;
	size_t enable_len;
	const struct pinctrl_dev_config *pcfg;
};

struct svpwm_stm32_data {
	uint32_t period;
	svm_t svm;
};

/*******************************************************************************
 * API
 ******************************************************************************/

static void svpwm_stm32_start(const struct device *dev)
{
	const struct svpwm_stm32_config *config = dev->config;
	struct svpwm_stm32_data *data = dev->data;

	/* configure timer OC for a, b, c */
	LL_TIM_OC_SetCompareCH1(config->timer, data->period / 2U);
	LL_TIM_OC_SetCompareCH2(config->timer, data->period / 2U);
	LL_TIM_OC_SetCompareCH3(config->timer, data->period / 2U);

	LL_TIM_CC_EnableChannel(config->timer, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(config->timer, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(config->timer, LL_TIM_CHANNEL_CH3);
	if (config->enable_comp_outputs) {
		LL_TIM_CC_EnableChannel(config->timer, LL_TIM_CHANNEL_CH1N);
		LL_TIM_CC_EnableChannel(config->timer, LL_TIM_CHANNEL_CH2N);
		LL_TIM_CC_EnableChannel(config->timer, LL_TIM_CHANNEL_CH3N);
	}

	/* configure timer OC for ADC trigger */

	/* start timer */
	LL_TIM_EnableAllOutputs(config->timer);
	if(config->timer == TIM1)
	{
		LL_TIM_CC_EnableChannel(config->timer, LL_TIM_CHANNEL_CH4);
		LL_TIM_EnableCounter(config->timer);
	}
}

static void svpwm_stm32_stop(const struct device *dev)
{
	const struct svpwm_stm32_config *config = dev->config;

	/* stop timer */
	LL_TIM_DisableCounter(config->timer);

	LL_TIM_DisableAllOutputs(config->timer);

	LL_TIM_CC_DisableChannel(config->timer, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_DisableChannel(config->timer, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_DisableChannel(config->timer, LL_TIM_CHANNEL_CH3);
	if (config->enable_comp_outputs) {
		LL_TIM_CC_DisableChannel(config->timer, LL_TIM_CHANNEL_CH1N);
		LL_TIM_CC_DisableChannel(config->timer, LL_TIM_CHANNEL_CH2N);
		LL_TIM_CC_DisableChannel(config->timer, LL_TIM_CHANNEL_CH3N);
	}

	LL_TIM_CC_DisableChannel(config->timer, LL_TIM_CHANNEL_CH4);
}

static void svpwm_stm32_set_phase_voltages(const struct device *dev,
					   float ua, float ub, float uc)
{
	const struct svpwm_stm32_config *config = dev->config;
	struct svpwm_stm32_data *data = dev->data;

	LL_TIM_OC_SetCompareCH1(config->timer,
				(uint32_t)(data->period * ua));
	LL_TIM_OC_SetCompareCH2(config->timer,
				(uint32_t)(data->period * ub));
	LL_TIM_OC_SetCompareCH3(config->timer,
				(uint32_t)(data->period * uc));
}

static const struct svpwm_driver_api svpwm_stm32_driver_api = {
	.start = svpwm_stm32_start,
	.stop = svpwm_stm32_stop,
	.set_phase_voltages = svpwm_stm32_set_phase_voltages,
};

/*******************************************************************************
 * Initialization
 ******************************************************************************/

int svpwm_stm32_init(const struct device *dev)
{
	const struct svpwm_stm32_config *config = dev->config;
	struct svpwm_stm32_data *data = dev->data;

	int ret;
	uint32_t freq;
	uint16_t psc;
	const struct device *clk;
	LL_TIM_InitTypeDef tim_init;
	LL_TIM_OC_InitTypeDef tim_ocinit;
	LL_TIM_BDTR_InitTypeDef brk_dt_init;

	/* configure pinmux */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl setup failed (%d)", ret);
		return ret;
	}

	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	/* enable timer clock */
	ret = clock_control_on(clk, (clock_control_subsys_t *)&config->pclken);
	if (ret < 0) {
		LOG_ERR("Could not turn on timer clock (%d)", ret);
		return ret;
	}

	/* compute ARR */
	ret = stm32_tim_clk_get(&config->pclken, &freq);
	if (ret < 0) {
		return ret;
	}

	psc = 0U;
	do {
		data->period = __LL_TIM_CALC_ARR(
			freq, psc, 30000 * 2U);
		psc++;
	} while (data->period > UINT16_MAX);

	/* initialize timer
	 * NOTE: repetition counter set to 1, update will happen on underflow
	 */
	LL_TIM_StructInit(&tim_init);
	tim_init.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP;
	tim_init.Autoreload = data->period;
	tim_init.RepetitionCounter = 1U;
	if (LL_TIM_Init(config->timer, &tim_init) != SUCCESS) {
		LOG_ERR("Could not initialize timer");
		return -EIO;
  	}
	LL_TIM_DisableARRPreload(config->timer);
	if(config->timer == TIM1)
	{
		LL_TIM_SetClockSource(config->timer, LL_TIM_CLOCKSOURCE_INTERNAL);
	}
	LL_TIM_OC_EnablePreload(config->timer, LL_TIM_CHANNEL_CH1);
	tim_ocinit.OCMode = LL_TIM_OCMODE_PWM1;
	tim_ocinit.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_ocinit.OCNState = LL_TIM_OCSTATE_DISABLE;
	tim_ocinit.CompareValue = 0;
	tim_ocinit.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_ocinit.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
	tim_ocinit.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	tim_ocinit.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	LL_TIM_OC_Init(config->timer, LL_TIM_CHANNEL_CH1, &tim_ocinit);
	LL_TIM_OC_DisableFast(config->timer, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(config->timer, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_Init(config->timer, LL_TIM_CHANNEL_CH2, &tim_ocinit);
	LL_TIM_OC_DisableFast(config->timer, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(config->timer, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_Init(config->timer, LL_TIM_CHANNEL_CH3, &tim_ocinit);
	LL_TIM_OC_DisableFast(config->timer, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnablePreload(config->timer, LL_TIM_CHANNEL_CH4);
	LL_TIM_OC_Init(config->timer, LL_TIM_CHANNEL_CH4, &tim_ocinit);
	LL_TIM_OC_DisableFast(config->timer, LL_TIM_CHANNEL_CH4);
	if(config->timer == TIM1)
	{
		LL_TIM_SetTriggerOutput(config->timer, LL_TIM_TRGO_ENABLE);
		LL_TIM_SetTriggerOutput2(config->timer, LL_TIM_TRGO2_OC4);
		LL_TIM_EnableMasterSlaveMode(config->timer);
	}else if(config->timer == TIM8){
		LL_TIM_SetTriggerInput(config->timer,LL_TIM_TS_ITR0);
  		LL_TIM_SetSlaveMode(config->timer, LL_TIM_SLAVEMODE_TRIGGER);
		// LL_TIM_DisableIT_TRIG(config->timer);
		// LL_TIM_DisableDMAReq_TRIG(config->timer);
		// LL_TIM_SetTriggerOutput(config->timer, LL_TIM_TRGO_RESET);
		// LL_TIM_SetTriggerOutput2(config->timer, LL_TIM_TRGO2_RESET);
		// LL_TIM_DisableMasterSlaveMode(config->timer);
	}
	brk_dt_init.OSSRState = LL_TIM_OSSR_DISABLE;
	brk_dt_init.OSSIState = LL_TIM_OSSI_DISABLE;
	brk_dt_init.LockLevel = LL_TIM_LOCKLEVEL_OFF;
	brk_dt_init.DeadTime = 120;
	brk_dt_init.BreakState = LL_TIM_BREAK_DISABLE;
	brk_dt_init.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
	brk_dt_init.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
	brk_dt_init.Break2State = LL_TIM_BREAK2_DISABLE;
	brk_dt_init.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
	brk_dt_init.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
	brk_dt_init.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
	LL_TIM_BDTR_Init(config->timer, &brk_dt_init);
	return 0;
}
#include <stm32h7xx_ll_tim.h>

void tim1_enable(void)
{
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 |\
                                LL_TIM_CHANNEL_CH1N| LL_TIM_CHANNEL_CH1N| LL_TIM_CHANNEL_CH1N);  
  LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH4);
  LL_TIM_EnableAllOutputs(TIM8);
  // LL_TIM_EnableCounter(TIM8);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3);  
}
void tim1_set_pwm(float _a, float _b, float _c)
{
    uint32_t arr = LL_TIM_GetAutoReload(TIM1) + 1; // 动态获取ARR
    LL_TIM_OC_SetCompareCH1(TIM1, (uint32_t)(_a * arr));
    LL_TIM_OC_SetCompareCH2(TIM1, (uint32_t)(_b * arr));
    LL_TIM_OC_SetCompareCH3(TIM1, (uint32_t)(_c * arr));
    LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(_c * arr));

    LL_TIM_OC_SetCompareCH1(TIM8, (uint32_t)(_a * arr));
    LL_TIM_OC_SetCompareCH2(TIM8, (uint32_t)(_b * arr));
    LL_TIM_OC_SetCompareCH3(TIM8, (uint32_t)(_c * arr));    
}



#define SVPMW_STM32_INIT(n)					\
								\
	PINCTRL_DT_INST_DEFINE(n);					\
	static const struct svpwm_stm32_config svpwm_stm32_config_##n = { \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)), \
		.pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)),	\
		.enable_comp_outputs = DT_PROP_OR(DT_INST_CHILD(n, driver), \
						  enable_comp_outputs, false), \
		.t_dead = DT_PROP_OR(DT_INST_CHILD(n, driver), t_dead_ns, 0), \
		.t_rise = DT_PROP_OR(DT_INST_CHILD(n, driver), t_rise_ns, 0), \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
	};								\
									\
	static struct svpwm_stm32_data svpwm_stm32_data_##n;		\
									\
	DEVICE_DT_INST_DEFINE(n, &svpwm_stm32_init, NULL,		\
			      &svpwm_stm32_data_##n,			\
			      &svpwm_stm32_config_##n,			\
			      POST_KERNEL,				\
			      80,	\
			      &svpwm_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SVPMW_STM32_INIT)
