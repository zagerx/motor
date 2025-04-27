#include "stm32h7xx.h"
#include <stm32_ll_tim.h>
#include <sys/_stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <stm32h7xx_ll_tim.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include <zephyr/logging/log.h>

#include <drivers/pwm.h>
LOG_MODULE_REGISTER(pwm_stm32, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT st_stm32_pwm_custom
struct pwm_stm32_config {
    TIM_TypeDef *timer;
    struct stm32_pclken pclken;
    const struct pinctrl_dev_config *pincfg;
    uint32_t timing_params[3];  // [0]=t_dead_ns, [1]=arr, [2]=psc
    uint32_t slave_enable;
};
// struct pwm_driver_api {
// 	void (*start)(const struct device *dev);
// 	void (*stop)(const struct device *dev);
// 	void (*set_phase_voltages)(const struct device *dev,
// 				   float ua, float ub, float uc);
// };

static void pwm_stm32_stop(const struct device *dev)
{
	const struct pwm_stm32_config *cfg = dev->config;
	uint32_t slave_flag;
	slave_flag = cfg->slave_enable;
	if(!slave_flag)
	{
	}else{
	}
}
static void pwm_stm32_start(const struct device *dev)
{
	const struct pwm_stm32_config *cfg = dev->config;
	uint32_t slave_flag;
	slave_flag = cfg->slave_enable;

	if(!slave_flag)
	{
		LOG_INF("master timer");
		LL_TIM_EnableAllOutputs(cfg->timer);
		LL_TIM_EnableCounter(cfg->timer);
		LL_TIM_CC_EnableChannel(cfg->timer,LL_TIM_CHANNEL_CH4);
	}else{
		LOG_INF("Slave timer");
		LL_TIM_EnableAllOutputs(cfg->timer);	
	}
	LL_TIM_CC_EnableChannel(cfg->timer,\
							LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 |\
							LL_TIM_CHANNEL_CH1N| LL_TIM_CHANNEL_CH1N| LL_TIM_CHANNEL_CH1N);  

}
static void pwm_stm32_setduties(const struct device *dev,float a,float b,float c)
{
	const struct pwm_stm32_config *cfg = dev->config;
    LL_TIM_OC_SetCompareCH1(cfg->timer, (uint32_t)(cfg->timing_params[1]*a));
    LL_TIM_OC_SetCompareCH2(cfg->timer, (uint32_t)(cfg->timing_params[1]*b));
    LL_TIM_OC_SetCompareCH3(cfg->timer, (uint32_t)(cfg->timing_params[1]*c));

    LL_TIM_OC_SetCompareCH4(cfg->timer, (uint32_t)(cfg->timing_params[1]-200)); //TODO
}
/*==========================================================================================
 * @brief        配置PWM频率、对应通道
 所需参数  
 1、通道号
 2、系统频率
 3、是否作为主定时器
 所有的配置参数都通过dts来配置
 * @FuncName     
 * @param        dev 
 * @version      0.1
--------------------------------------------------------------------------------------------*/
static int pwm_stm32_init(const struct device *dev)
{
    const struct pwm_stm32_config *config = dev->config;
	int ret;
	LL_TIM_InitTypeDef tim_init;
	LL_TIM_OC_InitTypeDef tim_ocinit;
	LL_TIM_BDTR_InitTypeDef brk_dt_init;

	/* configure pinmux */
	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl setup failed (%d)", ret);
		return ret;
	}
	/* 启动定时器时钟*/
	const struct device *clk;
	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	ret = clock_control_on(clk, (clock_control_subsys_t *)&config->pclken);
	if (ret < 0) {
		LOG_ERR("Could not turn on timer clock (%d)", ret);
		return ret;
	}
	/* 获取定时器频率 */
	uint32_t rate;
	if(clock_control_get_rate(clk, (clock_control_subsys_t *)&config->pclken, &rate))
	{
		LOG_ERR("clock_control_get_rate (%d)", ret);
	}

	tim_init.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP;
    tim_init.Autoreload = config->timing_params[1];  // arr
    tim_init.Prescaler = config->timing_params[2];   // psc
	tim_init.RepetitionCounter = 0U;
	if (LL_TIM_Init(config->timer, &tim_init) != SUCCESS) {
		LOG_ERR("Could not initialize timer");
		return -EIO;
  	}
	LL_TIM_DisableARRPreload(config->timer);
	if(!config->slave_enable)
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
	if(!config->slave_enable)
	{
		LL_TIM_SetTriggerOutput(config->timer, LL_TIM_TRGO_ENABLE);
		LL_TIM_SetTriggerOutput2(config->timer, LL_TIM_TRGO2_OC4);
		LL_TIM_EnableMasterSlaveMode(config->timer);
	}else{
		LL_TIM_SetTriggerInput(config->timer,LL_TIM_TS_ITR0);
  		LL_TIM_SetSlaveMode(config->timer, LL_TIM_SLAVEMODE_TRIGGER);
	}
	brk_dt_init.OSSRState = LL_TIM_OSSR_DISABLE;
	brk_dt_init.OSSIState = LL_TIM_OSSI_DISABLE;
	brk_dt_init.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    brk_dt_init.DeadTime = config->timing_params[0];  // t_dead_ns
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

#define PMW_STM32_INIT(n) \
    PINCTRL_DT_INST_DEFINE(n); \
    static const struct pwm_stm32_config pwm_stm32_config_##n = { \
        .timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)), \
        .pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)), \
        .timing_params = DT_INST_PROP(n, timing_params), \
        .slave_enable = DT_INST_PROP(n, slave), \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
    }; \
	\
	static const struct pwm_driver_api pwm_stm32_api_##n = {\
		.start = pwm_stm32_start,\
		.stop = pwm_stm32_stop,\
		.set_phase_voltages = pwm_stm32_setduties,\
	};\
	\
	DEVICE_DT_INST_DEFINE(n, \
        &pwm_stm32_init, \
        NULL, NULL, \
        &pwm_stm32_config_##n, \
        POST_KERNEL, \
        80, \
        &pwm_stm32_api_##n);

DT_INST_FOREACH_STATUS_OKAY(PMW_STM32_INIT)

