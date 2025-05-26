/*
 * STM32 PWM driver implementation
 *
 * Features:
 * - Center-aligned PWM generation
 * - Master/slave timer synchronization
 * - Dead-time insertion
 * - Complementary channel outputs
 */

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
 
 /* PWM configuration structure */
 struct pwm_stm32_config {
	 TIM_TypeDef *timer;                  /* Timer register base */
	 struct stm32_pclken pclken;          /* Clock enable info */
	 const struct pinctrl_dev_config *pincfg; /* Pin configuration */
	 uint32_t timing_params[3];           /* [0]=dead time (ns), [1]=ARR, [2]=PSC */
	 uint32_t slave_enable;               /* Slave mode flag */
 };
 
 /*
  * Stop PWM generation
  */
 static void pwm_stm32_stop(const struct device *dev)
 {
	 const struct pwm_stm32_config *cfg = dev->config;
	 uint32_t slave_flag = cfg->slave_enable;
	 
	 if (!slave_flag) {
		 /* Master timer stop logic */
		 LL_TIM_DisableCounter(cfg->timer);
	 } else {
		 /* Slave timer stop logic */
	 }
 }
 
 /*
  * Start PWM generation
  */
 static void pwm_stm32_start(const struct device *dev)
 {
	 const struct pwm_stm32_config *cfg = dev->config;
	 uint32_t slave_flag = cfg->slave_enable;
 
	 if (!slave_flag) {
		 LOG_INF("master timer");
		 LL_TIM_EnableAllOutputs(cfg->timer);
		 LL_TIM_EnableCounter(cfg->timer);
		 LL_TIM_CC_EnableChannel(cfg->timer, LL_TIM_CHANNEL_CH4);
		 LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(cfg->timing_params[1]-30));//TODO
	 } else {
		 LOG_INF("Slave timer");
		 LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(cfg->timing_params[1]-30));
		 LL_TIM_EnableAllOutputs(cfg->timer);    
	 }
	 
	 /* Enable all PWM channels */
	 LL_TIM_CC_EnableChannel(cfg->timer,
						   LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 |
						   LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);  
 }
 
 /*
  * Set PWM duty cycles for three phases
  */
 static void pwm_stm32_setduties(const struct device *dev, float a, float b, float c)
 {
	 const struct pwm_stm32_config *cfg = dev->config;
	 if(a>0.9f||b>0.9f||c>0.9f)
	 {
		return;
	 } 
	 LL_TIM_OC_SetCompareCH1(cfg->timer, (uint32_t)(cfg->timing_params[1]*a));
	 LL_TIM_OC_SetCompareCH2(cfg->timer, (uint32_t)(cfg->timing_params[1]*b));
	 LL_TIM_OC_SetCompareCH3(cfg->timer, (uint32_t)(cfg->timing_params[1]*c));
	 
	 if (!cfg->slave_enable) {
		 LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(cfg->timing_params[1]-30));
	 }
 }
static void pwm_stm32_setstatus(const struct device* dev,int8_t flag)
{
	const struct pwm_stm32_config *cfg = dev->config;
	LL_TIM_OC_SetCompareCH1(cfg->timer, 0);
	LL_TIM_OC_SetCompareCH2(cfg->timer, 0);
	LL_TIM_OC_SetCompareCH3(cfg->timer, 0);	
	if(flag)
	{
		LL_TIM_CC_EnableChannel(cfg->timer,
			LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 |
			LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N); 
	}else{
		LL_TIM_CC_DisableChannel(cfg->timer,
			LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 |
			LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N); 			 
	}
}

 /* External timer initialization functions */
 extern void MX_TIM1_Init(void);
 extern void MX_TIM8_Init(void);
 
 /*
  * Initialize PWM device
  */
 static int pwm_stm32_init(const struct device *dev)
 {
	 const struct pwm_stm32_config *config = dev->config;
	 int ret;
 
	 /* Initialize timer hardware */
	 if (!config->slave_enable) {
		 MX_TIM1_Init();
	 } else {
		 MX_TIM8_Init();
	 }
	 return 0;
	 /* Configure pinmux */
	 ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	 if (ret < 0) {
		 LOG_ERR("pinctrl setup failed (%d)", ret);
		 return ret;
	 }
 
	 /* Enable timer clock */
	 const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	 ret = clock_control_on(clk, (clock_control_subsys_t *)&config->pclken);
	 if (ret < 0) {
		 LOG_ERR("Could not turn on timer clock (%d)", ret);
		 return ret;
	 }
 
	 /* Timer base configuration */
	 LL_TIM_InitTypeDef tim_init = {
		 .CounterMode = LL_TIM_COUNTERMODE_CENTER_UP,
		 .Autoreload = config->timing_params[1],  /* ARR */
		 .Prescaler = config->timing_params[2],   /* PSC */
		 .RepetitionCounter = 0U,
		 .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1
	 };
 
	 if (LL_TIM_Init(config->timer, &tim_init) != SUCCESS) {
		 LOG_ERR("Could not initialize timer");
		 return -EIO;
	 }
 
	 /* Channel configuration */
	 LL_TIM_DisableARRPreload(config->timer);
	 if (!config->slave_enable) {
		 LL_TIM_SetClockSource(config->timer, LL_TIM_CLOCKSOURCE_INTERNAL);
	 }
 
	 /* Output compare configuration */
	 LL_TIM_OC_InitTypeDef tim_ocinit = {
		 .OCMode = LL_TIM_OCMODE_PWM1,
		 .OCState = LL_TIM_OCSTATE_DISABLE,
		 .OCNState = LL_TIM_OCSTATE_DISABLE,
		 .CompareValue = 0,
		 .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
		 .OCNPolarity = LL_TIM_OCPOLARITY_LOW,
		 .OCIdleState = LL_TIM_OCIDLESTATE_LOW,
		 .OCNIdleState = LL_TIM_OCIDLESTATE_LOW
	 };
 
	 /* Configure all channels */
	 for (int ch = LL_TIM_CHANNEL_CH1; ch <= LL_TIM_CHANNEL_CH3; ch++) {
		 LL_TIM_OC_EnablePreload(config->timer, ch);
		 LL_TIM_OC_Init(config->timer, ch, &tim_ocinit);
		 LL_TIM_OC_DisableFast(config->timer, ch);
	 }
 
	 /* Master/slave configuration */
	 if (!config->slave_enable) {
		 LL_TIM_OC_EnablePreload(config->timer, LL_TIM_CHANNEL_CH4);
		 LL_TIM_OC_Init(config->timer, LL_TIM_CHANNEL_CH4, &tim_ocinit);
		 LL_TIM_OC_DisableFast(config->timer, LL_TIM_CHANNEL_CH4);        
		 LL_TIM_SetTriggerOutput(config->timer, LL_TIM_TRGO_ENABLE);
		 LL_TIM_SetTriggerOutput2(config->timer, LL_TIM_TRGO2_OC4);
		 LL_TIM_EnableMasterSlaveMode(config->timer);
	 } else {
		 LL_TIM_SetTriggerInput(config->timer, LL_TIM_TS_ITR0);
		 LL_TIM_SetSlaveMode(config->timer, LL_TIM_SLAVEMODE_TRIGGER);
	 }
 
	 /* Break and dead-time configuration */
	 LL_TIM_BDTR_InitTypeDef brk_dt_init = {
		 .OSSRState = LL_TIM_OSSR_DISABLE,
		 .OSSIState = LL_TIM_OSSI_DISABLE,
		 .LockLevel = LL_TIM_LOCKLEVEL_OFF,
		 .DeadTime = config->timing_params[0],  /* Dead time */
		 .BreakState = LL_TIM_BREAK_DISABLE,
		 .BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH,
		 .BreakFilter = LL_TIM_BREAK_FILTER_FDIV1,
		 .Break2State = LL_TIM_BREAK2_DISABLE,
		 .Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH,
		 .Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1,
		 .AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE
	 };
	 LL_TIM_BDTR_Init(config->timer, &brk_dt_init);
 
	 LOG_INF("pwm_stm32_init Finish");
	 return 0;
 }
 
 /* Device instance macro */
 #define PMW_STM32_INIT(n) \
	 PINCTRL_DT_INST_DEFINE(n); \
	 static const struct pwm_stm32_config pwm_stm32_config_##n = { \
		 .timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)), \
		 .pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)), \
		 .timing_params = DT_INST_PROP(n, timing_params), \
		 .slave_enable = DT_INST_PROP(n, slave), \
		 .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
	 }; \
	 static const struct pwm_driver_api pwm_stm32_api_##n = { \
		 .start = pwm_stm32_start, \
		 .stop = pwm_stm32_stop, \
		 .set_phase_voltages = pwm_stm32_setduties, \
		 .set_phase_state = pwm_stm32_setstatus\
	 }; \
	 DEVICE_DT_INST_DEFINE(n, \
		 &pwm_stm32_init, \
		 NULL, NULL, \
		 &pwm_stm32_config_##n, \
		 PRE_KERNEL_1, \
		 CONFIG_PWMX_STM32_INIT_PRIORITY, \
		 &pwm_stm32_api_##n);
 
 /* Initialize all instances */
 DT_INST_FOREACH_STATUS_OKAY(PMW_STM32_INIT)