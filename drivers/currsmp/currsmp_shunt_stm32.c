#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#define DT_DRV_COMPAT st_stm32_currsmp_shunt
struct currsmp_shunt_stm32_config {
	ADC_TypeDef *adc;
	// struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
    void (*irq_cfg_func)(void);
};
extern void ADC_IRQHandler(void);
static void adc_isr(const struct device *dev)
{
    ADC_IRQHandler();
}

extern void MX_ADC1_Init(void);
void adc1_start(void);

static int adc_stm32_init(const struct device *dev)
{
    const struct currsmp_shunt_stm32_config *cfg = dev->config;
    MX_ADC1_Init();
    int ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return ret;
    }
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
    LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

    LL_ADC_SetOverSamplingScope(cfg->adc, LL_ADC_OVS_DISABLE);
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(cfg->adc, &ADC_InitStruct);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
    LL_ADC_REG_Init(cfg->adc, &ADC_REG_InitStruct);
    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV4;
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_INJ_SIMULT;
    ADC_CommonInitStruct.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_2CYCLES_5;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(cfg->adc), &ADC_CommonInitStruct);
    ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_CH4;
    ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS;
    ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
    ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
    LL_ADC_INJ_Init(cfg->adc, &ADC_INJ_InitStruct);
    LL_ADC_INJ_SetQueueMode(cfg->adc, LL_ADC_INJ_QUEUE_DISABLE);
    LL_ADC_SetOverSamplingScope(cfg->adc, LL_ADC_OVS_DISABLE);
    LL_ADC_INJ_SetTriggerEdge(cfg->adc, LL_ADC_INJ_TRIG_EXT_RISING);

    LL_ADC_DisableDeepPowerDown(cfg->adc);
    LL_ADC_EnableInternalRegulator(cfg->adc);
    __IO uint32_t wait_loop_index;
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0) { wait_loop_index--; }

    LL_ADC_REG_SetSequencerRanks(cfg->adc, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
    LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelPreselection(cfg->adc, LL_ADC_CHANNEL_3);

    LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_14);
    LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_14, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_14, LL_ADC_SINGLE_ENDED);

    LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_16);
    LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_16, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_16, LL_ADC_SINGLE_ENDED);

    LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_17);
    LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_17, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_17, LL_ADC_SINGLE_ENDED);

    // IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc1)),
    //             DT_IRQ(DT_NODELABEL(adc1), priority),
    //             adc_isr, NULL, 0);
    // irq_enable(DT_IRQN(DT_NODELABEL(adc1)));
    // 通过 currsmp1 的父节点（adc1）配置中断
    // IRQ_CONNECT(DT_IRQN(DT_PARENT(DT_NODELABEL(currsmp1))),  // 获取父节点 adc1 的中断号
    //           DT_IRQ(DT_PARENT(DT_NODELABEL(currsmp1)), priority),  // 获取父节点中断优先级
    //           adc_isr, NULL, 0);
    // irq_enable(DT_IRQN(DT_PARENT(DT_NODELABEL(currsmp1))));
    
    // 方法二：通过实例化宏获取父节点中断
    IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(0)),          // 获取实例0的父节点（adc1）中断号
                DT_IRQ(DT_INST_PARENT(0), priority), // 获取中断优先级
                adc_isr, NULL, 0);
    irq_enable(DT_IRQN(DT_INST_PARENT(0)));    
    adc1_start();

    return 0;
}

#define FIRST_WITH_IRQN_INTERNAL(n, irqn)			\
	COND_CODE_1(IS_EQ(irqn, DT_IRQN(DT_INST_PARENT(n))), (n,), (EMPTY,))

#define FIRST_WITH_IRQN(n)						\
	GET_ARG_N(1, LIST_DROP_EMPTY(DT_INST_FOREACH_STATUS_OKAY_VARGS(FIRST_WITH_IRQN_INTERNAL, \
								       DT_IRQN(DT_INST_PARENT(n)))))

#define HANDLE_IRQS(n, irqn)                                                                   \
	COND_CODE_1(IS_EQ(irqn, DT_IRQN(DT_INST_PARENT(n))), (adc_stm32_isr(DEVICE_DT_INST_GET(n));), \
		    (EMPTY))

#define ISR_FUNC(n) UTIL_CAT(adc_stm32_isr_, DT_IRQN(DT_INST_PARENT(n)))

#define GENERATE_ISR_CODE(n)						\
	static void ISR_FUNC(n)(void)					\
	{								\
		DT_INST_FOREACH_STATUS_OKAY_VARGS(HANDLE_IRQS, DT_IRQN(DT_INST_PARENT(n))) \
	}								\
									\
	static void UTIL_CAT(ISR_FUNC(n), _init)(void)			\
	{								\
		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(n)),			\
			    DT_IRQ(DT_INST_PARENT(n), priority), ISR_FUNC(n), \
			    NULL, 0);					\
		irq_enable(DT_IRQN(DT_INST_PARENT(n)));		\
	}

#define GENERATE_ISR(n)							\
	COND_CODE_1(IS_EQ(n, FIRST_WITH_IRQN(n)), (GENERATE_ISR_CODE(n)), (EMPTY))

DT_INST_FOREACH_STATUS_OKAY(GENERATE_ISR)



// 实例化驱动宏
#define CURRSMP_SHUNT_STM32_CURRSMP_SHUNT_INIT(n) \
    PINCTRL_DT_INST_DEFINE(n); \
    static const struct currsmp_shunt_stm32_config currsmp_shunt_stm32_config_##n = { \
        .adc = (ADC_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)), \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
    }; \
    DEVICE_DT_INST_DEFINE(n, adc_stm32_init, NULL, NULL, \
        &currsmp_shunt_stm32_config_##n, POST_KERNEL, 70, NULL);
// 为所有状态为 "okay" 的实例生成设备
DT_INST_FOREACH_STATUS_OKAY(CURRSMP_SHUNT_STM32_CURRSMP_SHUNT_INIT)