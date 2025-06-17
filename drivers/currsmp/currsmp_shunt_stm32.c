/**
 * @addtogroup currsmp_drivers
 * @{
 */

 #include "stm32h7xx_ll_adc.h"
#include <math.h>
#include <stdint.h>
#include <sys/_intsup.h>
#include <zephyr/logging/log.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/adc.h>
 #include <zephyr/irq.h>
 #include <zephyr/drivers/pinctrl.h>
 #include <zephyr/drivers/clock_control/stm32_clock_control.h>
 #include <stm32h7xx_ll_gpio.h>
 #include <drivers/currsmp.h>
 #include <algorithmlib/filter.h>
 LOG_MODULE_REGISTER(currsmp_shunt_stm32, LOG_LEVEL_DBG);
 #define DT_DRV_COMPAT st_stm32_currsmp_shunt
 
 /** @brief Configuration structure for STM32 shunt current sampling driver. */
 struct currsmp_shunt_stm32_config {
     /** ADC peripheral instance. */
     ADC_TypeDef *adc;
     /** Clock configuration for ADC. */
     struct stm32_pclken pclken;
     /** Pin control configuration. */
     const struct pinctrl_dev_config *pcfg;
     /** Flag indicating slave mode. */
     uint32_t slave_mode_flag;
     /** IRQ configuration function. */
     void (*irq_cfg_func)(void);
 };
 
 /** @brief Data structure for STM32 shunt current sampling driver. */
 struct currsmp_shunt_stm32_data {
     /** Regulation callback function. */
     currsmp_regulation_cb_t regulation_cb;
     /** Regulation callback context. */
     void *regulation_ctx;
     /** ADC channel data for phase A. */
     uint32_t adc_channl_a;
     /** ADC channel data for phase B. */
     uint32_t adc_channl_b;
     /** ADC channel data for phase C. */
     uint32_t adc_channl_c;

     uint32_t adc_f_channl_a;
     /** ADC channel data for phase B. */
     uint32_t adc_f_channl_b;
     /** ADC channel data for phase C. */
     uint32_t adc_f_channl_c;

     int16_t adc_average;

    moving_avg_t *average_filter;
    moving_avg_t *cha_filter;
    moving_avg_t *chb_filter;
    moving_avg_t *chc_filter;

};
 
 /** @brief ADC interrupt service routine.
  *
  * @param[in] dev Current sampling device.
  */
 static void inline adc_stm32_isr(const struct device *dev)
 {
     const struct currsmp_shunt_stm32_config* cfg = dev->config;
     struct currsmp_shunt_stm32_data* data = dev->data;
     if (LL_ADC_IsActiveFlag_JEOS(cfg->adc)) 
     {   
       LL_ADC_ClearFlag_JEOS(cfg->adc);
       if(data->regulation_cb)
       {
         data->regulation_cb(data->regulation_ctx);
       }
     }
 }
 
 /** @brief Configure current sampling device.
  *
  * @param[in] dev Current sampling device.
  * @param[in] regulation_cb Callback called on each regulation cycle.
  * @param[in] ctx Callback context.
  */
 static void currsmp_shunt_stm32_configure(const struct device *dev,
                       currsmp_regulation_cb_t regulation_cb,
                       void *ctx)
 {
     struct currsmp_shunt_stm32_data *data = dev->data;
 
     data->regulation_cb = regulation_cb;
     data->regulation_ctx = ctx;
 }
 
 /** @brief Get phase currents.
  *
  * @param[in] dev Current sampling device.
  * @param[out] curr Pointer where phase currents will be stored.
  */
 static void currsmp_shunt_stm32_get_currents(const struct device *dev,
                          struct currsmp_curr *curr)
 {
    const struct currsmp_shunt_stm32_config *cfg = dev->config;
    struct currsmp_shunt_stm32_data *data = dev->data;
      
    data->adc_channl_a = LL_ADC_INJ_ReadConversionData12(cfg->adc, LL_ADC_INJ_RANK_1);
    data->adc_channl_b = LL_ADC_INJ_ReadConversionData12(cfg->adc, LL_ADC_INJ_RANK_2);
    data->adc_channl_c = LL_ADC_INJ_ReadConversionData12(cfg->adc, LL_ADC_INJ_RANK_3);

    data->adc_f_channl_a = moving_avg_update(data->cha_filter,data->adc_channl_a);//TODO
    data->adc_f_channl_b = moving_avg_update(data->chb_filter,data->adc_channl_b);//TODO
    data->adc_f_channl_c = moving_avg_update(data->chc_filter,data->adc_channl_c);//TODO
    //计算偏置
    data->adc_average = moving_avg_update(data->average_filter, (data->adc_channl_a+data->adc_channl_b+data->adc_channl_c)/3);
    // 2. 改进转换公式
    const float scale = 0.006012f;
    // curr->i_a = ((int16_t)(data->adc_channl_a - data->adc_average)) * scale;
    // curr->i_b = ((int16_t)(data->adc_channl_b - data->adc_average)) * scale;
    // curr->i_c = ((int16_t)(data->adc_channl_c - data->adc_average)) * scale;
    curr->i_a = ((int16_t)(data->adc_channl_a - 2048)) * scale;
    curr->i_b = ((int16_t)(data->adc_channl_b - 2048)) * scale;
    curr->i_c = ((int16_t)(data->adc_channl_c - 2048)) * scale;
 }
 static void currsmp_shunt_stm32_get_bus_vol_curr(const struct device* dev,float *bus_vol,float *bus_curr)
 {
     LL_ADC_REG_StartConversion(ADC1);
     while(!LL_ADC_IsActiveFlag_EOC(ADC1));
     uint32_t ch14_value = LL_ADC_REG_ReadConversionData16(ADC1);
     *bus_vol = ch14_value*0.01632f;// (3.3f/4096*ch14_value)*(100.0f/(104.7f));
 
     LL_ADC_REG_StartConversion(ADC2);
     while(!LL_ADC_IsActiveFlag_EOC(ADC2));
     uint32_t ch4_value = LL_ADC_REG_ReadConversionData16(ADC2);
     *bus_curr =  ch4_value*0.01632f;// (3.3f/4096*ch14_value)*(100.0f/(104.7f));         
 }
  
 /** @brief STM32 Shunt Current Sampling Driver API. */
 static const struct currsmp_driver_api currsmp_shunt_stm32_driver_api = {
     .configure = currsmp_shunt_stm32_configure,
     .get_currents = currsmp_shunt_stm32_get_currents,
     .get_bus_volcurr = currsmp_shunt_stm32_get_bus_vol_curr,
 };
 
 /** @brief Initialization function for STM32 shunt current sampling driver.
  *
  * @param[in] dev Current sampling device.
  * @return 0 on success, negative error code on failure.
  */
 static int currsmp_shunt_stm32_init(const struct device *dev)
 {
     const struct currsmp_shunt_stm32_config *cfg = dev->config;
     int ret;
 
     const struct device *clk;
     clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
     ret = clock_control_on(clk, (clock_control_subsys_t *)&cfg->pclken);
     if (ret < 0) {
         return ret;
     }
 
     ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
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
 
     if(!cfg->slave_mode_flag)
     {
         ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV4;
         ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_INJ_SIMULT;
         ADC_CommonInitStruct.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_2CYCLES_5;
         LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(cfg->adc), &ADC_CommonInitStruct);
         ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_CH4;
         ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS;
         ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
         ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
         LL_ADC_INJ_Init(cfg->adc, &ADC_INJ_InitStruct);
     }else{
         ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS;
         ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
         ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
         LL_ADC_INJ_Init(cfg->adc, &ADC_INJ_InitStruct);        
     }
 
     LL_ADC_INJ_SetQueueMode(cfg->adc, LL_ADC_INJ_QUEUE_DISABLE);
     LL_ADC_SetOverSamplingScope(cfg->adc, LL_ADC_OVS_DISABLE);
     LL_ADC_INJ_SetTriggerEdge(cfg->adc, LL_ADC_INJ_TRIG_EXT_FALLING);
 
     LL_ADC_DisableDeepPowerDown(cfg->adc);
     LL_ADC_EnableInternalRegulator(cfg->adc);
     __IO uint32_t wait_loop_index;
     wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
     while(wait_loop_index != 0) { wait_loop_index--; }
     if(!cfg->slave_mode_flag)
     {
        LL_ADC_REG_SetSequencerRanks(cfg->adc, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_15);
        LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_1CYCLE_5);
        LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_15, LL_ADC_SINGLE_ENDED);
        LL_ADC_SetChannelPreselection(cfg->adc, LL_ADC_CHANNEL_15);
     
         LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_14);
         LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_14, LL_ADC_SAMPLINGTIME_1CYCLE_5);
         LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_14, LL_ADC_SINGLE_ENDED);
     
         LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_16);
         LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_16, LL_ADC_SAMPLINGTIME_1CYCLE_5);
         LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_16, LL_ADC_SINGLE_ENDED);
     
         LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_17);
         LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_17, LL_ADC_SAMPLINGTIME_1CYCLE_5);
         LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_17, LL_ADC_SINGLE_ENDED);    
 
         LL_ADC_SetChannelPreselection(cfg->adc,LL_ADC_CHANNEL_14);
         LL_ADC_SetChannelPreselection(cfg->adc,LL_ADC_CHANNEL_16);
         LL_ADC_SetChannelPreselection(cfg->adc,LL_ADC_CHANNEL_17);        
     }else{
         LL_ADC_REG_SetSequencerRanks(cfg->adc, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
         LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_1CYCLE_5);
         LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED);
         LL_ADC_SetChannelPreselection(cfg->adc, LL_ADC_CHANNEL_4);    

         LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_9);
         LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_1CYCLE_5);
         LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);
       
         LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_5);
         LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_1CYCLE_5);
         LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);
       
         LL_ADC_INJ_SetSequencerRanks(cfg->adc, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_8);
         LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_1CYCLE_5);
         LL_ADC_SetChannelSingleDiff(cfg->adc, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);       
         
         LL_ADC_SetChannelPreselection(cfg->adc,LL_ADC_CHANNEL_8);
         LL_ADC_SetChannelPreselection(cfg->adc,LL_ADC_CHANNEL_9);
         LL_ADC_SetChannelPreselection(cfg->adc,LL_ADC_CHANNEL_5);
     }
 
     if(!cfg->slave_mode_flag)
     {
         if (cfg->irq_cfg_func != NULL) {
             cfg->irq_cfg_func();
         }
         LL_ADC_Disable(cfg->adc);
         LL_ADC_StartCalibration(cfg->adc,LL_ADC_CALIB_OFFSET,LL_ADC_SINGLE_ENDED);
         while (LL_ADC_IsCalibrationOnGoing(cfg->adc));
         LL_ADC_Enable(cfg->adc);
         while (LL_ADC_IsActiveFlag_ADRDY(cfg->adc) == 0);
         LL_ADC_EnableIT_JEOS(cfg->adc);
         LL_ADC_INJ_StartConversion(cfg->adc);
     }else{
         LL_ADC_Disable(cfg->adc);
         LL_ADC_StartCalibration(cfg->adc,LL_ADC_CALIB_OFFSET,LL_ADC_SINGLE_ENDED);
         while (LL_ADC_IsCalibrationOnGoing(cfg->adc));
          LL_ADC_Enable(cfg->adc);
          while (LL_ADC_IsActiveFlag_ADRDY(cfg->adc) == 0);      
         LL_ADC_DisableIT_JEOS(ADC2);        
     }
     LOG_INF("currsmp_shunt_stm32_init");

     /*filter init*/
     struct currsmp_shunt_stm32_data* data = dev->data;
     moving_avg_init(data->average_filter,NULL,0);
     moving_avg_init(data->cha_filter,NULL,0);
     moving_avg_init(data->chb_filter,NULL,0);
     moving_avg_init(data->chc_filter,NULL,0);

     return 0;
 }
 float currsmp_get_busvol(void)
 {
   LL_ADC_REG_StartConversion(ADC1);
   while(!LL_ADC_IsActiveFlag_EOC(ADC1));
   uint32_t ch14_value = LL_ADC_REG_ReadConversionData16(ADC1);
   return ch14_value*0.01632f;// (3.3f/4096*ch14_value)*(100.0f/(104.7f)); 
 } 
 /** @internal Helper macro to define the first instance with a specific IRQn. */
 #define FIRST_WITH_IRQN_INTERNAL(n, irqn)			\
     COND_CODE_1(IS_EQ(irqn, DT_IRQN(DT_INST_PARENT(n))), (n,), (EMPTY,))
 
 /** @internal Helper macro to get the first instance with a specific IRQn. */
 #define FIRST_WITH_IRQN(n)						\
     GET_ARG_N(1, LIST_DROP_EMPTY(DT_INST_FOREACH_STATUS_OKAY_VARGS(FIRST_WITH_IRQN_INTERNAL, \
                                        DT_IRQN(DT_INST_PARENT(n)))))
 
 /** @internal Helper macro to handle IRQs for a specific instance. */
 #define HANDLE_IRQS(n, irqn)                                                                   \
     COND_CODE_1(IS_EQ(irqn, DT_IRQN(DT_INST_PARENT(n))), (adc_stm32_isr(DEVICE_DT_INST_GET(n));), \
             (EMPTY))
 
 /** @internal Helper macro to generate ISR code for a specific instance. */
 #define ISR_FUNC(n) UTIL_CAT(adc_stm32_isr_, DT_IRQN(DT_INST_PARENT(n)))
 
 /** @internal Generate ISR code for a specific instance. */
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
 
 /** @internal Generate ISR code for a specific instance. */
 #define GENERATE_ISR(n)							\
     COND_CODE_1(IS_EQ(n, FIRST_WITH_IRQN(n)), (GENERATE_ISR_CODE(n)), (EMPTY))
 
 /** @internal Generate device instances. */
 DT_INST_FOREACH_STATUS_OKAY(GENERATE_ISR)
 
 /** @internal Helper macro to define the IRQ function for a specific instance. */
 #define CURRSMP_SHUNT_STM32_IRQ_FUNC(n)                                                                  \
     .irq_cfg_func = COND_CODE_1(IS_EQ(n, FIRST_WITH_IRQN(n)),                          \
                     (UTIL_CAT(ISR_FUNC(n), _init)), (NULL)),
 
 /** @brief Device instance definition for STM32 shunt current sampling driver. */
 #define CURRSMP_SHUNT_STM32_CURRSMP_SHUNT_INIT(n)			\
                                     \
     PINCTRL_DT_INST_DEFINE(n);					\
     static const struct currsmp_shunt_stm32_config currsmp_shunt_stm32_config_##n = { \
         .adc = (ADC_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)),	\
         .pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)),	\
         .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
         .slave_mode_flag = DT_INST_PROP(n, adc_slave),\
         CURRSMP_SHUNT_STM32_IRQ_FUNC(n)			\
     };								\
     static int16_t buffer_cha_##n[32];\
     static moving_avg_t filter_cha_##n = {\
        .buffer = (buffer_cha_##n),\
        .size = 32\
     };\
     static int16_t buffer_chb_##n[32];\
     static moving_avg_t filter_chb_##n = {\
        .buffer = (buffer_chb_##n),\
        .size = 32\
     };\
     static int16_t buffer_chc_##n[32];\
     static moving_avg_t filter_chc_##n = {\
        .buffer = (buffer_chc_##n),\
        .size = 32\
     };\
     static int16_t buffer_average_##n[32];\
     static moving_avg_t filter_average_##n = {\
        .buffer = (buffer_average_##n),\
        .size = 32\
     };\
     static struct currsmp_shunt_stm32_data currsmp_shunt_stm32_data_##n = {\
        .cha_filter = &(filter_cha_##n),\
        .chb_filter = &(filter_chb_##n),\
        .chc_filter = &(filter_chc_##n),\
        .average_filter = &(filter_average_##n),\
    }; \
                                     \
     DEVICE_DT_INST_DEFINE(n, &currsmp_shunt_stm32_init,\
                   NULL,	\
                   &currsmp_shunt_stm32_data_##n,\
                   &currsmp_shunt_stm32_config_##n,\
                   PRE_KERNEL_1,\
                   CONFIG_CURRSMP_INIT_PRIORITY, \
                   &currsmp_shunt_stm32_driver_api);
 
 /** @brief Generate device instances. */
 DT_INST_FOREACH_STATUS_OKAY(CURRSMP_SHUNT_STM32_CURRSMP_SHUNT_INIT)
 
 /** @} */