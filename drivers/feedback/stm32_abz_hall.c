/*
 * STM32 ABZ Hall Sensor Driver
 *
 * Features:
 * - ABZ encoder interface
 * - Hall sensor position detection  
 * - Speed calculation
 */

 #include "zephyr/device.h"
#include "stm32h7xx_ll_tim.h"
#include <sys/_intsup.h>
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
 #include <lib/bldcmotor/motor_Parameter.h>

 LOG_MODULE_REGISTER(abz_hall_stm32, LOG_LEVEL_DBG);

#define ABZ_ENCODER_LINES_X2            (ABZ_ENCODER_LINES*2)

static inline float _normalize_angle(float angle)
{
  float a = fmodf(angle, 360.0f);
  return a >= 0 ? a : (a + 360.0f);  
}
 /* Driver configuration structure */
 struct abz_hall_stm32_config {
     uint32_t lines;                     /* Encoder lines per revolution */
     uint32_t pole_pairs;                /* Motor pole pairs */
     TIM_TypeDef *timer;                 /* Timer instance */
     struct stm32_pclken pclken;         /* Clock enable info */
     const struct pinctrl_dev_config *pcfg; /* Pin configuration */
     struct gpio_dt_spec hu_gpio;        /* Hall U phase GPIO */
     struct gpio_dt_spec hv_gpio;        /* Hall V phase GPIO */ 
     struct gpio_dt_spec hw_gpio;        /* Hall W phase GPIO */
 };
 
 /* Driver runtime data */
 struct hall_data_t{
    uint8_t cur_sect;
    uint8_t pre_sect;
    float base_angle[7];
    float eangle;
    float eomega;
    float rel_odom;
    int8_t dir;

 };
 struct abz_hall_stm32_data {
     int overflow;                       /* Timer overflow count */
     float mangle_ratio;                 /* Mechanical angle ratio */
     float eangle_ratio;                 /* Electrical angle ratio */
     struct gpio_callback gpio_cb;       /* GPIO callback */
     const struct device *dev;           /* Device instance */
     struct hall_data_t hall;
 };
 static void hall_angleupdate(const void* obj,uint8_t cur_sect) 
 {
    struct hall_data_t* hall = (struct hall_data_t*)obj;
    hall->cur_sect = cur_sect;
    switch(hall->pre_sect)
    {
    /****************************SECTION 6***********************************/    
        case 6:
            if(cur_sect == 4) {//正转
                hall->dir = 1;
            }else if(cur_sect == 2){//
                hall->dir = -1;
            }else if (cur_sect == 6) {//仍在当前扇区
             
            }else{
                //错误
            }
        break;  
    /****************************SECTION 4***********************************/    
        case 4:
            if(cur_sect == 5) {//正转
                hall->dir = 1;
            }else if(cur_sect == 6){//
                hall->dir = -1;
            }else if (cur_sect == 4) {//仍在当前扇区
             
            }else{
                //错误
            }
        break;
    /****************************SECTION 5***********************************/    
        case 5:
            if(cur_sect == 1) {//正转
                hall->dir = 1;
            }else if(cur_sect == 4){//
                hall->dir = -1;
            }else if (cur_sect == 5) {//仍在当前扇区
            
            }else{
                //错误
            }
        break;
    /****************************SECTION 1***********************************/    
        case 1:
            if(cur_sect == 3) {//正转
                hall->dir = 1;
            }else if(cur_sect == 5){//
                hall->dir = -1;
            }else if (cur_sect == 1) {//仍在当前扇区
             
            }else{
                //错误
            }
        break;
    /****************************SECTION 3***********************************/    
        case 3:
            if(cur_sect == 2) {//正转
                hall->dir = 1;
            }else if(cur_sect == 1){//
                hall->dir = -1;
            }else if (cur_sect == 3) {//仍在当前扇区
            
            }else{
                //错误
            }        
        break;
    /****************************SECTION 2***********************************/    
        case 2:
            if(cur_sect == 6) {//正转
                hall->dir = 1;
            }else if(cur_sect == 3){//
                hall->dir = -1;
            }else if (cur_sect == 2) {//仍在当前扇区
            
            }else{
                //错误
            }        
        break;
    /***********************ERR SECTION***********************************/    
        default:
        break;        
    }
    float temp;
    // if(cur_sect == 6)
    {
        if(hall->dir == 1)
        {
            temp =  hall->base_angle[cur_sect] + HALL_SENSOR_POSITIVE_OFFSET;//根据实际情况正转角度进行补偿
            hall->eangle = temp;
        }else if(hall->dir == -1){
            hall->eangle = hall->base_angle[cur_sect] + HALL_SENSOR_NEGATIVE_OFFSET;
        }
    }
    hall->pre_sect = cur_sect;
 }
 /* API implementation */
 static float abz_stm32_get_eangle(const struct device *dev)
 { 
    /* TODO: Implement electrical angle calculation */
    const struct abz_hall_stm32_config *cfg = dev->config;
    const struct abz_hall_stm32_data *data = dev->data;
    struct hall_data_t* hall = (struct hall_data_t*)(&data->hall);
   
    int32_t delt_cnt =  LL_TIM_GetCounter(cfg->timer) - ABZ_ENCODER_LINES;
    LL_TIM_SetCounter(cfg->timer,ABZ_ENCODER_LINES);
    float diff = (delt_cnt)*ABZ_ENCODER_RESOLUTION;

    hall->eangle += diff;
    hall->eomega = (diff)/57.2957795131f;
    hall->rel_odom += (diff/57.295779513f); 
    hall->eangle = _normalize_angle(hall->eangle );
    return hall->eangle;
 }
static void abz_stm32_set_rel_odom(const struct device *dev)
{
    const struct abz_hall_stm32_data *data = dev->data;
    struct hall_data_t* hall = (struct hall_data_t*)(&data->hall);
    hall->rel_odom = 0.0f;
}
 static float abz_stm32_get_mangle(const struct device *dev)
 {
     /* TODO: Implement mechanical angle calculation */
     return 0.0f;
 }
 
 static float abz_stm32_get_rads(const struct device *dev)
 {
     /* TODO: Implement eomega calculation */
    const struct abz_hall_stm32_data *data = dev->data;
    struct hall_data_t* hall = (struct hall_data_t*)(&data->hall);     
    return hall->eomega;
 }
 
 static float abz_stm32_get_rel_odom(const struct device *dev)
 {
     /* TODO: Implement position calculation */
     const struct abz_hall_stm32_data *data = dev->data;
     struct hall_data_t* hall = (struct hall_data_t*)(&data->hall);
     return hall->rel_odom;
 }

 static int abz_stm32_calibrate_eangle(const struct device *dev)
 {
    /* TODO: Implement position calculation */
    const struct abz_hall_stm32_data *data = dev->data;
    struct hall_data_t* hall = (struct hall_data_t*)(&data->hall);

    const struct abz_hall_stm32_config *cfg = dev->config;
    int hu_state = gpio_pin_get_dt(&cfg->hu_gpio);
    int hv_state = gpio_pin_get_dt(&cfg->hv_gpio); 
    int hw_state = gpio_pin_get_dt(&cfg->hw_gpio);     
    uint8_t cur_sect;
    cur_sect = hu_state<<2|hw_state<<1|hv_state;          
    hall->cur_sect = cur_sect;
    hall->eangle = hall->base_angle[cur_sect] + 30.0f;
    LL_TIM_SetCounter(cfg->timer,ABZ_ENCODER_LINES);
    return 0;
 }

  /*
  * Enable hall sensor interface
  */
  static void abz_hall_stm32_enable(const struct device *dev)
  {
      const struct abz_hall_stm32_config *cfg = dev->config;
      uint8_t ret;
      LOG_INF("device name: %s", dev->name);
      /* Start encoder timer */
      LL_TIM_EnableCounter(cfg->timer);
      /* Configure hall sensor interrupts */
      ret = gpio_pin_interrupt_configure_dt(&cfg->hu_gpio, GPIO_INT_EDGE_BOTH);
      ret |= gpio_pin_interrupt_configure_dt(&cfg->hv_gpio, GPIO_INT_EDGE_BOTH);
      ret |= gpio_pin_interrupt_configure_dt(&cfg->hw_gpio, GPIO_INT_EDGE_BOTH);
      if (ret < 0) {
          LOG_ERR("Failed to configure interrupts");
      }
      /* Update sector information */

  }
  
 /* Driver API structure */
 static const struct feedback_driver_api driver_feedback = {
     .get_rads = abz_stm32_get_rads,
     .get_eangle = abz_stm32_get_eangle,
     .get_mangle = abz_stm32_get_mangle,
     .calibration = abz_stm32_calibrate_eangle,
     .get_rel_odom = abz_stm32_get_rel_odom,
     .feedback_enable = abz_hall_stm32_enable,
     .set_rel_odom = abz_stm32_set_rel_odom,
 };
 
 /*
  * Hall sensor GPIO callback
  * Triggered on any hall sensor state change
  */
 static void hall_gpio_callback(const struct device *port,
                              struct gpio_callback *cb,
                              uint32_t pins)
 {
     struct abz_hall_stm32_data *data = CONTAINER_OF(cb, struct abz_hall_stm32_data, gpio_cb);
     const struct device *dev = data->dev;
     const struct abz_hall_stm32_config *cfg = dev->config;
 
     /* Read current hall states */
     int hu_state = gpio_pin_get_dt(&cfg->hu_gpio);
     int hv_state = gpio_pin_get_dt(&cfg->hv_gpio); 
     int hw_state = gpio_pin_get_dt(&cfg->hw_gpio);
     
    //  LOG_DBG("Hall states - HALL_VAL:%d", hu_state<<2|hv_state<<1|hw_state);
 
     /* Update sector information */
     uint8_t cur_sect;
     cur_sect = hu_state<<2|hw_state<<1|hv_state;     
     hall_angleupdate(&(data->hall),cur_sect);

 }
 
 /*
  * Initialize ABZ encoder interface
  */
 static int abz_stm32_init(const struct device *dev)
 {
     const struct abz_hall_stm32_config *config = dev->config;
     int ret;
 
     /* Apply pin configuration */
     ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
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
 
     /* Configure timer in encoder mode */
     LL_TIM_InitTypeDef TIM_InitStruct = {
         .Prescaler = 0,
         .CounterMode = LL_TIM_COUNTERMODE_UP,
         .Autoreload = ABZ_ENCODER_LINES_X2,//config->lines,
         .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1
     };
     LL_TIM_Init(config->timer, &TIM_InitStruct);
     LL_TIM_DisableARRPreload(config->timer);
     LL_TIM_SetEncoderMode(config->timer, LL_TIM_ENCODERMODE_X4_TI12);
     
     /* Channel 1 configuration */
     LL_TIM_IC_SetActiveInput(config->timer, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
     LL_TIM_IC_SetPrescaler(config->timer, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
     LL_TIM_IC_SetFilter(config->timer, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
     LL_TIM_IC_SetPolarity(config->timer, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
 
     /* Channel 2 configuration */
     LL_TIM_IC_SetActiveInput(config->timer, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
     LL_TIM_IC_SetPrescaler(config->timer, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
     LL_TIM_IC_SetFilter(config->timer, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
     LL_TIM_IC_SetPolarity(config->timer, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
 
     /* Trigger configuration */
     LL_TIM_SetTriggerOutput(config->timer, LL_TIM_TRGO_RESET);
     LL_TIM_DisableMasterSlaveMode(config->timer);
 
     return 0;
 }
 

 /*
  * Initialize hall sensor interface
  */
 static int hall_stm32_init(const struct device *dev)
 {
     const struct abz_hall_stm32_config *cfg = dev->config;
     struct abz_hall_stm32_data *data = dev->data;
     int ret;
     uint32_t pin_mask = BIT(cfg->hu_gpio.pin) | BIT(cfg->hv_gpio.pin) | BIT(cfg->hw_gpio.pin);
 
     /* Store device instance */
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
     struct hall_data_t *hall = &(data->hall);
     hall->base_angle[1] = SCETION_1_BASEANGLE;  
     hall->base_angle[2] = SCETION_2_BASEANGLE;
     hall->base_angle[3] = SCETION_3_BASEANGLE;
     hall->base_angle[4] = SCETION_4_BASEANGLE;
     hall->base_angle[5] = SCETION_5_BASEANGLE;
     hall->base_angle[6] = SCETION_6_BASEANGLE;
     hall->eangle = 0.0f;
     return 0;
 }
 
 /*
  * Main initialization function
  */
 static int stm32_abz_hall_init(const struct device *dev)
 {
     abz_stm32_init(dev);
     hall_stm32_init(dev);
     LOG_INF("stm32_abz_hall_init Finish");
     return 0;
 }
 
 /* Device instance macro */
 #define ABZ_HALL_STM32_INIT(n)                      \
     PINCTRL_DT_INST_DEFINE(n);                      \
     static const struct abz_hall_stm32_config abz_hall_stm32_cfg_##n = {   \
         .timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)), \
         .pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)),   \
         .lines = DT_INST_PROP(n, lines),            \
         .pole_pairs = DT_INST_PROP(n, pole_pairs),      \
         .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),      \
         .hu_gpio = GPIO_DT_SPEC_INST_GET(n, hu_gpios), \
         .hv_gpio = GPIO_DT_SPEC_INST_GET(n, hv_gpios), \
         .hw_gpio = GPIO_DT_SPEC_INST_GET(n, hw_gpios), \
     };                              \
     static struct abz_hall_stm32_data abz_hall_stm32_data_##n = {};        \
                                      \
     DEVICE_DT_INST_DEFINE(n, stm32_abz_hall_init, NULL,            \
                &abz_hall_stm32_data_##n,           \
                &abz_hall_stm32_cfg_##n,            \
                PRE_KERNEL_1, CONFIG_FEEDBACK_INIT_PRIORITY, \
                &driver_feedback           \
      );
 
 /* Initialize all instances */
 DT_INST_FOREACH_STATUS_OKAY(ABZ_HALL_STM32_INIT)