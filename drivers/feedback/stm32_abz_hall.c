/*
 * STM32 ABZ Hall Sensor Driver
 *
 * Features:
 * - ABZ encoder interface
 * - Hall sensor position detection  
 * - Speed calculation
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
 #define SCETION_6_BASEANGLE             (0.3416f)
 #define SCETION_4_BASEANGLE             (1.4624f)
 #define SCETION_5_BASEANGLE             (2.4623f)
 #define SCETION_1_BASEANGLE             (3.6222f)
 #define SCETION_3_BASEANGLE             (4.5830f)
 #define SCETION_2_BASEANGLE             (5.5344f)

 #define HALL_POSITIVE_OFFSET            (0.0f)
 #define HALL_NEGATIVE_OFFSET            (0.0f)

 #define _2PI             6.2831852f
 #define PI               3.14159260f

 #define ABZ_ENCODER_LINES_HALF          (2500)
 #define ABZ_ENCODER_LINES               (5000)//编码器线数
 #define ABZ_ENCODER_RESOLUTION          (0.00628f)//编码器分辨率 2*pi/5000

static float _normalize_angle(float angle)
{
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + 2.0f*PI);  
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
 typedef struct _sect
 {
     /* data */
     uint8_t last_sect;
     uint8_t next_sect;
     float angle;
     float diff;
 }sect_t;
 
 struct hall_data_t{
    uint8_t pre_sect;
    uint8_t err_count;
    float base_angle[6];
    float realcacle_angle;
    float pre_angle;
    float speed;
    sect_t positive_sect[7];
    sect_t negative_sect[7];    
 };
 struct abz_hall_stm32_data {
     int overflow;                       /* Timer overflow count */
     float mangle_ratio;                 /* Mechanical angle ratio */
     float eangle_ratio;                 /* Electrical angle ratio */
     struct gpio_callback gpio_cb;       /* GPIO callback */
     const struct device *dev;           /* Device instance */
     uint8_t cur_sect;                   /* Current hall sector */
     uint8_t pre_sect;                   /* Previous hall sector */
     struct hall_data_t hall;
 };
 static void hall_angleupdate(const void* obj,uint8_t cur_sect) 
 {
    struct hall_data_t* hall = (struct hall_data_t*)obj;
    sect_t *psect;
    switch(hall->pre_sect)
    {
    /****************************SECTION 6***********************************/    
        case 6:
            if(cur_sect == 4) {//正转
                psect = (sect_t *)hall->positive_sect;
            }else if(cur_sect == 2){//
                psect = (sect_t *)hall->negative_sect;
            }else if (cur_sect == 6) {//仍在当前扇区
             
            }else{
                //错误
            }
        break;  
    /****************************SECTION 4***********************************/    
        case 4:
            if(cur_sect == 5) {//正转
                psect = (sect_t *)hall->positive_sect;
            }else if(cur_sect == 6){//
                psect = (sect_t *)hall->negative_sect;
            }else if (cur_sect == 4) {//仍在当前扇区
             
            }else{
                //错误
            }
        break;
    /****************************SECTION 5***********************************/    
        case 5:
            if(cur_sect == 1) {//正转
                psect = (sect_t *)hall->positive_sect;
            }else if(cur_sect == 4){//
                psect = (sect_t *)hall->negative_sect;
            }else if (cur_sect == 5) {//仍在当前扇区
            
            }else{
                //错误
            }
        break;
    /****************************SECTION 1***********************************/    
        case 1:
            if(cur_sect == 3) {//正转
                psect = (sect_t *)hall->positive_sect;
            }else if(cur_sect == 5){//
                psect = (sect_t *)hall->negative_sect;
            }else if (cur_sect == 1) {//仍在当前扇区
             
            }else{
                //错误
            }
        break;
    /****************************SECTION 3***********************************/    
        case 3:
            if(cur_sect == 2) {//正转
                psect = (sect_t *)hall->positive_sect;
            }else if(cur_sect == 1){//
                psect = (sect_t *)hall->negative_sect;
            }else if (cur_sect == 3) {//仍在当前扇区
            
            }else{
                //错误
            }        
        break;
    /****************************SECTION 2***********************************/    
        case 2:
            if(cur_sect == 6) {//正转
                psect = (sect_t *)hall->positive_sect;
            }else if(cur_sect == 3){//
                psect = (sect_t *)hall->negative_sect;
            }else if (cur_sect == 2) {//仍在当前扇区
            
            }else{
                //错误
            }        
        break;
    /***********************ERR SECTION***********************************/    
        default:
        break;        
    }

    hall->realcacle_angle = psect[cur_sect].angle;

    hall->pre_sect = cur_sect;
 }
 /* API implementation */
 static float abz_stm32_get_eangle(const struct device *dev)
 { 
    /* TODO: Implement electrical angle calculation */
    const struct abz_hall_stm32_config *cfg = dev->config;
    const struct abz_hall_stm32_data *data = dev->data;
    struct hall_data_t* hall = (struct hall_data_t*)(&data->hall);
   
     int32_t delt_cnt =  LL_TIM_GetCounter(cfg->timer) - ABZ_ENCODER_LINES_HALF;
     LL_TIM_SetCounter(cfg->timer,ABZ_ENCODER_LINES_HALF);
     float diff = (delt_cnt)*ABZ_ENCODER_RESOLUTION;

     hall->realcacle_angle += diff;     
     hall->speed = (hall->realcacle_angle - hall->pre_angle)/95493.0f;
     hall->pre_angle = hall->realcacle_angle;
     return hall->realcacle_angle;
 }
 
 static float abz_stm32_get_mangle(const struct device *dev)
 {
     /* TODO: Implement mechanical angle calculation */
     return 0.0f;
 }
 
 static float abz_stm32_get_rads(const struct device *dev)
 {
     /* TODO: Implement speed calculation */
    const struct abz_hall_stm32_data *data = dev->data;
    struct hall_data_t* hall = (struct hall_data_t*)(&data->hall);     
    return hall->speed;
 }
 
 static float abz_stm32_get_position(const struct device *dev)
 {
     /* TODO: Implement position calculation */
     return 0.0f;
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
  }
  
 /* Driver API structure */
 static const struct feedback_driver_api driver_feedback = {
     .get_rads = abz_stm32_get_rads,
     .get_eangle = abz_stm32_get_eangle,
     .get_mangle = abz_stm32_get_mangle,
     .get_position = abz_stm32_get_position,
     .hall_start = abz_hall_stm32_enable,
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
    //  data->pre_sect = data->cur_sect;

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
         .Autoreload = config->lines,
         .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1
     };
     LL_TIM_Init(config->timer, &TIM_InitStruct);
     LL_TIM_DisableARRPreload(config->timer);
     LL_TIM_SetEncoderMode(config->timer, LL_TIM_ENCODERMODE_X2_TI1);
     
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
     hall->positive_sect[6].angle = SCETION_6_BASEANGLE + HALL_POSITIVE_OFFSET;
     hall->positive_sect[4].angle = SCETION_4_BASEANGLE + HALL_POSITIVE_OFFSET;
     hall->positive_sect[5].angle = SCETION_5_BASEANGLE + HALL_POSITIVE_OFFSET;
     hall->positive_sect[1].angle = SCETION_1_BASEANGLE + HALL_POSITIVE_OFFSET;
     hall->positive_sect[3].angle = SCETION_3_BASEANGLE + HALL_POSITIVE_OFFSET;
     hall->positive_sect[2].angle = SCETION_2_BASEANGLE + HALL_POSITIVE_OFFSET;
     hall->positive_sect[6].diff = _normalize_angle(hall->positive_sect[6].angle - hall->positive_sect[2].angle);
     hall->positive_sect[4].diff = _normalize_angle(hall->positive_sect[4].angle - hall->positive_sect[6].angle);
     hall->positive_sect[5].diff = _normalize_angle(hall->positive_sect[5].angle - hall->positive_sect[4].angle);
     hall->positive_sect[1].diff = _normalize_angle(hall->positive_sect[1].angle - hall->positive_sect[5].angle);
     hall->positive_sect[3].diff = _normalize_angle(hall->positive_sect[3].angle - hall->positive_sect[1].angle);
     hall->positive_sect[2].diff = _normalize_angle(hall->positive_sect[2].angle - hall->positive_sect[3].angle);

     hall->negative_sect[6].angle = hall->positive_sect[4].angle + HALL_NEGATIVE_OFFSET;
     hall->negative_sect[4].angle = hall->positive_sect[5].angle + HALL_NEGATIVE_OFFSET;
     hall->negative_sect[5].angle = hall->positive_sect[1].angle + HALL_NEGATIVE_OFFSET;
     hall->negative_sect[1].angle = hall->positive_sect[3].angle + HALL_NEGATIVE_OFFSET;
     hall->negative_sect[3].angle = hall->positive_sect[2].angle + HALL_NEGATIVE_OFFSET;
     hall->negative_sect[2].angle = hall->positive_sect[6].angle + HALL_NEGATIVE_OFFSET;
     hall->negative_sect[6].diff = -hall->positive_sect[5].diff;
     hall->negative_sect[4].diff = -hall->positive_sect[1].diff;
     hall->negative_sect[5].diff = -hall->positive_sect[3].diff;
     hall->negative_sect[1].diff = -hall->positive_sect[2].diff; 
     hall->negative_sect[3].diff = -hall->positive_sect[6].diff;
     hall->negative_sect[2].diff = -hall->positive_sect[4].diff;        
     return 0;
 }
 
 /*
  * Main initialization function
  */
 static int stm32_abz_hall_init(const struct device *dev)
 {
     abz_stm32_init(dev);
     hall_stm32_init(dev);
    //  abz_hall_stm32_enable(dev);
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