/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

 #define DT_DRV_COMPAT st_stm32_hall

 #include <zephyr/device.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/irq.h>
 
 LOG_MODULE_REGISTER(hall_stm32, LOG_LEVEL_DBG);
 
 struct hall_stm32_config {
     struct gpio_dt_spec hu_gpio;
     struct gpio_dt_spec hv_gpio;
     struct gpio_dt_spec hw_gpio;
 };
 
 struct hall_stm32_data {
     struct gpio_callback gpio_cb;
     const struct device *dev;  // 添加设备指针
     int8_t cur_sect;
     int8_t pre_sect;
};
struct stm32_hall_driver_api {
	int8_t (*get_sect)(const struct device *dev);
}; 
 static void hall_gpio_callback(const struct device *port,
                              struct gpio_callback *cb,
                              uint32_t pins)
 {
     struct hall_stm32_data *data = CONTAINER_OF(cb, struct hall_stm32_data, gpio_cb);
 
     ARG_UNUSED(port);
     ARG_UNUSED(pins);

     const struct device *dev = data->dev;
     const struct hall_stm32_config *cfg = dev->config;

     int hu_state = gpio_pin_get_dt(&cfg->hu_gpio);
     int hv_state = gpio_pin_get_dt(&cfg->hv_gpio);
     int hw_state = gpio_pin_get_dt(&cfg->hw_gpio);
     LOG_DBG("Hall states - HALL_VAL:%d", hu_state<<2|hv_state<<1|hw_state);

     data->pre_sect = data->cur_sect;
     data->cur_sect = hu_state<<2|hv_state<<1|hw_state;
 }

static int8_t hall_get_sect(const struct device *dev)
{
   const struct hall_stm32_data *data = dev->data;
   return data->cur_sect; 
} 
static const struct stm32_hall_driver_api hall_stm32_driver_api = {
    .get_sect =  hall_get_sect,
};

 static int hall_stm32_init(const struct device *dev)
 {
     const struct hall_stm32_config *cfg = dev->config;
     struct hall_stm32_data *data = dev->data;
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
     ret = gpio_pin_interrupt_configure_dt(&cfg->hu_gpio, GPIO_INT_EDGE_BOTH);
     ret |= gpio_pin_interrupt_configure_dt(&cfg->hv_gpio, GPIO_INT_EDGE_BOTH);
     ret |= gpio_pin_interrupt_configure_dt(&cfg->hw_gpio, GPIO_INT_EDGE_BOTH);
     if (ret < 0) {
         LOG_ERR("Failed to configure interrupts");
         return ret;
     }
 
     return 0;
 }
 
 #define HALL_STM32_INIT(n) \
     static const struct hall_stm32_config hall_stm32_cfg_##n = { \
         .hu_gpio = GPIO_DT_SPEC_INST_GET(n, hu_gpios), \
         .hv_gpio = GPIO_DT_SPEC_INST_GET(n, hv_gpios), \
         .hw_gpio = GPIO_DT_SPEC_INST_GET(n, hw_gpios), \
     }; \
     static struct hall_stm32_data hall_stm32_data_##n; \
     DEVICE_DT_INST_DEFINE(n, \
                          hall_stm32_init, \
                          NULL, \
                          &hall_stm32_data_##n, \
                          &hall_stm32_cfg_##n, \
                          POST_KERNEL, \
                          99, \
                          &hall_stm32_driver_api); 
 DT_INST_FOREACH_STATUS_OKAY(HALL_STM32_INIT)