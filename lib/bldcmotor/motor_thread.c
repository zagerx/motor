#include <zephyr/kernel.h>
#include "zephyr/device.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor_thread, LOG_LEVEL_DBG);

/* 线程栈定义 */
K_THREAD_STACK_DEFINE(motor_thread_stack, 1024);

/* 设备树节点定义 */
#define LED0_NODE DT_ALIAS(led0)
#define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)
#define ENCODER_VCC DT_NODELABEL(encoder_vcc)
#define W_DOG DT_NODELABEL(wdog)

/* GPIO设备定义 */
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

extern void motor_start(void);

/**
 * @struct motor_thread_data
 * @brief 电机线程数据结构
 */
 struct motor_thread_data {
    const struct device *motor_dev;  ///< 电机设备指针
    struct k_thread thread;         ///< 线程控制块
};
/**
 * @brief 电机线程入口函数
 * 
 * 执行流程:
 * 1. 初始化所有GPIO设备
 * 2. 启动两个电机的FOC控制
 * 3. 运行看门狗循环
 */
 static void motor_thread_entry(void *p1, void *p2, void *p3)
 {
     /* LED初始化 */
     if (!device_is_ready(led.port)) {
         printk("Error: LED device not ready\n");
         return;
     }
     int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
     if (ret < 0) {
         printk("Error %d: failed to configure LED\n", ret);
     }
 
     /* 刹车引脚初始化 */
     const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
     ret = gpio_pin_configure_dt(&mot12_brk, GPIO_OUTPUT_ACTIVE);
     if (ret < 0) {
         printk("Error %d: Failed to configure brake pin\n", ret);
     }
 
     /* 看门狗引脚初始化 */
     const struct gpio_dt_spec w_dog = GPIO_DT_SPEC_GET(W_DOG, gpios);
     ret = gpio_pin_configure_dt(&w_dog, GPIO_OUTPUT_ACTIVE);
     if (ret < 0) {
         printk("Error %d: Failed to configure brake pin\n", ret);
     }
     /* 编码器供电引脚初始化 */
     const struct gpio_dt_spec encoder_vcc = GPIO_DT_SPEC_GET(ENCODER_VCC, gpios);
     ret = gpio_pin_configure_dt(&encoder_vcc, GPIO_OUTPUT_ACTIVE);
     if (ret < 0) {
         printk("Error %d: Failed to configure brake pin\n", ret);
     }
 
     k_msleep(1000);  // 初始化延时

     motor_start();
     while (1) {
         gpio_pin_toggle_dt(&w_dog);
         k_msleep(1);
     }
 }
 
 /**
  * @brief 创建电机控制线程
  * @param dev 设备指针(未使用)
  */
 void creat_motor_thread(const struct device *dev)
 {
     static struct motor_thread_data thread_data __aligned(4);
     k_thread_create(&thread_data.thread,
                    motor_thread_stack,
                    K_THREAD_STACK_SIZEOF(motor_thread_stack),
                    motor_thread_entry,
                    NULL,
                    NULL,
                    NULL,
                    K_PRIO_COOP(5),  // 高优先级协作线程
                    0,
                    K_NO_WAIT);
 }
 