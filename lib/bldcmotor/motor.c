/**
 * @file motor.c
 * @brief BLDC电机控制线程实现
 * 
 * 功能:
 * 1. 初始化电机控制相关硬件(GPIO/PWM等)
 * 2. 启动FOC控制算法
 * 3. 运行看门狗定时器
 */

#include "zephyr/device.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <lib/foc/foc.h>
#define DT_DRV_COMPAT motor_bldc

/* 日志模块注册 */
LOG_MODULE_REGISTER(motor_thread, LOG_LEVEL_DBG);

/* 线程栈定义 */
K_THREAD_STACK_DEFINE(motor_thread_stack, 1024);

/**
 * @struct motor_thread_data
 * @brief 电机线程数据结构
 */
struct motor_thread_data {
    const struct device *motor_dev;  ///< 电机设备指针
    struct k_thread thread;         ///< 线程控制块
};
struct motor_config {
    const struct device *foc_dev;  // FOC控制算法设备
};

struct motor_data {
    uint8_t test;
};

static int motor_init(const struct device *dev)
{
    return 0;
}


/* 设备树节点定义 */
#define LED0_NODE DT_ALIAS(led0)
#define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)
#define ENCODER_VCC DT_NODELABEL(encoder_vcc)
#define W_DOG DT_NODELABEL(wdog)
#define MOTOR0_NODE DT_NODELABEL(motor1)
#define MOTOR1_NODE DT_NODELABEL(motor2)
/* GPIO设备定义 */
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

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

    /* 电机0初始化 */
    const struct device *motor0 = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor0)) {
        LOG_ERR("PWM motor1 device not ready");
        return;
    }
    const struct motor_config *cfg  = motor0->config;
    const struct device *foc0= cfg->foc_dev;
    foc_start(foc0);

    /* 电机1初始化 */
    const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
    if (!device_is_ready(motor1)) {
        LOG_ERR("PWM motor1 device not ready");
        return;
    }
    cfg  = motor1->config;
    const struct device *foc1= cfg->foc_dev;
    foc_start(foc1);    
 
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


#define MOTOR_INIT(n) \
    static const struct motor_config motor_cfg_##n = { \
        .foc_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(n), control_algorithm)), \
    }; \
    static struct motor_data motor_data_##n; \
    DEVICE_DT_INST_DEFINE(n, motor_init, NULL, \
                         &motor_data_##n, \
                         &motor_cfg_##n, \
                         POST_KERNEL, \
                         CONFIG_MOTOR_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_INIT)

