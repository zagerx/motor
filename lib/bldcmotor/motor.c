#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <lib/foc/foc.h>
LOG_MODULE_REGISTER(motor_thread, LOG_LEVEL_DBG);

// 定义线程栈（增大到2048字节）
K_THREAD_STACK_DEFINE(motor_thread_stack, 1024);

// FOC线程数据结构
struct motor_thread_data {
    const struct device *motor_dev;
    struct k_thread thread;
};

#define LED0_NODE DT_ALIAS(led0)
#define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)
#define ENCODER_VCC DT_NODELABEL(encoder_vcc)
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define MOTOR0_NODE DT_INST(0, foc_ctrl_algo)
#define MOTOR1_NODE DT_INST(1, foc_ctrl_algo)

static void motor_thread_entry(void *p1, void *p2, void *p3)
{
    const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(MOT12_BRK_PIN_NODE, gpios);
    int ret = gpio_pin_configure_dt(&mot12_brk, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: Failed to configure brake pin\n", ret);
    }
    // gpio_pin_set(mot12_brk.port, mot12_brk.pin,1);
	const struct gpio_dt_spec encoder_vcc = GPIO_DT_SPEC_GET(ENCODER_VCC, gpios);
    ret = gpio_pin_configure_dt(&encoder_vcc, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: Failed to configure brake pin\n", ret);
    }
	LOG_INF("main statr"); 
	const struct device *motor0 = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor0)) {
        LOG_ERR("PWM motor1 device not ready");
        return;
    }
	foc_start(motor0);

	const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
    if (!device_is_ready(motor1)) {
        LOG_ERR("PWM motor1 device not ready");
        return;
    }
	foc_start(motor1);	
     
    while (1) {
        k_msleep(1000);
        LOG_INF("foc thread running");
    }
}

void motor_thread_creat(const struct device *dev)
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