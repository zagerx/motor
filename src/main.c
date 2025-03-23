#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// 通过设备树别名获取 LED 配置
#define LED_NODE DT_ALIAS(status_led)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

void main(void) {
    // 初始化 LED GPIO
    if (!device_is_ready(led.port)) {
        return;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    while (1) {
        gpio_pin_toggle_dt(&led);  // 翻转 LED 状态
        k_msleep(500);             // 延时 500ms
    }
}