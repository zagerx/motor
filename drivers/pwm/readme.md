# 创建一个pwm的设备

## 创建.waml文件
```

# Copyright (c) 2021, Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

description: |
  STM32S PWM device.

  The PWM device is expected to be a children of any STM32 advanced control
  timer. Example usage:

      &timers1 {
          pwm: pwm {
              compatible = "st,stm32-svpwm";
              pinctrl-0 = <&tim1_ch1_pa8 &tim1_ch2_pa9 &tim1_ch3_pa10 &tim1_ocp_pa11>;
              pinctrl-names = "default";
              ...
          };
      };

compatible: "st,stm32-pwm"

include: [base.yaml, pinctrl-device.yaml]

properties:
  pinctrl-0:
    required: true

  pinctrl-names:
    required: true


```
## .dts文件

```
&timers1 {
	pwm1: pwm {
		compatible = "st,stm32-pwm";
		pinctrl-0 = <&tim1_ch1_pe9  &tim1_ch1n_pe8
			     &tim1_ch2_pe11 &tim1_ch2n_pe10
			     &tim1_ch3_pe13 &tim1_ch3n_pe12>;
		pinctrl-names = "default";
		status = "okay";
	};
};
```
## 创建设备
- 将驱动程序与设备树(Device Tree)中的兼容性字符串进行绑定
    - `#define DT_DRV_COMPAT st_stm32_svpwm`
        - `st_stm32_svpwm`和设备树`compatible = "st,stm32-pwm"`强相关，其中设备树中的`",""-"`统一自动转为`"_"`
- 定义`DT_INST_FOREACH_STATUS_OKAY()`宏
    ```
    #define PMW_STM32_INIT(n)					\
    \
        PINCTRL_DT_INST_DEFINE(n);					\
            static const struct pwm_stm32_config pwm_stm32_config_##n = { \
            .timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)), \
            .t_dead = DT_PROP_OR(DT_INST_CHILD(n, pwm), t_dead_ns, 0), \
            .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
        };								\
            \
        DEVICE_DT_INST_DEFINE(n, \
            &pwm_stm32_init, \
            NULL,	\
            NULL,	\
            &pwm_stm32_config_##n,			\
            POST_KERNEL,				\
            80,	\
            NULL\
        );

    DT_INST_FOREACH_STATUS_OKAY(PMW_STM32_INIT)
    ```
    - 宏的参数解释
    ```
    DEVICE_DT_DEFINE(
        node_id,       // DT_DRV_INST(n)
        init_fn,       // pwm_stm32_init
        pm,           // NULL (无电源管理)
        data,         // NULL (无实例数据)
        config,       // &pwm_stm32_config_##n
        level,        // POST_KERNEL
        prio,         // 初始化优先级
        api,          // &svpwm_stm32_driver_api 后续补充
        ...           // 其他参数
    );  
    ```
