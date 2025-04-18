#include "zephyr/device.h"

#define DT_DRV_COMPAT bldc_motor


struct motor_config{
    const struct device *pwm;
    const struct device *ctrl_algo;
};

static int motor_init(const struct device* dev)
{
    return 0;
} 

#define MOTOR_INIT(n) \
\
    static const struct motor_config motor_cfg_##n = {\
        .pwm = DEVICE_DT_GET(DT_INST(n, st_stm32_pwm_custom)),\
        .ctrl_algo = DEVICE_DT_GET(DT_INST(n, foc_ctrl_algo)),\
    };\
	DEVICE_DT_INST_DEFINE(n, \
        &motor_init, \
        NULL, \
        NULL, \
        &motor_cfg_##n, \
        POST_KERNEL, \
        90, \
        NULL\
    );

DT_INST_FOREACH_STATUS_OKAY(MOTOR_INIT)


