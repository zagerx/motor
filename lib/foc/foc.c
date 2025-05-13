#include "dsp/controller_functions.h"
#include "zephyr/device.h"

#include <math.h>
#include <zephyr/logging/log.h>

#include <lib/foc/foc.h>
#include <drivers/currsmp.h>
#include <drivers/pwm.h>
#include <lib/focutils/svm/svm.h>
#include <lib/focutils/utils/utils.h>
#include <drivers/feedback.h>
#include <zephyr/drivers/gpio.h>
#include <stm32h7xx_ll_gpio.h>//TODO

LOG_MODULE_REGISTER(foc, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT foc_ctrl_algo 

struct foc_config{
    const struct device *pwm;
    const struct device *currsmp;
    const struct device *feedback;
    void (*modulate)(svm_t*,float,float);
};

struct foc_data{
    svm_t *svm_handle;

    float self_theta;
    float test_a;
	/* Read only */
	float i_d;
	float i_q;
	float rads;
	float angle;
	float eangle;
	float sin_eangle;
	float cos_eangle;
	float v_alpha;
	float v_beta;
	float v_q;
	float v_d;	
};

static int foc_posloop(const struct device* dev)
{
    return 0;
}

static int foc_currentloop(const struct device* dev)
{
    return 0;
}

static int foc_openloop(const struct device* dev)
{
    // LOG_INF("foc_openloop  name %s",dev->name);
    return 0;
}
extern const struct gpio_dt_spec led;
extern void currsmp_shunt_stm32_get_currents(const struct device *dev,struct currsmp_curr *curr);
    void test_fun(void)
    {
        LOG_INF("test");
    }
    void _2r_2s(float* dq,float theta,float* alpbet)
    {
        alpbet[0] = dq[0] * cosf(theta) -  dq[1] * sinf(theta);
        alpbet[1] = dq[0] * sinf(theta) +  dq[1] * cosf(theta);
    }    
static void foc_curr_regulator(void *ctx)
{    
    struct device *dev = (struct device*)ctx;
    struct foc_config *cfg = (struct foc_config *)dev->config;
    struct device *currsmp = (struct device *)cfg->currsmp;
    struct foc_data *data = dev->data;

    struct currsmp_curr current_now;
    // LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_1);
    // gpio_pin_set_dt(&led, 1);
    // test_fun();
    // currsmp_get_currents(currsmp,&current_now);
 	// data->eangle = feedback_get_eangle(cfg->feedback);
    // clarke_f32(current_now.i_a, current_now.i_b, &data->v_alpha, &data->v_beta);

    // float sin_val,cos_val;
    // sin_cos_f32(data->eangle,&sin_val,&cos_val);
    // park_f32(data->v_alpha, data->v_beta,&data->v_d,&data->v_q,sin_val,cos_val);
    // //pid
	

    // // inv_park_f32()


    // cfg->modulate(data->svm_handle, 0.0f, 0.4f);
	
    /* set phase voltage */
    // svm_duties_t *duties = &(data->svm_handle->duties);
    // pwm_set_phase_voltages(cfg->pwm,duties->a,duties->b,duties->c);

    float alph,beta,sin_the,cos_the;
    sin_cos_f32(data->self_theta,&sin_the,&cos_the);
    data->self_theta += 0.0015f;
    if(data->self_theta>6.28f)
    {
        data->self_theta = 0.0f;
    }
    // inv_park_f32(0.2f,0.0f,&alph,&beta,sin_the,cos_the);
    // data->self_theta = 3.70f;
    float dq[2], ab[2];
    dq[0] = 0.00f;
    dq[1] = 0.045f;
    _2r_2s(dq,data->self_theta,ab);
    alph = ab[0];
    beta = ab[1];
    // if(cfg->modulate)
    // {
        cfg->modulate(data->svm_handle, alph, beta);
    // }else{
    //     LOG_INF("ERR");
    // }
    // svm_set(data->svm_handle, alph, beta);
    // LOG_INF("X");
    svm_t *svm = data->svm_handle;
    pwm_set_phase_voltages(cfg->pwm,svm->duties.a,svm->duties.b,svm->duties.c);
    data->test_a = svm->duties.a*10000.0f;
    // pwm_set_phase_voltages(cfg->pwm,0.5f,0.5f,0.5f);
    // gpio_pin_toggle_dt(&led);
    // gpio_pin_set_dt(&led, 0);
    // LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_1);

    return;

}
static int foc_init(const struct device* dev)
{
    const struct foc_config *cfg = dev->config;
    const struct device *currsmp = cfg->currsmp;
    currsmp_configure(currsmp,foc_curr_regulator,(void *)dev);
    LOG_INF("foc_init  name :%s",dev->name);
    return 0;
}
//todo
void foc_start(const struct device* dev)
{
    const struct foc_config *cfg = dev->config;
    const struct device *devc = cfg->pwm;
    pwm_start(devc);
}

#define FOC_INIT(n) \
\
    static const struct foc_api foc_api_##n = {\
        .posloop = foc_posloop,\
        .currloop = foc_currentloop,\
        .opencloop = foc_openloop,\
    };\
\
    static svm_t svm_##n;\
    static struct foc_data foc_data_##n = {\
        .svm_handle = &svm_##n,\
    };\
\
    static const struct foc_config foc_cfg_##n = {\
        .pwm = DEVICE_DT_GET(DT_INST(n, st_stm32_pwm_custom)),\
        .currsmp = DEVICE_DT_GET(DT_INST(n, st_stm32_currsmp_shunt)),\
        .feedback = DEVICE_DT_GET(DT_INST(n, st_stm32_abz_hall)),\
        .modulate = svm_set,\
    };\
\
	DEVICE_DT_INST_DEFINE(n, \
        &foc_init, \
        NULL, \
        &foc_data_##n, \
        &foc_cfg_##n, \
        POST_KERNEL, \
        99, \
        &foc_api_##n\
    );

DT_INST_FOREACH_STATUS_OKAY(FOC_INIT)
