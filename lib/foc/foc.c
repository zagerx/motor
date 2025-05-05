#include "dsp/controller_functions.h"
#include "zephyr/device.h"

#include <zephyr/logging/log.h>

#include <lib/foc/foc.h>
#include <drivers/currsmp.h>
#include <drivers/pwm.h>
#include <lib/focutils/svm/svm.h>
#include <lib/focutils/utils/utils.h>
#include <drivers/feedback.h>
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
    LOG_INF("foc_openloop  name %s",dev->name);
    return 0;
}

static void foc_curr_regulator(void *ctx)
{    
    struct device *dev = (struct device*)ctx;
    const struct foc_config *cfg = dev->config;
    const struct device *currsmp = cfg->currsmp;
    struct foc_data *data = dev->data;

    struct currsmp_curr current_now;
    currsmp_get_currents(currsmp,&current_now);
 	data->eangle = feedback_get_eangle(cfg->feedback);
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
    pwm_set_phase_voltages(cfg->pwm,0.5f,0.5f,0.5f);


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
