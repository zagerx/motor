#include "zephyr/device.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(foc, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT foc_ctrl_algo 


struct foc_data{

};

struct foc_api{
    int (*posloop)(const struct device*);
    int (*currloop)(const struct device*);
    int (*opencloop)(const struct device*);
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


static int foc_init(const struct device* dev)
{
    LOG_INF("foc_init  name %s",dev->name);
    return 0;
}


#define FOC_INIT(n) \
\
    static const struct foc_api foc_api_##n = {\
        .posloop = foc_posloop,\
        .currloop = foc_currentloop,\
        .opencloop = foc_openloop,\
    };\
\
	DEVICE_DT_INST_DEFINE(n, \
        &foc_init, \
        NULL, \
        NULL, \
        NULL, \
        POST_KERNEL, \
        85, \
        &foc_api_##n\
    );

DT_INST_FOREACH_STATUS_OKAY(FOC_INIT)
