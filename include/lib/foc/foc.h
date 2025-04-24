#ifndef ZEPHYR_INCLUDE_CONTROL_FOC_H_
#define ZEPHYR_INCLUDE_CONTROL_FOC_H_

#include <stdlib.h>
#include <string.h>

#include "zephyr/device.h"

struct foc_api{
    int (*posloop)(const struct device*);
    int (*currloop)(const struct device*);
    int (*opencloop)(const struct device*);
    void (*curr_regulator)(void *ctx);
};


void foc_start(const struct device* dev);





#endif

