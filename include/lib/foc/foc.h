#ifndef ZEPHYR_INCLUDE_CONTROL_FOC_H_
#define ZEPHYR_INCLUDE_CONTROL_FOC_H_

#include <stdlib.h>
#include <string.h>

#include "zephyr/device.h"
#include <lib/focutils/svm/svm.h>

 /* FOC runtime data */
 struct foc_data {
    svm_t *svm_handle;              /* Space Vector Modulation handle */

    float self_theta;               /* Internal theta for open loop */
    float test_a;                   /* Test variable */
    
    /* Read only variables */
    float i_d, i_q;                 /* D/Q axis currents */
    float rads;                     /* Rotor speed (rad/s) */
    float angle;                    /* Mechanical angle */
    float eangle;                   /* Electrical angle */
    float sin_eangle, cos_eangle;   /* sin/cos of electrical angle */
    float v_alpha, v_beta;          /* Alpha/beta voltages */
    float v_q, v_d;                 /* Q/D axis voltages */
};

struct foc_config {
    void (*modulate)(svm_t*,float,float); /* Modulation function */
};

struct foc_api{
    int (*posloop)(const struct device*);
    int (*currloop)(const struct device*);
    int (*opencloop)(const struct device*);
    void (*curr_regulator)(void *ctx);
};

static inline void foc_modulate(const struct device* dev,float alpha,float beta)
{
   const struct foc_config* cfg = dev->config;
   const struct foc_data* data = dev->data; 
   cfg->modulate(data->svm_handle,alpha,beta);
} 
#endif

