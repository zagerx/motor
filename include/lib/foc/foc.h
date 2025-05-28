#ifndef ZEPHYR_INCLUDE_CONTROL_FOC_H_
#define ZEPHYR_INCLUDE_CONTROL_FOC_H_

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "zephyr/device.h"
#include <lib/focutils/svm/svm.h>
#include <algorithmlib/pid.h>
#include <sys/types.h>
#include <algorithmlib/filter.h>
enum FOC_DATA_INDEX{
    FOC_PARAM_D_PID = 0,
    FOC_PARAM_Q_PID,
    FOC_PARAM_DQ_REF,
    FOC_PARAM_SPEED_REF,


    FOC_PARAM_DQ_REAL,
    FOC_PARAM_SPEED_REAL,
    FOC_PARAM_ME_ANGLE_REAL,
    FOC_PARAM_AB_REAL,
};
/* FOC runtime data */
 struct foc_data {
    svm_t *svm_handle;              /* Space Vector Modulation handle */

    float self_theta;               /* Internal theta for open loop */
    pid_cb_t id_pid;
    pid_cb_t iq_pid;
    pid_cb_t speed_pid;
    float id_ref,iq_ref;
    float speed_ref;
    float speed_real;
    lowfilter_t speed_filter;
    /* Read only variables */
    float i_a,i_b,i_c;
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
    void (*write_data)(const struct device*,int16_t,float*);
};
 

static inline void foc_modulate(const struct device* dev,float alpha,float beta)
{
   const struct foc_config* cfg = dev->config;
   const struct foc_data* data = dev->data; 
   cfg->modulate(data->svm_handle,alpha,beta);
} 

static inline void foc_write_data(const struct device* dev,int16_t flag,float* input)
{
   const struct foc_api *api = dev->api; 
   api->write_data(dev,flag,input);
}

extern  float foc_speedexcu(const struct device* dev,float cur_speed);

#endif

