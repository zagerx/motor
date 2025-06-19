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
#include <lib/focutils/utils/deadtime_comp.h>
#include <algorithmlib/s_posi_planning.h>
enum FOC_DATA_INDEX{
    FOC_PARAM_D_PID = 0,
    FOC_PARAM_Q_PID,
    FOC_PARAM_DQ_REF,
    FOC_PARAM_SPEED_REF,
    FOC_PARAM_POSI_REF,
    FOC_PARAM_POSI_PLANNING,
    FOC_PARAM_DQ_REAL,
    FOC_PARAM_SPEED_REAL,
    FOC_PARAM_ME_ANGLE_REAL,
    FOC_PARAM_AB_REAL,

    FOC_PARAM_BUSVOL,
    FOC_PARAM_BUSCURR,
};
/* FOC runtime data */
// 调制比控制结构体
typedef struct {
    float max_modulation; // 最大允许调制比 (0.95~1.15)
    float fsw;            // 开关频率 (Hz)
    float dead_time;      // 死区时间 (秒)
    bool overmodulation;  // 过调制标志
} modulation_ctrl_t;

 struct foc_data {
    svm_t *svm_handle;              /* Space Vector Modulation handle */

    float self_theta;               /* Internal theta for open loop */
    pid_cb_t id_pid;
    pid_cb_t iq_pid;
    pid_cb_t speed_pid;
    pid_cb_t pos_pid;    
    float id_ref,iq_ref;
    float speed_ref;
    float speed_real;
    float pos_real,pos_ref,pos_pre,pos_splanning_targe;
    SPosPlanner s_pos_ph;
    lowfilter_t speed_filter;
	modulation_ctrl_t modulation;
    /** 新增字段 - 电流信息 */
    float last_comp_alpha;  // 最后一次α轴补偿量
    float last_comp_beta;   // 最后一次β轴补偿量
    float i_alpha;          // α轴电流 (需在调用前更新)
    float i_beta;           // β轴电流 (需在调用前更新)
    DeadTimeCompConfig comp_cfg;    ///< 死区补偿配置
    DeadTimeCompState comp_state;   ///< 死区补偿状态
    float bus_vol;
    float debug_a;
    float debug_b;
    float debug_c;
    float debug_d;
    /* Read only variables */
    float i_a,i_b,i_c;
    float i_d, i_q;                 /* D/Q axis currents */
    float rads;                     /* Rotor speed (rad/s) */
    float angle;                    /* Mechanical angle */
    float eangle;                   /* Electrical angle */
    float sin_eangle, cos_eangle;   /* sin/cos of electrical angle */
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

extern  float foc_calculate_speed(const struct device* dev,float cur_speed);
void svm_apply_voltage_limiting(const struct device* dev, float *vd, float *vq,float Vdc);
void svm_apply_svm_compensation(const struct device* dev, float *valpha, float *vbeta,float Vdc) ;

#endif

