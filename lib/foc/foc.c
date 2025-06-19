/*
 * Field Oriented Control (FOC) implementation
 *
 * Implements FOC control algorithms for BLDC/PMSM motors
 * Features:
 * - Current loop regulation
 * - Position/speed control
 * - Open loop control
 */

#include "algorithmlib/filter.h"
#include "algorithmlib/pid.h"
#include "lib/focutils/svm/svm.h"
#include "lib/focutils/utils/deadtime_comp.h"
#include "zephyr/device.h"
 
 #include <zephyr/logging/log.h>
 #include <lib/foc/foc.h>
 #include <lib/focutils/utils/focutils.h>

 LOG_MODULE_REGISTER(foc, LOG_LEVEL_DBG);
 
 #define DT_DRV_COMPAT foc_ctrl_algo 
 
 /* FOC configuration structure */
 static void modulation_manager_init(modulation_ctrl_t *ctrl, float max_modulation,float dead_time,float fsw);
static void _write(const struct device* dev,int16_t flag,float *input)
{
    struct foc_data *data = dev->data;
    switch (flag) {
        case FOC_PARAM_D_PID:
            {
                float kp,ki,kc,max,min;
                kp = input[0];ki = input[1];kc = input[2];max = input[3];min = input[4];
                pid_init(&data->id_pid,kp,ki,kc,max,min);
            }
        break;
        case FOC_PARAM_Q_PID:
            {
                float kp,ki,kc,max,min;
                kp = input[0];ki = input[1];kc = input[2];max = input[3];min = input[4];
                pid_init(&data->iq_pid,kp,ki,kc,max,min);
            }
        break;
        case FOC_PARAM_DQ_REF:
            {
                float id_ref,iq_ref;
                id_ref = input[0];iq_ref = input[1];
                data->id_ref = id_ref;
                data->iq_ref = iq_ref;
            }
        break;
        case FOC_PARAM_SPEED_REF:
            {
                float speed_ref;
                speed_ref = input[0];
                data->speed_ref = speed_ref;
            }
            break;
        case FOC_PARAM_POSI_REF:
        {
            float posi_ref;
            posi_ref = input[0];
            data->pos_ref = posi_ref;
        }
        break;
        case FOC_PARAM_POSI_PLANNING:
        {
            float posi_ref;
            posi_ref = input[0];
            data->pos_splanning_targe = posi_ref;
        }
        break;
        case FOC_PARAM_DQ_REAL:
            {
                data->i_d = input[0];
                data->i_q = input[1];
            }
        break;
        case FOC_PARAM_ME_ANGLE_REAL:
            {
                data->angle = input[0];
                data->eangle = input[1];
            }
        break;
        case FOC_PARAM_BUSVOL:
            {
                data->bus_vol = input[0];
            }
        break;
    } 
}
 /*
  * Position control loop (stub)
  * Returns: 0 on success
  */
 static int foc_posloop(const struct device* dev)
 {
     return 0;
 }
 
 /*
  * Current control loop (stub)
  * Returns: 0 on success
  */
 static int foc_currentloop(const struct device* dev)
 {
     return 0;
 }
 
 /*

  * Open loop control (stub)
  * Returns: 0 on success
  */
 static int foc_openloop(const struct device* dev)
 {
     return 0;
 }
 
 float foc_calculate_speed(const struct device* dev,float cur_speed)
 {
    struct foc_data *data = dev->data;
    float speed;
    speed = lowfilter_cale((lowfilter_t *)&data->speed_filter, cur_speed);
    data->speed_real = speed;
    return speed;
}
// 电压限制函数 - 在dq坐标系应用
void svm_apply_voltage_limiting(const struct device* dev, float *vd, float *vq,float Vdc)
{
    struct foc_data *f_data = dev->data;
    modulation_ctrl_t *ctrl = &(f_data->modulation);
    // 计算最大允许相电压幅值 (Vdc/sqrt(3))
    const float Vmax_linear = Vdc * 0.57735f * ctrl->max_modulation; // 0.57735=1/sqrt(3)
    float Vmag ;
    sqrt_f32((*vd * *vd + *vq * *vq),&Vmag);
    
    if (Vmag > Vmax_linear) {
        // 进入过调制区域
        ctrl->overmodulation = true;
        
        // 线性缩放
        float scale = Vmax_linear / Vmag;
        *vd *= scale;
        *vq *= scale;
    } else {
        ctrl->overmodulation = false;
    }
}

// 初始化调制比控制器
static void modulation_manager_init(modulation_ctrl_t *ctrl, float max_modulation,float dead_time,float fsw)
{
    ctrl->max_modulation = max_modulation;
    ctrl->fsw = 10000.0f;     // 默认10kHz
    ctrl->dead_time = 1e-6f;   // 默认1μs死区
    ctrl->overmodulation = false;
}


// SVM补偿函数 - 在αβ坐标系应用
void svm_apply_svm_compensation(const struct device* dev, float *valpha, float *vbeta,float Vdc) 
{
    struct foc_data *f_data = dev->data;

    deadtime_compensation_apply(
        (const DeadTimeCompConfig *)&f_data->comp_cfg,
        &f_data->comp_state,
        valpha,
        vbeta,
        Vdc,
        f_data->i_alpha,
        f_data->i_beta
    ); 
}



// 根据温度动态调整
void update_modulation_limit(modulation_ctrl_t *ctrl, float temp_c) {
    // 温度每升高1°C，降低0.5%调制比
    float derating = 0.005f * (temp_c - 25.0f);
    ctrl->max_modulation = 0.95f - fmaxf(0, derating);
}

 /*
  * Initialize FOC device
  * Returns: 0 on success
  */
 static int foc_init(const struct device* dev)
 {
    const struct foc_data *data = dev->data;
    pid_init((pid_cb_t*)&(data->id_pid), 0.0f, 0.0f, 0.0f,0.0f,0.0f);
    pid_init((pid_cb_t*)&(data->iq_pid), 0.0f, 0.0f, 0.0f,0.0f,0.0f);
    lowfilter_init((lowfilter_t *)&(data->speed_filter), 10.0f);
    svm_init((svm_t *)(data->svm_handle));
    modulation_manager_init((modulation_ctrl_t*)&(data->modulation),0.95f,650e-9f,10000);    
    // 初始化死区补偿配置
    deadtime_comp_config_init(
        (DeadTimeCompConfig*)&data->comp_cfg,   // 配置结构体
        650.0f,            // 死区时间 (ns)
        10000.0f,          // 开关频率 (Hz)
        0.05f,             // 最大补偿比例
        0.05f,             // 零电流阈值 (A)
        0.15f              // 过渡区阈值 (A)
    );
    
    // 初始化状态结构体
    memset((void*)&data->comp_state, 0, sizeof(DeadTimeCompState));

    return 0;
 }




 /* Device instance macro */
 #define FOC_INIT(n) \
     static const struct foc_api foc_api_##n = { \
         .posloop = foc_posloop, \
         .currloop = foc_currentloop, \
         .opencloop = foc_openloop, \
         .write_data = _write,\
     }; \
     static svm_t svm_##n; \
     static struct foc_data foc_data_##n = { \
         .svm_handle = &svm_##n, \
     }; \
     static const struct foc_config foc_cfg_##n = { \
         .modulate = svm_set, \
     }; \
     DEVICE_DT_INST_DEFINE(n, \
         &foc_init, \
         NULL, \
         &foc_data_##n, \
         &foc_cfg_##n, \
         POST_KERNEL, \
         CONFIG_FOC_INIT_PRIORITY, \
         &foc_api_##n \
     );
 
 /* Initialize all FOC instances */
 DT_INST_FOREACH_STATUS_OKAY(FOC_INIT)