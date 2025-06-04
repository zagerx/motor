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
 
 float foc_speedexcu(const struct device* dev,float cur_speed)
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
    ctrl->dead_time = 0.6e-6f;   // 默认1μs死区
    ctrl->overmodulation = false;
}


// SVM补偿函数 - 在αβ坐标系应用
void svm_apply_svm_compensation(const struct device* dev, float *valpha, float *vbeta,float Vdc) 
{
    struct foc_data *f_data = dev->data;
 // 获取调制控制参数
 modulation_ctrl_t *ctrl = &f_data->modulation;
    
 // 如果死区时间为0或未使能，直接返回
 if (ctrl->dead_time <= 0)
 {
    return;
 } 
 
 // 1. 计算死区电压损失 (伏特)
 float V_dead_comp = ctrl->dead_time * ctrl->fsw * Vdc;
 
 // 2. 计算电压矢量幅值
 float V_mag;
sqrt_f32(*valpha * *valpha + *vbeta * *vbeta,&V_mag);
 
 // 3. 获取当前电流 (假设已在SVM结构体中更新)
 float i_alpha = f_data->i_alpha;
 float i_beta = f_data->i_beta;
 float I_mag;
 sqrt_f32(i_alpha * i_alpha + i_beta * i_beta,&I_mag);
 
 // 4. 计算电流方向单位矢量
 float dir_alpha = 0.0f;
 float dir_beta = 0.0f;
 
 if (I_mag > 0.01f) { // 避免除以零
     dir_alpha = i_alpha / I_mag;
     dir_beta = i_beta / I_mag;
 } else {
     // 电流过零区域 - 使用电压方向作为近似
     if (V_mag > 0.01f) {
         dir_alpha = *valpha / V_mag;
         dir_beta = *vbeta / V_mag;
     } else {
         // 无可靠方向信息，不补偿
         return;
     }
 }
 
 // 5. 过零区域平滑处理
 if (I_mag < 0.1f) { // 10%额定电流以下
     float k = I_mag / 0.1f; // 线性过渡系数 [0,1]
     V_dead_comp *= k;
 }
 
 // 6. 应用补偿 (方向与电流方向相反)
 *valpha = dir_alpha * V_dead_comp;
 *vbeta = dir_beta * V_dead_comp;
 
 // 7. 记录补偿状态 (用于调试)
 f_data->last_comp_alpha = dir_alpha * V_dead_comp;
 f_data->last_comp_beta = dir_beta * V_dead_comp;
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
    svm_init((svm_t *)&(data->svm_handle));
    modulation_manager_init((modulation_ctrl_t*)&(data->modulation),0.95f,650e-6f,10000e-6f);    
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