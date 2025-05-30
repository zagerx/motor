/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <lib/focutils/svm/svm.h>
#include <lib/focutils/utils/focutils.h>
#include <zephyr/logging/log.h>
/*******************************************************************************
 * Private
 ******************************************************************************/

/** Value sqrt(3). */
#define SQRT_3 1.7320508075688773f
#define PWM_TS (1.0f) 
#define T_UDC (1.0f)
/*******************************************************************************
 * Public
 ******************************************************************************/
 LOG_MODULE_REGISTER(SVM, LOG_LEVEL_DBG);

void svm_init(svm_t *svm)
{
	svm->sector = 0U;

	svm->duties.a = 0.0f;
	svm->duties.b = 0.0f;
	svm->duties.c = 0.0f;

	svm->d_min = 0.0f;
	svm->d_max = 1.0f;
}

void svm_set(svm_t *svm, float va, float vb)
{
    //判断扇区
    unsigned char sector;
    sector = 0;
    /*-------------------------------*/
    if(vb*(1<<15) > 0) {
        sector = 1;
    }
    if(((SQRT_3 * va - vb)/2.0F*(1<<15)) > 0) {
        sector += 2;
    }
    if(((-SQRT_3 * va - vb) / 2.0F)*(1<<15) > 0) {
        sector += 4;
    }
    //计算对应扇区的换相时间
    float X,Y,Z;
    X = (SQRT_3 * vb * T_UDC);
    Y = (1.5F * va + SQRT_3/2.0f * vb) * T_UDC;
    Z = (-1.5F * va + SQRT_3/2.0f * vb) * T_UDC;

    float s_vector = 0.0f,m_vector = 0.0f;
    switch (sector) {
        case 1:
            m_vector = Z;
            s_vector = Y;
        break;

        case 2:
            m_vector = Y;
            s_vector = -X;
        break;

        case 3:
            m_vector = -Z;
            s_vector = X;
        break;

        case 4:
            m_vector = -X;
            s_vector = Z;
        break;

        case 5:
            m_vector = X;
            s_vector = -Y;
        break;

        default:
            m_vector = -Y;
            s_vector = -Z;
        break;
    }
    /*--------------------限制矢量圆----------------------*/
    if (m_vector + s_vector > PWM_TS) 
    {
        float sum;
        sum = m_vector+s_vector;
        m_vector = (m_vector/(sum)*PWM_TS);
        s_vector = (s_vector/(sum)*PWM_TS);
    }
    /*---------------------------------------------------*/
    float Ta,Tb,Tc;
    Ta = (PWM_TS - (m_vector + s_vector)) / 4.0F;  
    Tb = Ta + m_vector/2.0f;
    Tc = Tb + s_vector/2.0f;
    /*------------------------换相点---------------------*/
    float Tcmp1 = 0.0f;
    float Tcmp2 = 0.0f;
    float Tcmp3 = 0.0f;
    switch (sector) {
        case 1:Tcmp1 = Tb;Tcmp2 = Ta;Tcmp3 = Tc;break;
        case 2:Tcmp1 = Ta;Tcmp2 = Tc;Tcmp3 = Tb;break;
        case 3:Tcmp1 = Ta;Tcmp2 = Tb;Tcmp3 = Tc;break;
        case 4:Tcmp1 = Tc;Tcmp2 = Tb;Tcmp3 = Ta;break;
        case 5:Tcmp1 = Tc;Tcmp2 = Ta;Tcmp3 = Tb;break;
        case 6:Tcmp1 = Tb;Tcmp2 = Tc;Tcmp3 = Ta;break;
    }
    /*-------------------------占空比---------------------------*/
    svm->duties.a =(PWM_TS - Tcmp1*2.0f )/PWM_TS;
    svm->duties.b =(PWM_TS - Tcmp2*2.0f )/PWM_TS;
    svm->duties.c =(PWM_TS - Tcmp3*2.0f )/PWM_TS;
	// svm->duties.a = CLAMP(svm->duties.a, svm->d_min, svm->d_max);
	// svm->duties.b = CLAMP(svm->duties.b, svm->d_min, svm->d_max);
	// svm->duties.c = CLAMP(svm->duties.c, svm->d_min, svm->d_max);
}


// 初始化调制比控制器
void modulation_manager_init(modulation_ctrl_t *ctrl, float max_modulation) {
    ctrl->max_modulation = max_modulation;
    ctrl->fsw = 10000.0f;     // 默认10kHz
    ctrl->dead_time = 1e-6f;   // 默认1μs死区
    ctrl->overmodulation = false;
}

// 电压限制函数 - 在dq坐标系应用
void apply_voltage_limiting(modulation_ctrl_t *ctrl, float *vd, float *vq,float Vdc)
{
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

// SVM补偿函数 - 在αβ坐标系应用
void apply_svm_compensation(modulation_ctrl_t *ctrl, float *valpha, float *vbeta,float Vdc) 
{
    // 1. 死区补偿
    if (ctrl->dead_time > 0) {
        // 计算死区电压损失 (伏特)
        float V_dead_comp = ctrl->dead_time * ctrl->fsw * Vdc;
        
        // 计算电压矢量幅值
        float Vmag ;
        sqrt_f32((*valpha * *valpha + *vbeta * *vbeta),&Vmag);
            
        if (Vmag > 1e-6f) { // 避免除以零
            // 方向保持不变的补偿
            float comp_alpha = *valpha * (V_dead_comp / Vmag);
            float comp_beta = *vbeta * (V_dead_comp / Vmag);
            
            *valpha += comp_alpha;
            *vbeta += comp_beta;
        }
    }
    
    // 2. 过调制处理
    if (ctrl->overmodulation) {
        // 计算当前调制比
        float Vref;
        sqrt_f32(*valpha * *valpha + *vbeta * *vbeta,&Vref);
        float m = Vref / (Vdc * 0.57735f);
        
        // 简单过调制处理：保持角度，限制幅值
        float max_voltage = Vdc * 0.57735f * 1.1547f; // 最大理论值(2/√3)
        
        if (m > 1.1547f) {
            // 方波模式 (六步换向) - 特殊处理
            *valpha = 0; 
            *vbeta = 0;
        } else {
            // 计算角度
            float angle = atan2f(*vbeta, *valpha);
            
            // 应用限幅
            *valpha = max_voltage * cosf(angle);
            *vbeta = max_voltage * sinf(angle);
        }
    }
}
// 根据温度动态调整
void update_modulation_limit(modulation_ctrl_t *ctrl, float temp_c) {
    // 温度每升高1°C，降低0.5%调制比
    float derating = 0.005f * (temp_c - 25.0f);
    ctrl->max_modulation = 0.95f - fmaxf(0, derating);
}
