/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <lib/focutils/svm/svm.h>
#include <lib/focutils/utils/focutils.h>
#include <zephyr/logging/log.h>

#undef  CLAMP
#undef  MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define CLAMP(val, low, high) (((val) <= (low)) ? (low) : MIN(val, high))

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

	svm->d_min = 0.01f;
	svm->d_max = 0.99f;
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
    static float da,db,dc;
    // svm->duties.a =(PWM_TS - Tcmp1*2.0f )/PWM_TS;
    // svm->duties.b =(PWM_TS - Tcmp2*2.0f )/PWM_TS;
    // svm->duties.c =(PWM_TS - Tcmp3*2.0f )/PWM_TS;
	
    da = (PWM_TS - Tcmp1*2.0f )/PWM_TS;
    db = (PWM_TS - Tcmp2*2.0f )/PWM_TS;
    dc = (PWM_TS - Tcmp3*2.0f )/PWM_TS;
    svm->duties.a = CLAMP(da, svm->d_min, svm->d_max);
	svm->duties.b = CLAMP(db, svm->d_min, svm->d_max);
	svm->duties.c = CLAMP(dc, svm->d_min, svm->d_max);
}

