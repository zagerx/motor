#ifndef __LIB_UTILS_H_
#define __LIB_UTILS_H_

#if defined(CONFIG_CMSIS_DSP)
    #include "arm_math.h"
    #define clarke_f32    arm_clarke_f32
    #define park_f32      arm_park_f32
    #define inv_park_f32  arm_inv_park_f32
    #define sin_cos_f32   arm_sin_cos_f32
    // #define cos_f32   arm_cos_f32
#endif





#endif