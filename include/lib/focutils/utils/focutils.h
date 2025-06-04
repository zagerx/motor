#ifndef __LIB_UTILS_H_
#define __LIB_UTILS_H_

#if defined(CONFIG_CMSIS_DSP)
    #include "arm_math.h"
    #define clarke_f32    arm_clarke_f32
    #define park_f32      arm_park_f32
    #define inv_park_f32  arm_inv_park_f32
    #define sin_cos_f32   arm_sin_cos_f32
    #define atan_f32    arm_atan2_f32
    #define sqrt_f32      arm_sqrt_f32
#endif

#define _2_PI_     6.28318530717958647692f  // 2π
#define _PI_       3.14159265358979323846f  // π
#define _PI_2_     1.57079632679489661923f  // π/2
#define _PI_3_     1.04719755119659774615f  // π/3
#define _PI_6_     0.52359877559829887308f  // π/6
#define _PI_4_     0.78539816339744830962f  // π/4
#define _SQRT_3_   1.73205080756887729353f  // √3
#define _SQRT_2_   1.41421356237309504880f  // √2
#define _SQRT_1_3_ 0.57735026918962576451f  // 1/√3
#define _SQRT_1_2_ 0.70710678118654752440f  // 1/√2
#endif