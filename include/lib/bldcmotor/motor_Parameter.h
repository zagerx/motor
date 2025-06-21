#ifndef _MOTOR_PARAMETER_H
#define _MOTOR_PARAMETER_H

#if defined(CONFIG_MOTOR_SUPER_ABZHALL_400W)

#define MOTOR_PAIRS                     (5.0f)
#define MOTOR_RS                        (0.005f)
#define MOTOR_LS                        (0.005f)

#define SCETION_3_BASEANGLE             (286.00f)
#define SCETION_2_BASEANGLE             (346.00f)
#define SCETION_6_BASEANGLE             (046.00f)
#define SCETION_4_BASEANGLE             (106.00f)
#define SCETION_5_BASEANGLE             (166.00f)
#define SCETION_1_BASEANGLE             (226.00f)
#define HALL_SENSOR_POSITIVE_OFFSET     (-30.0f)  /* 霍尔传感器正方向偏移量 (度) */
#define HALL_SENSOR_NEGATIVE_OFFSET     (0.0f)   /* 霍尔传感器负方向偏移量 (度) */

#define ABZ_ENCODER_LINES               (4096)//编码器线数
#define ABZ_ENCODER_RESOLUTION          (0.087890625f)//360.0f/ABZ_ENCODER_LINES
#elif defined(CONFIG_MOTOR_ELUR_HALL)

#else
    #error "No valid board configuration selected!"
#endif

#endif
