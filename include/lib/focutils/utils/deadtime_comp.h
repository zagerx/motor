#ifndef DEADTIME_COMP_H
#define DEADTIME_COMP_H

#include <stdbool.h>

/**
 * @brief 死区补偿配置参数
 */
typedef struct {
    float dead_time_ns;     ///< 死区时间 (纳秒)
    float switching_freq;   ///< 开关频率 (Hz)
    float max_comp_ratio;   ///< 最大补偿比例 (0.0~0.15)
    float zero_current_th;  ///< 零电流阈值 (安培)
    float transition_th;    ///< 过渡区电流阈值 (安培)
    bool initialized;       ///< 配置是否已初始化
} DeadTimeCompConfig;

/**
 * @brief 死区补偿运行时状态
 */
typedef struct {
    float last_comp_alpha;  ///< 上次α轴补偿电压
    float last_comp_beta;   ///< 上次β轴补偿电压
    float avg_comp;         ///< 平均补偿量 (用于监控)
} DeadTimeCompState;

/**
 * @brief 初始化死区补偿配置
 * 
 * @param cfg       补偿配置参数指针
 * @param dead_time_ns      死区时间 (纳秒)
 * @param switching_freq    开关频率 (Hz)
 * @param max_comp_ratio    最大补偿比例 (0.0~0.15)
 * @param zero_current_th   零电流阈值 (安培)
 * @param transition_th     过渡区电流阈值 (安培)
 */
void deadtime_comp_config_init(
    DeadTimeCompConfig *cfg,
    float dead_time_ns,
    float switching_freq,
    float max_comp_ratio,
    float zero_current_th,
    float transition_th
);

/**
 * @brief 应用死区电压补偿
 * 
 * @param cfg       补偿配置参数
 * @param state     补偿状态跟踪
 * @param valpha    输入/输出: α轴电压
 * @param vbeta     输入/输出: β轴电压
 * @param vdc       直流母线电压
 * @param i_alpha   α轴电流
 * @param i_beta    β轴电流
 * 
 * @return true     应用了补偿
 * @return false    未应用补偿
 */
bool deadtime_compensation_apply(
    const DeadTimeCompConfig *cfg,
    DeadTimeCompState *state,
    float *valpha,
    float *vbeta,
    float vdc,
    float i_alpha,
    float i_beta
);

#endif // DEADTIME_COMP_H