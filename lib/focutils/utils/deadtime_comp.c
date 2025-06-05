#include <lib/focutils/utils/deadtime_comp.h>
#include <math.h>
#include <stdbool.h>

// 内部函数：参数有效性检查
static void validate_config(DeadTimeCompConfig *cfg) {
    cfg->dead_time_ns = (cfg->dead_time_ns > 0) ? cfg->dead_time_ns : 650.0f;
    cfg->switching_freq = (cfg->switching_freq > 0) ? cfg->switching_freq : 10000.0f;
    cfg->max_comp_ratio = (cfg->max_comp_ratio > 0 && cfg->max_comp_ratio <= 0.15f) ? 
                          cfg->max_comp_ratio : 0.05f;
    cfg->zero_current_th = (cfg->zero_current_th > 0) ? cfg->zero_current_th : 0.05f;
    cfg->transition_th = (cfg->transition_th > cfg->zero_current_th) ? 
                         cfg->transition_th : cfg->zero_current_th * 2.0f;
    cfg->initialized = true;
}

void deadtime_comp_config_init(
    DeadTimeCompConfig *cfg,
    float dead_time_ns,
    float switching_freq,
    float max_comp_ratio,
    float zero_current_th,
    float transition_th)
{
    // 设置参数
    cfg->dead_time_ns = dead_time_ns;
    cfg->switching_freq = switching_freq;
    cfg->max_comp_ratio = max_comp_ratio;
    cfg->zero_current_th = zero_current_th;
    cfg->transition_th = transition_th;
    
    // 验证并修正参数
    validate_config(cfg);
}

bool deadtime_compensation_apply(
    const DeadTimeCompConfig *cfg,
    DeadTimeCompState *state,
    float *valpha,
    float *vbeta,
    float vdc,
    float i_alpha,
    float i_beta)
{
    // 检查配置有效性
    if (!cfg->initialized || cfg->dead_time_ns <= 0 || 
        cfg->switching_freq <= 0 || vdc <= 0) {
        return false;
    }

    // 计算基本补偿量
    const float dead_time_sec = cfg->dead_time_ns * 1e-9f;
    float v_comp_base = dead_time_sec * cfg->switching_freq * vdc;

    // 计算电流幅值和方向
    const float i_mag_sq = i_alpha*i_alpha + i_beta*i_beta;
    const float i_mag = sqrtf(i_mag_sq);
    
    float dir_alpha = 0.0f;
    float dir_beta = 0.0f;
    
    if (i_mag > cfg->zero_current_th) {
        dir_alpha = i_alpha / i_mag;
        dir_beta = i_beta / i_mag;
    } else {
        // 使用电压方向作为近似
        const float v_mag = sqrtf(*valpha * *valpha + *vbeta * *vbeta);
        if (v_mag > 0.01f) {
            dir_alpha = *valpha / v_mag;
            dir_beta = *vbeta / v_mag;
        }
    }

    // 过零区域平滑处理
    float comp_factor = 1.0f;
    if (i_mag < cfg->transition_th) {
        if (i_mag <= cfg->zero_current_th) {
            comp_factor = 0.0f;
        } else {
            comp_factor = (i_mag - cfg->zero_current_th) / 
                          (cfg->transition_th - cfg->zero_current_th);
        }
    }

    // 计算实际补偿量
    float v_comp = v_comp_base * comp_factor;
    
    // 应用安全限制
    const float max_comp = cfg->max_comp_ratio * vdc;
    if (v_comp > max_comp) v_comp = max_comp;
    if (v_comp < -max_comp) v_comp = -max_comp;
    
    // 应用补偿
    *valpha += dir_alpha * v_comp;
    *vbeta += dir_beta * v_comp;
    
    // 更新状态
    state->last_comp_alpha = dir_alpha * v_comp;
    state->last_comp_beta = dir_beta * v_comp;
    state->avg_comp = 0.95f * state->avg_comp + 0.05f * v_comp;
    
    return true;
}