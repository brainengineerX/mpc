#include <math.h>             /* fabsf */
#include "adc_calibration.h" /* 本模块头文件 */

/* ==========================================================================
 * adc_calibration.c 实现说明
 * --------------------------------------------------------------------------
 * current_sensor_apply_calib：静态校准（偏置 + 比例）
 * current_sensor_online_calibration：慢速在线修正（补偿温漂）
 * adc_delay_compensation：时序补偿（模型前推）
 * ======================================================================== */

/* 应用静态校准参数 */
CurrentState current_sensor_apply_calib(const CurrentState *raw,
                                        const CurrentSensorCalib *c)
{
    CurrentState out;                                          /* 输出结构 */
    out.i_alpha = (raw->i_alpha - c->offset_alpha) * c->gain_alpha; /* alpha 校准 */
    out.i_beta = (raw->i_beta - c->offset_beta) * c->gain_beta;     /* beta 校准 */
    return out;                                                 /* 返回校准后电流 */
}

/* 在线校准（低速慢调） */
void current_sensor_online_calibration(CurrentSensorCalib *c,
                                       const CurrentState *raw,
                                       const CurrentState *target,
                                       int enable_gain_update)
{
    const float e_alpha = raw->i_alpha - target->i_alpha; /* alpha 误差 */
    const float e_beta = raw->i_beta - target->i_beta;    /* beta 误差 */

    c->offset_alpha += c->lpf_gain * e_alpha;             /* offset_alpha 低通更新 */
    c->offset_beta += c->lpf_gain * e_beta;               /* offset_beta 低通更新 */

    if (enable_gain_update) {                             /* 仅在允许时更新比例 */
        const float ref_mag_alpha = fabsf(target->i_alpha); /* alpha 激励强度 */
        const float ref_mag_beta = fabsf(target->i_beta);   /* beta 激励强度 */

        if (ref_mag_alpha > 1.0f) {                      /* 激励足够才更新，避免数值不稳定 */
            const float ratio_a = target->i_alpha / (raw->i_alpha - c->offset_alpha + 1.0e-6f); /* 比例估计 */
            c->gain_alpha += c->lpf_gain * 0.1f * (ratio_a - c->gain_alpha);                     /* 慢速收敛 */
        }
        if (ref_mag_beta > 1.0f) {                       /* 同理处理 beta */
            const float ratio_b = target->i_beta / (raw->i_beta - c->offset_beta + 1.0e-6f);
            c->gain_beta += c->lpf_gain * 0.1f * (ratio_b - c->gain_beta);
        }
    }
}

/* ADC 延迟补偿 */
CurrentState adc_delay_compensation(const CurrentState *i_sample,
                                    const Vec2 *v_applied_ab,
                                    float rs,
                                    float ls,
                                    float delay_s)
{
    CurrentState out = *i_sample;                                /* 先复制采样值 */
    const float did_alpha = (v_applied_ab->x - rs * i_sample->i_alpha) / ls; /* alpha 斜率估计 */
    const float did_beta = (v_applied_ab->y - rs * i_sample->i_beta) / ls;   /* beta 斜率估计 */
    out.i_alpha += delay_s * did_alpha;                          /* 前推到控制时刻 */
    out.i_beta += delay_s * did_beta;                            /* 前推到控制时刻 */
    return out;                                                   /* 返回补偿结果 */
}
