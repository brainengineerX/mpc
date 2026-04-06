#ifndef ADC_CALIBRATION_H
#define ADC_CALIBRATION_H

#include "motor_model.h" /* CurrentState / Vec2 等类型 */

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * ADC 采样校准模块
 * --------------------------------------------------------------------------
 * 处理三类常见误差：
 * 1) 零偏（offset）
 * 2) 比例（gain）
 * 3) 采样时序延迟（adc_delay）
 * ======================================================================== */

/* 电流传感器校准参数 */
typedef struct {
    float adc_delay_s;  /* PWM中点到ADC实际采样时间偏差 */
    float lpf_gain;     /* 在线校准低通更新系数 */
    float offset_alpha; /* alpha 通道零偏 */
    float offset_beta;  /* beta 通道零偏 */
    float gain_alpha;   /* alpha 通道比例系数 */
    float gain_beta;    /* beta 通道比例系数 */
} CurrentSensorCalib;

/* 对原始采样做 offset/gain 校准 */
CurrentState current_sensor_apply_calib(const CurrentState *raw,
                                        const CurrentSensorCalib *c);

/* 在线校准：
 * - target 通常设为当前期望电流（或零电流窗口目标）
 * - enable_gain_update=0 时只更新 offset，更稳 */
void current_sensor_online_calibration(CurrentSensorCalib *c,
                                       const CurrentState *raw,
                                       const CurrentState *target,
                                       int enable_gain_update);

/* 延迟补偿：把“延迟采样电流”前推到“当前控制时刻” */
CurrentState adc_delay_compensation(const CurrentState *i_sample,
                                    const Vec2 *v_applied_ab,
                                    float rs,
                                    float ls,
                                    float delay_s);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CALIBRATION_H */
