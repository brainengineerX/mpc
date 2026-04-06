#ifndef OBSERVERS_H
#define OBSERVERS_H

#include "motor_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：observers
 * 职责：提供鲁棒性增强所需的观测与自适应工具：
 * 1) 总扰动观测（电流环前馈补偿）
 * 2) 负载转矩观测（速度环前馈/监测）
 * 3) Rs在线自适应（温漂补偿）
 * 4) 速度滤波与位置插值（提升角度/速度可用性）
 * ========================================================================== */

/* 电流环总扰动观测器（Luenberger/ESO简化） */
typedef struct {
    float a;
    float b;
    float l_i;
    float l_d;

    CurrentState i_hat;
    Vec2 d_hat;
} DisturbanceObserver;

void disturbance_observer_init(DisturbanceObserver *obs, float a, float b, float l_i, float l_d);
void disturbance_observer_update(DisturbanceObserver *obs,
                                 const CurrentState *i_meas,
                                 const Vec2 *v_applied_ab);

/* 负载转矩观测 */
typedef struct {
    float J;
    float B;
    float lpf_gain;
    float tl_hat;
    float omega_prev;
    int inited;
} LoadTorqueObserver;

void load_torque_observer_init(LoadTorqueObserver *obs, float J, float B, float lpf_gain);
float load_torque_observer_update(LoadTorqueObserver *obs,
                                  float omega_m,
                                  float torque_e,
                                  float Ts);

/* Rs在线自适应（梯度法） */
typedef struct {
    float gamma;
    float rs_min;
    float rs_max;
} RsAdaptiveConfig;

float rs_adaptation_update(float rs_prev,
                           const RsAdaptiveConfig *cfg,
                           float vd, float vq,
                           float id, float iq,
                           float did, float diq,
                           float omega_e,
                           float Ld, float Lq,
                           float psi_f);

/* 速度滤波 + 位置插值（可用于编码器低分辨率补偿） */
typedef struct {
    float alpha;
    float omega_filt;
    float theta_interp;
    int inited;
} SpeedPosEstimator;

void speed_pos_estimator_init(SpeedPosEstimator *est, float alpha);
void speed_pos_estimator_update(SpeedPosEstimator *est,
                                float omega_meas,
                                float Ts,
                                float pole_pairs);

#ifdef __cplusplus
}
#endif

#endif /* OBSERVERS_H */
