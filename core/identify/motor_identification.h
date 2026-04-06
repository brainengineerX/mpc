#ifndef MOTOR_IDENTIFICATION_H
#define MOTOR_IDENTIFICATION_H

#include "motor_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：motor_identification
 * 职责：在“进入MPC控制之前”识别关键参数，降低模型失配。
 *
 * 本模块采用线性最小二乘方法，优点：
 * 1) 计算稳定、实现简单，适合嵌入式；
 * 2) 对噪声有一定鲁棒性（比逐样本直接反解更稳）；
 * 3) 便于在线滚动更新。
 * ========================================================================== */

/* 单个电参数辨识样本：由测量/观测得到 */
typedef struct {
    float id;
    float iq;
    float did;
    float diq;
    float vd;
    float vq;
    float omega_e;
} MotorIdSample;

typedef struct {
    float Rs;
    float Ld;
    float Lq;
    float psi_f;

    int used_samples;
    float rms_error;
} MotorIdResult;

/* 线性最小二乘辨识 PMSM 关键参数：Rs/Ld/Lq/psi_f。
 * 返回 0 表示成功，非 0 表示矩阵退化或样本不足。 */
int motor_identify_pmsm_params(const MotorIdSample *samples,
                               int sample_count,
                               MotorIdResult *out);

/* 将辨识结果映射到 MPC 当前使用的简化参数：
 * 说明：MPC当前简化模型仅用等效Ls，因此取 (Ld+Lq)/2。 */
void motor_id_result_to_mpc_params(const MotorIdResult *id,
                                   float Ts,
                                   float Vdc,
                                   MotorParams *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_IDENTIFICATION_H */
