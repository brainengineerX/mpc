/**
 * @file motor_model_fixed.c
 * @brief 定点化电机模型实现
 *
 * 本文件实现定点化电机模型，包括：
 * - 电流预测计算
 * - Park/Clarke 变换（查表法）
 * - 死区补偿参考电压计算
 */

#include "motor_model_fixed.h"
#include <math.h>

/* ============================================================================
 * 电机参数初始化
 * ============================================================================ */

void motor_params_q15_init(MotorParamsQ15 *params,
                           float Rs, float Ls, float Ts,
                           float Vdc, float i_base, float v_base) {
    /* 计算离散系数 a = 1 - Ts*Rs/Ls */
    float a_f = 1.0f - (Ts * Rs / Ls);
    /* 计算离散系数 b = Ts/Ls */
    float b_f = Ts / Ls;

    /* 转换为 Q30 格式 */
    params->a_coeff = FLOAT_TO_Q30(a_f);
    params->b_coeff = FLOAT_TO_Q30(b_f);

    /* 母线电压转换为 Q15 (Vdc / v_base) */
    float vdc_norm = Vdc / v_base;
    if (vdc_norm > 1.0f) vdc_norm = 1.0f;
    if (vdc_norm < -1.0f) vdc_norm = -1.0f;
    params->v_dc = FLOAT_TO_Q15(vdc_norm);

    /* 基准值 */
    params->i_base = FLOAT_TO_Q15(i_base / i_base);  /* = 1.0 in Q15 */
    params->v_base = FLOAT_TO_Q15(v_base / v_base);  /* = 1.0 in Q15 */
}

/* ============================================================================
 * 电流预测
 * ============================================================================ */

CurrentStateQ15 motor_predict_current_q15(const CurrentStateQ15 *x,
                                          const VoltageQ15 *v,
                                          const MotorParamsQ15 *params) {
    CurrentStateQ15 x_next;

    /* i_alpha(k+1) = a*i_alpha(k) + b*v_alpha(k) */
    /* 使用 Q30 * Q15 -> Q15 乘累加 */
    x_next.i_alpha = q30_mac_q15(0, params->a_coeff, x->i_alpha);
    x_next.i_alpha = q30_mac_q15(x_next.i_alpha, params->b_coeff, v->v_alpha);

    /* i_beta(k+1) = a*i_beta(k) + b*v_beta(k) */
    x_next.i_beta = q30_mac_q15(0, params->a_coeff, x->i_beta);
    x_next.i_beta = q30_mac_q15(x_next.i_beta, params->b_coeff, v->v_beta);

    return x_next;
}

/* ============================================================================
 * Park 变换 (alpha-beta to d-q)
 * ============================================================================ */

void alphabeta_to_dq_q15(q15_t i_alpha, q15_t i_beta,
                         q15_t theta_e,
                         q15_t *i_d, q15_t *i_q) {
    /* Park 变换矩阵:
     * [d]   [ cos  sin][alpha]
     * [q] = [-sin  cos][ beta]
     *
     * i_d =  i_alpha * cos(theta) + i_beta * sin(theta)
     * i_q = -i_alpha * sin(theta) + i_beta * cos(theta)
     */

    q15_t cos_theta = q15_cos_lut(theta_e);
    q15_t sin_theta = q15_sin_lut(theta_e);

    /* i_d = i_alpha * cos + i_beta * sin */
    q15_t term1 = q15_mul(i_alpha, cos_theta);
    q15_t term2 = q15_mul(i_beta, sin_theta);
    *i_d = q15_add(term1, term2);

    /* i_q = -i_alpha * sin + i_beta * cos */
    q15_t neg_i_alpha = q15_neg(i_alpha);
    q15_t term3 = q15_mul(neg_i_alpha, sin_theta);
    q15_t term4 = q15_mul(i_beta, cos_theta);
    *i_q = q15_add(term3, term4);
}

/* ============================================================================
 * 逆 Park 变换 (d-q to alpha-beta)
 * ============================================================================ */

void dq_to_alphabeta_q15(q15_t i_d, q15_t i_q,
                         q15_t theta_e,
                         q15_t *i_alpha, q15_t *i_beta) {
    /* 逆 Park 变换矩阵:
     * [alpha]   [ cos -sin][d]
     * [ beta] = [ sin  cos][q]
     *
     * i_alpha = i_d * cos(theta) - i_q * sin(theta)
     * i_beta  = i_d * sin(theta) + i_q * cos(theta)
     */

    q15_t cos_theta = q15_cos_lut(theta_e);
    q15_t sin_theta = q15_sin_lut(theta_e);

    /* i_alpha = i_d * cos - i_q * sin */
    q15_t term1 = q15_mul(i_d, cos_theta);
    q15_t neg_i_q = q15_neg(i_q);
    q15_t term2 = q15_mul(neg_i_q, sin_theta);
    *i_alpha = q15_add(term1, term2);

    /* i_beta = i_d * sin + i_q * cos */
    q15_t term3 = q15_mul(i_d, sin_theta);
    q15_t term4 = q15_mul(i_q, cos_theta);
    *i_beta = q15_add(term3, term4);
}

/* ============================================================================
 * Clarke 变换 (a-b-c to alpha-beta)
 * ============================================================================ */

void abc_to_alphabeta_q15(q15_t i_a, q15_t i_b, q15_t i_c,
                          q15_t *i_alpha, q15_t *i_beta) {
    /* Clarke 变换 (等幅值变换):
     * i_alpha = (2/3) * (i_a - 0.5*i_b - 0.5*i_c)
     * i_beta  = (1/sqrt(3)) * (i_b - i_c)
     *
     * 假设三相电流平衡: i_a + i_b + i_c = 0，简化为:
     * i_alpha = i_a
     * i_beta  = (i_a + 2*i_b) / sqrt(3)
     */

    /* 方法1: 使用简化的等幅值变换 (假设三相平衡) */
    /* i_alpha = i_a */
    *i_alpha = i_a;

    /* i_beta = (i_a + 2*i_b) / sqrt(3)
     * 1/sqrt(3) ≈ 0.57735，在 Q15 中表示为 18919 (0.57735 * 32767)
     * 先计算 (i_a + 2*i_b)，然后乘以 1/sqrt(3)
     */
    q15_t two_i_b = q15_shl(i_b, 1);  /* 2 * i_b */
    q15_t sum = q15_add(i_a, two_i_b);  /* i_a + 2*i_b */

    /* 乘以 1/sqrt(3) ≈ 0.57735 (Q15: 18919) */
    /* sum (Q15) * 18919 (Q15) -> 结果需要右移 15 位 */
    int32_t beta_temp = (int32_t)sum * 18919;
    beta_temp = (beta_temp + (1 << 14)) >> 15;  /* 四舍五入右移 */
    if (beta_temp > 32767) beta_temp = 32767;
    if (beta_temp < -32768) beta_temp = -32768;
    *i_beta = (q15_t)beta_temp;
}

/* ============================================================================
 * 死区补偿参考电压计算
 * ============================================================================ */

void deadbeat_voltage_ref_q15(const CurrentStateQ15 *x,
                              const CurrentRefQ15 *r,
                              const MotorParamsQ15 *params,
                              VoltageQ15 *v_ref) {
    /* Deadbeat 控制思想：让下一拍电流直接到参考值
     * 由 i(k+1) = a*i(k) + b*v(k) = i_ref，反解 v(k)：
     * v(k) = (i_ref - a*i(k)) / b
     *
     * 定点实现：
     * v = (i_ref - a*i/b_coeff_scale * b_coeff) / b
     * 由于 a 和 b 都是 Q30，需要做相应移位处理
     */

    /* 计算 alpha 轴电压参考 */
    /* 当前电流对预测的贡献: a * i_alpha，结果是 Q15 */
    q15_t a_i_alpha = q30_mac_q15(0, params->a_coeff, x->i_alpha);

    /* i_ref - a*i，注意这是定点减法 */
    q15_t diff_alpha = q15_add(r->i_alpha_ref, q15_neg(a_i_alpha));

    /* 除以 b 得到电压: (i_ref - a*i) / b
     * diff (Q15) / b (Q30) = diff (Q15) * (1/b) (Q30) >> 30
     * 这里我们使用近似方法：diff << 15 / (b >> 15)
     */
    /* 简化实现：v = diff / b * 1.0，其中 b 是 Q30，所以 v = diff * (1/b) */
    /* 先将 diff 扩展到 Q30，然后除以 b_coeff */
    int32_t diff_alpha_32 = (int32_t)diff_alpha << 15;  /* Q15 -> Q30 */
    int32_t b_val = params->b_coeff;
    /* 除法: diff / b，结果为 Q15 */
    int32_t v_alpha_val;
    if (b_val != 0) {
        v_alpha_val = (diff_alpha_32 / b_val) >> 15;
    } else {
        v_alpha_val = 0;
    }
    /* 饱和处理 */
    if (v_alpha_val > 32767) v_alpha_val = 32767;
    if (v_alpha_val < -32768) v_alpha_val = -32768;
    v_ref->v_alpha = (q15_t)v_alpha_val;

    /* 计算 beta 轴电压参考 */
    q15_t a_i_beta = q30_mac_q15(0, params->a_coeff, x->i_beta);
    q15_t diff_beta = q15_add(r->i_beta_ref, q15_neg(a_i_beta));

    int32_t diff_beta_32 = (int32_t)diff_beta << 15;
    int32_t v_beta_val;
    if (b_val != 0) {
        v_beta_val = (diff_beta_32 / b_val) >> 15;
    } else {
        v_beta_val = 0;
    }
    if (v_beta_val > 32767) v_beta_val = 32767;
    if (v_beta_val < -32768) v_beta_val = -32768;
    v_ref->v_beta = (q15_t)v_beta_val;

    /* 电压限幅：根据母线电压限制输出电压范围
     * 两电平 SVPWM 的最大输出电压约为 v_dc / sqrt(3)
     * 这里简化为 ±v_dc/2
     */
    q15_t v_limit = q15_shr(params->v_dc, 1);  /* v_dc / 2 */

    if (v_ref->v_alpha > v_limit) v_ref->v_alpha = v_limit;
    if (v_ref->v_alpha < -v_limit) v_ref->v_alpha = -v_limit;
    if (v_ref->v_beta > v_limit) v_ref->v_beta = v_limit;
    if (v_ref->v_beta < -v_limit) v_ref->v_beta = -v_limit;
}
