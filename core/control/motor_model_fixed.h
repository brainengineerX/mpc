/**
 * @file motor_model_fixed.h
 * @brief 定点化电机模型头文件
 *
 * 本文件提供定点化电机模型的接口，包括：
 * - 电流预测计算 (i(k+1) = a*i(k) + b*v(k))
 * - Park/Clark 变换（查表法实现）
 * - 死区补偿参考电压计算
 *
 * 所有定点数使用 Q15 格式，中间计算使用 Q30 格式
 */

#ifndef MOTOR_MODEL_FIXED_H
#define MOTOR_MODEL_FIXED_H

#include "fixed_point_types.h"
#include "fixed_point_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 数据结构定义
 * ============================================================================ */

/**
 * @brief 定点化电机模型电流状态
 */
typedef struct {
    q15_t i_alpha;  /**< alpha 轴电流 (Q15)，单位 A */
    q15_t i_beta;   /**< beta 轴电流 (Q15)，单位 A */
} CurrentStateQ15;

/**
 * @brief 定点化参考电流
 */
typedef struct {
    q15_t i_alpha_ref;  /**< alpha 轴参考电流 (Q15) */
    q15_t i_beta_ref;   /**< beta 轴参考电流 (Q15) */
} CurrentRefQ15;

/**
 * @brief 定点化电压
 */
typedef struct {
    q15_t v_alpha;  /**< alpha 轴电压 (Q15)，单位 V */
    q15_t v_beta;   /**< beta 轴电压 (Q15)，单位 V */
} VoltageQ15;

/**
 * @brief 定点化电机参数
 *
 * 离散模型: i(k+1) = a*i(k) + b*v(k)
 * 其中 a = 1 - Ts*Rs/Ls, b = Ts/Ls
 */
typedef struct {
    q30_t a_coeff;      /**< 离散模型系数 a (Q30)，a = 1 - Ts*Rs/Ls */
    q30_t b_coeff;      /**< 离散模型系数 b (Q30)，b = Ts/Ls */
    q15_t v_dc;         /**< 母线电压 (Q15)，单位 V */
    q15_t i_base;       /**< 电流基准值 (Q15)，用于标幺化 */
    q15_t v_base;       /**< 电压基准值 (Q15)，用于标幺化 */
} MotorParamsQ15;

/* ============================================================================
 * 函数声明
 * ============================================================================ */

/**
 * @brief 初始化定点化电机参数
 *
 * 将浮点参数转换为定点格式，计算离散系数 a 和 b
 *
 * @param[out] params 定点化电机参数结构体
 * @param[in] Rs 定子电阻 (Ω)
 * @param[in] Ls 定子电感 (H)
 * @param[in] Ts 控制周期 (s)
 * @param[in] Vdc 直流母线电压 (V)
 * @param[in] i_base 电流基准值 (A)，用于标幺化
 * @param[in] v_base 电压基准值 (V)，用于标幺化
 */
void motor_params_q15_init(MotorParamsQ15 *params,
                           float Rs, float Ls, float Ts,
                           float Vdc, float i_base, float v_base);

/**
 * @brief 定点化电流预测
 *
 * 计算下一拍电流预测值：i(k+1) = a*i(k) + b*v(k)
 *
 * @param[in] x 当前电流状态 (Q15)
 * @param[in] v 当前电压 (Q15)
 * @param[in] params 电机参数，包含系数 a 和 b (Q30)
 * @return 预测的下一拍电流状态 (Q15)
 */
CurrentStateQ15 motor_predict_current_q15(const CurrentStateQ15 *x,
                                          const VoltageQ15 *v,
                                          const MotorParamsQ15 *params);

/**
 * @brief 定点化 Park 变换 (alpha-beta to d-q)
 *
 * 使用查表法计算三角函数：
 * i_d =  i_alpha * cos(theta) + i_beta * sin(theta)
 * i_q = -i_alpha * sin(theta) + i_beta * cos(theta)
 *
 * @param[in] i_alpha alpha 轴电流 (Q15)
 * @param[in] i_beta beta 轴电流 (Q15)
 * @param[in] theta_e 电角度 (Q15)，范围 [-32768, 32767] 对应 [-pi, pi]
 * @param[out] i_d d 轴电流 (Q15)
 * @param[out] i_q q 轴电流 (Q15)
 */
void alphabeta_to_dq_q15(q15_t i_alpha, q15_t i_beta,
                         q15_t theta_e,
                         q15_t *i_d, q15_t *i_q);

/**
 * @brief 定点化逆 Park 变换 (d-q to alpha-beta)
 *
 * 使用查表法计算三角函数：
 * i_alpha = i_d * cos(theta) - i_q * sin(theta)
 * i_beta  = i_d * sin(theta) + i_q * cos(theta)
 *
 * @param[in] i_d d 轴电流 (Q15)
 * @param[in] i_q q 轴电流 (Q15)
 * @param[in] theta_e 电角度 (Q15)
 * @param[out] i_alpha alpha 轴电流 (Q15)
 * @param[out] i_beta beta 轴电流 (Q15)
 */
void dq_to_alphabeta_q15(q15_t i_d, q15_t i_q,
                         q15_t theta_e,
                         q15_t *i_alpha, q15_t *i_beta);

/**
 * @brief 定点化 Clarke 变换 (a-b-c to alpha-beta)
 *
 * 等幅值变换：
 * i_alpha = (2/3) * (i_a - 0.5*i_b - 0.5*i_c)
 * i_beta  = (1/sqrt(3)) * (i_b - i_c)
 *
 * 假设三相电流平衡：i_a + i_b + i_c = 0，简化后：
 * i_alpha = i_a
 * i_beta  = (i_a + 2*i_b) / sqrt(3)
 *
 * @param[in] i_a a 相电流 (Q15)
 * @param[in] i_b b 相电流 (Q15)
 * @param[in] i_c c 相电流 (Q15)
 * @param[out] i_alpha alpha 轴电流 (Q15)
 * @param[out] i_beta beta 轴电流 (Q15)
 */
void abc_to_alphabeta_q15(q15_t i_a, q15_t i_b, q15_t i_c,
                          q15_t *i_alpha, q15_t *i_beta);

/**
 * @brief 定点化死区补偿参考电压计算
 *
 * 根据当前电流状态和参考电流，反推理想电压：
 * v_ref = (i_ref - a*i) / b
 *
 * 结果会限幅到逆变器输出能力范围内
 *
 * @param[in] x 当前电流状态 (Q15)
 * @param[in] r 参考电流 (Q15)
 * @param[in] params 电机参数，包含系数 a, b 和母线电压
 * @param[out] v_ref 输出参考电压 (Q15)
 */
void deadbeat_voltage_ref_q15(const CurrentStateQ15 *x,
                              const CurrentRefQ15 *r,
                              const MotorParamsQ15 *params,
                              VoltageQ15 *v_ref);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MODEL_FIXED_H */
