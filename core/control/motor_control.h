#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：motor_control
 * 职责：实现电流环有限控制集MPC（FCS-MPC）核心。
 *
 * 控制流程（每个控制周期）：
 * 1) 根据当前电流状态和参考，反推理想连续电压 v_ref；
 * 2) 枚举有限个逆变器开关状态（8个候选）；
 * 3) 计算每个候选在“有功矢量占空比+零矢量”下的等效电压；
 * 4) 用简化模型预测下一拍电流；
 * 5) 计算代价函数并选最优候选；
 * 6) 输出最优电压与开关状态。
 *
 * 说明：
 * - 本模块只关注控制算法，不直接依赖ADC/PWM/编码器硬件。
 * - 实际工程通过HAL层把本模块与BSP解耦。
 * ========================================================================== */

typedef struct {
    int switch_a;
    int switch_b;
    int switch_c;

    /* 等效输出电压（考虑有功矢量占空比） */
    float v_alpha;
    float v_beta;

    /* 有功矢量占空比及其本体 */
    float duty_active;
    float v_alpha_active;
    float v_beta_active;

    /* 总代价与分项代价（用于调试与在线整定） */
    float cost;
    float cost_current;
    float cost_switch;
    float cost_voltage;

    /* 连续理想电压参考 */
    float v_alpha_ref;
    float v_beta_ref;
} MpcOutput;

typedef struct {
    /* 电流跟踪误差权重 */
    float w_current;
    /* 开关变化惩罚权重（抑制开关频率） */
    float w_switch;
    /* 电压参考误差权重（让离散电压逼近连续理想电压） */
    float w_voltage;
    /* 自适应开关松弛增益：误差大时降低开关惩罚 */
    float switch_relax_gain;
} MpcWeights;

typedef struct {
    MotorParams motor;
    MpcWeights weights;

    int last_sa;
    int last_sb;
    int last_sc;

    /* 简化模型离散系数：i(k+1)=a*i+b*v */
    float a;
    float b;

    /* 二步预测相关配置：
     * two_step_enable=1 时启用“预筛选二步预测”；
     * two_step_top_k 表示第一步保留的候选数（建议 2~3）；
     * two_step_weight 表示第二步代价权重。 */
    int two_step_enable;
    int two_step_top_k;
    float two_step_weight;

    /* 标幺化配置：
     * pu_enable=1 时，代价函数中的电流/电压误差会先除以基值；
     * i_base/v_base 分别是电流基值和电压基值。 */
    int pu_enable;
    float i_base;
    float v_base;
} MpcController;

/* 候选层：开关状态映射到 alpha-beta 电压 */
void mpc_candidate_switch_to_voltage(float vdc,
                                     int sa, int sb, int sc,
                                     float *v_alpha, float *v_beta);

/* 代价层：返回单个候选的总代价。
 * 其中 cost_current/cost_switch/cost_voltage 是可选输出，
 * 便于在线观察各项权重对行为的影响。 */
float mpc_cost_evaluate(const CurrentState *x_pred,
                        const CurrentRef *r,
                        const MpcWeights *weights,
                        float v_alpha, float v_beta,
                        float v_alpha_ref, float v_beta_ref,
                        float switch_activity,
                        int sa, int sb, int sc,
                        int last_sa, int last_sb, int last_sc,
                        float *cost_current,
                        float *cost_switch,
                        float *cost_voltage);

/* 初始化控制器上下文（参数、权重、离散系数、上拍开关状态） */
void mpc_controller_init(MpcController *ctrl,
                         const MotorParams *motor,
                         const MpcWeights *weights,
                         int init_sa, int init_sb, int init_sc);

/* 母线电压实时前馈：每周期刷新 Vdc，
 * 让候选矢量幅值跟随母线波动，减小模型失配。 */
void mpc_controller_update_vdc(MpcController *ctrl, float vdc_meas);

/* 设置“预筛选二步预测”参数：
 * - enable：0=一步预测，1=预筛选二步预测
 * - top_k：第一步保留候选数（会自动限幅到 [1,8]）
 * - second_weight：第二步代价权重（建议 0.5~1.5） */
void mpc_controller_set_two_step(MpcController *ctrl,
                                 int enable,
                                 int top_k,
                                 float second_weight);

/* 设置标幺化参数：
 * - enable：0=关闭，1=开启
 * - i_base：电流基值（A）
 * - v_base：电压基值（V） */
void mpc_controller_set_per_unit(MpcController *ctrl,
                                 int enable,
                                 float i_base,
                                 float v_base);

/* 优化器单步：输入当前电流和参考，输出最优控制结果 */
MpcOutput mpc_controller_step(MpcController *ctrl,
                              const CurrentState *x,
                              const CurrentRef *r);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
