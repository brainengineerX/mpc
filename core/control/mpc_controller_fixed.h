/**
 * @file mpc_controller_fixed.h
 * @brief 定点化 MPC 电流环控制器
 *
 * 本文件实现基于定点数 (Q15/Q30) 的模型预测控制 (MPC) 电流环控制器，
 * 目标是在 Cortex-M7 平台上实现 50kHz 控制频率。
 *
 * 核心特性：
 * - 定点化电流预测 (Q15)
 * - 定点化代价函数计算
 * - 动态 Top-K 候选筛选
 * - 两步预测算法
 */

#ifndef MPC_CONTROLLER_FIXED_H
#define MPC_CONTROLLER_FIXED_H

#include "fixed_point_types.h"
#include "fixed_point_math.h"
#include "motor_model_fixed.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 常量定义
 * ============================================================================ */

/** 逆变器开关状态数 (2级3相 = 8种状态) */
#define MPC_SWITCH_STATE_COUNT  8

/** 默认 Top-K 值 (稳态) */
#define MPC_TOP_K_STEADY        1

/** 动态 Top-K 最大值 */
#define MPC_TOP_K_DYNAMIC       4

/** 电流误差阈值 (Q15)，超过此值启用动态 Top-K */
#define MPC_ERR_THRESHOLD_Q15   1024  /* 约 1A */

/** 两步预测权重 (Q15)，第二步代价的权重 */
#define MPC_STEP2_WEIGHT_Q15    16384  /* 0.5 in Q15 */

/* ============================================================================
 * 数据类型定义
 * ============================================================================ */

/**
 * @brief 定点化 MPC 权重
 *
 * 代价函数: J = w_i * ||i - i_ref||^2 + w_sw * N_sw + w_v * ||v - v_ref||^2
 */
typedef struct {
    q15_t w_current;    /* 电流跟踪权重 (Q15) */
    q15_t w_switch;     /* 开关惩罚权重 (Q15) */
    q15_t w_voltage;    /* 电压逼近权重 (Q15) */
} MpcWeightsQ15;

/**
 * @brief 开关候选状态
 */
typedef struct {
    int8_t sa;          /* A相开关状态: 0 或 1 */
    int8_t sb;          /* B相开关状态: 0 或 1 */
    int8_t sc;          /* C相开关状态: 0 或 1 */
} SwitchState;

/**
 * @brief 定点化电压矢量 (alpha-beta坐标系)
 * @note 使用 motor_model_fixed.h 中的 VoltageQ15
 */
typedef VoltageQ15 VoltageVectorQ15;

/**
 * @brief 定点化代价分量
 */
typedef struct {
    q15_t j_current;    /* 电流跟踪代价 (Q15) */
    q15_t j_switch;     /* 开关惩罚代价 (Q15) */
    q15_t j_voltage;    /* 电压逼近代价 (Q15) */
    q15_t j_total;      /* 总代价 (Q15) */
} MpcCostQ15;

/**
 * @brief 候选评估结果 (用于 Top-K 筛选)
 */
typedef struct {
    SwitchState sw;              /* 开关状态 */
    VoltageVectorQ15 v_vector;   /* 电压矢量 */
    CurrentStateQ15 x_pred;      /* 预测电流状态 */
    MpcCostQ15 cost;             /* 代价 */
    int8_t valid;                /* 是否有效 */
} MpcCandidateQ15;

/**
 * @brief 动态 Top-K 配置
 */
typedef struct {
    int8_t k_steady;        /* 稳态 Top-K */
    int8_t k_dynamic;       /* 动态 Top-K 最大值 */
    q15_t err_threshold;    /* 切换阈值 (Q15) */
    int8_t k_current;       /* 当前 Top-K */
} AdaptiveTopKQ15;

/**
 * @brief 定点化 MPC 控制器状态
 */
typedef struct {
    /* 参数 */
    MotorParamsQ15 motor_params;      /* 电机参数 */
    MpcWeightsQ15 weights;            /* 代价权重 */
    AdaptiveTopKQ15 topk_cfg;         /* Top-K 配置 */

    /* 状态 */
    SwitchState last_switch;          /* 上一拍开关状态 */
    CurrentStateQ15 i_prev;           /* 上一拍电流 */
    q15_t vdc_measured;               /* 母线电压测量值 (Q15) */

    /* 配置 */
    int8_t two_step_enable;           /* 两步预测使能 */
    q15_t step2_weight;               /* 第二步权重 (Q15) */
} MpcControllerQ15;

/* ============================================================================
 * 函数声明
 * ============================================================================ */

/**
 * @brief 初始化 MPC 控制器
 *
 * @param ctrl 控制器实例
 * @param motor_params 电机参数
 * @param weights 代价权重
 * @param init_switch 初始开关状态
 */
void mpc_controller_q15_init(
    MpcControllerQ15 *ctrl,
    const MotorParamsQ15 *motor_params,
    const MpcWeightsQ15 *weights,
    const SwitchState *init_switch
);

/**
 * @brief 设置动态 Top-K 配置
 *
 * @param ctrl 控制器实例
 * @param k_steady 稳态 Top-K
 * @param k_dynamic 动态 Top-K 最大值
 * @param err_threshold 切换阈值 (Q15)
 */
void mpc_controller_q15_set_topk(
    MpcControllerQ15 *ctrl,
    int8_t k_steady,
    int8_t k_dynamic,
    q15_t err_threshold
);

/**
 * @brief 更新母线电压
 *
 * @param ctrl 控制器实例
 * @param vdc 母线电压 (Q15)
 */
void mpc_controller_q15_update_vdc(
    MpcControllerQ15 *ctrl,
    q15_t vdc
);

/**
 * @brief 评估单个开关候选
 *
 * @param ctrl 控制器实例
 * @param x_now 当前电流状态
 * @param i_ref 电流参考
 * @param v_ref_alpha 电压参考 alpha 分量
 * @param v_ref_beta 电压参考 beta 分量
 * @param sw 开关状态
 * @param candidate 输出候选结果
 */
void mpc_evaluate_candidate_q15(
    const MpcControllerQ15 *ctrl,
    const CurrentStateQ15 *x_now,
    const CurrentRefQ15 *i_ref,
    q15_t v_ref_alpha,
    q15_t v_ref_beta,
    const SwitchState *sw,
    MpcCandidateQ15 *candidate
);

/**
 * @brief 插入 Top-K 列表
 *
 * @param topk Top-K 数组
 * @param top_k Top-K 大小
 * @param candidate 新候选
 */
void mpc_insert_topk_q15(
    MpcCandidateQ15 *topk,
    int8_t top_k,
    const MpcCandidateQ15 *candidate
);

/**
 * @brief MPC 单步控制
 *
 * 执行完整的 MPC 优化：
 * 1. 生成候选开关状态
 * 2. 评估代价函数
 * 3. 选择最优开关状态
 *
 * @param ctrl 控制器实例
 * @param x_now 当前电流状态
 * @param i_ref 电流参考
 * @return 最优开关状态
 */
SwitchState mpc_controller_q15_step(
    MpcControllerQ15 *ctrl,
    const CurrentStateQ15 *x_now,
    const CurrentRefQ15 *i_ref
);

/**
 * @brief 两步预测 MPC 控制（高级功能）
 *
 * 执行两步预测优化：
 * 1. 第一步：Top-K 预筛选
 * 2. 第二步：展开最优候选
 *
 * @param ctrl 控制器实例
 * @param x_now 当前电流状态
 * @param i_ref 电流参考
 * @return 最优开关状态
 */
SwitchState mpc_controller_q15_step_two_stage(
    MpcControllerQ15 *ctrl,
    const CurrentStateQ15 *x_now,
    const CurrentRefQ15 *i_ref
);

#ifdef __cplusplus
}
#endif

#endif /* MPC_CONTROLLER_FIXED_H */
