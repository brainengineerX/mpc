/**
 * @file mpc_controller_fixed.c
 * @brief 定点化 MPC 电流环控制器实现
 */

#include "mpc_controller_fixed.h"
#include <string.h>

/* ============================================================================
 * 内部常量
 * ============================================================================ */

/** 逆变器开关状态表 (8种组合) */
static const SwitchState switch_states[8] = {
    {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
    {0, 1, 1}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}
};

/* ============================================================================
 * 初始化函数
 * ============================================================================ */

void mpc_controller_q15_init(
    MpcControllerQ15 *ctrl,
    const MotorParamsQ15 *motor_params,
    const MpcWeightsQ15 *weights,
    const SwitchState *init_switch) {

    /* 复制参数 */
    ctrl->motor_params = *motor_params;
    ctrl->weights = *weights;

    /* 初始化开关状态 */
    ctrl->last_switch = *init_switch;

    /* 初始化 Top-K 配置 */
    ctrl->topk_cfg.k_steady = MPC_TOP_K_STEADY;
    ctrl->topk_cfg.k_dynamic = MPC_TOP_K_DYNAMIC;
    ctrl->topk_cfg.err_threshold = MPC_ERR_THRESHOLD_Q15;
    ctrl->topk_cfg.k_current = MPC_TOP_K_STEADY;

    /* 初始化两步预测配置 */
    ctrl->two_step_enable = 1;  /* 默认使能 */
    ctrl->step2_weight = MPC_STEP2_WEIGHT_Q15;

    /* 初始化状态 */
    ctrl->i_prev.i_alpha = 0;
    ctrl->i_prev.i_beta = 0;
    ctrl->vdc_measured = VOLT_TO_Q15(48.0f);  /* 默认 48V */
}

void mpc_controller_q15_set_topk(
    MpcControllerQ15 *ctrl,
    int8_t k_steady,
    int8_t k_dynamic,
    q15_t err_threshold) {

    ctrl->topk_cfg.k_steady = k_steady;
    ctrl->topk_cfg.k_dynamic = k_dynamic;
    ctrl->topk_cfg.err_threshold = err_threshold;
    ctrl->topk_cfg.k_current = k_steady;
}

void mpc_controller_q15_update_vdc(MpcControllerQ15 *ctrl, q15_t vdc) {
    ctrl->vdc_measured = vdc;
    ctrl->motor_params.v_dc = vdc;
}

/* ============================================================================
 * 开关到电压转换
 * ============================================================================ */

/**
 * @brief 开关状态到 alpha-beta 电压转换
 *
 * 公式:
 * v_alpha = (2*sa - sb - sc) * vdc / 3
 * v_beta = (sb - sc) * vdc / sqrt(3)
 *
 * 定点化:
 * v_alpha = ((2*sa - sb - sc) * vdc) / 3
 * v_beta = ((sb - sc) * vdc * 18919) >> 15  (18919 = round(32767/sqrt(3)))
 */
static void switch_to_voltage_q15(
    const SwitchState *sw,
    q15_t vdc,
    VoltageQ15 *v_out) {

    int32_t sa = sw->sa;
    int32_t sb = sw->sb;
    int32_t sc = sw->sc;

    /* 计算有效电压矢量 */
    int32_t v_alpha_num = (2 * sa - sb - sc);
    int32_t v_beta_num = (sb - sc);

    /* v_alpha = v_alpha_num * vdc / 3 */
    v_out->v_alpha = (q15_t)((v_alpha_num * (int32_t)vdc) / 3);

    /* v_beta = v_beta_num * vdc / sqrt(3) */
    /* 乘以 18919 (1/sqrt(3) in Q15)，然后右移 15 */
    int32_t v_beta_temp = (v_beta_num * (int32_t)vdc * 18919) >> 15;
    v_out->v_beta = (q15_t)v_beta_temp;
}

/* ============================================================================
 * 代价函数计算
 * ============================================================================ */

/**
 * @brief 计算代价函数
 *
 * J = w_i * ||i - i_ref||^2 + w_sw * N_sw + w_v * ||v - v_ref||^2
 *
 * @param i_pred 预测电流
 * @param i_ref 电流参考
 * @param v_vector 电压矢量
 * @param v_ref 电压参考
 * @param weights 权重
 * @param switch_count 开关翻转次数
 * @param cost 输出代价
 */
static void compute_cost_q15(
    const CurrentStateQ15 *i_pred,
    const CurrentRefQ15 *i_ref,
    const VoltageVectorQ15 *v_vector,
    const VoltageVectorQ15 *v_ref,
    const MpcWeightsQ15 *weights,
    int8_t switch_count,
    MpcCostQ15 *cost) {

    /* 电流误差: i_err = i_pred - i_ref */
    q15_t i_alpha_err = q15_sub(i_pred->i_alpha, i_ref->i_alpha_ref);
    q15_t i_beta_err = q15_sub(i_pred->i_beta, i_ref->i_beta_ref);

    /* 电流代价: w_i * (i_alpha_err^2 + i_beta_err^2) */
    q15_t i_alpha_err_sq = q15_square_lut(i_alpha_err);
    q15_t i_beta_err_sq = q15_square_lut(i_beta_err);
    q15_t i_err_sum = q15_add(i_alpha_err_sq, i_beta_err_sq);
    cost->j_current = q15_mul(weights->w_current, i_err_sum);

    /* 开关代价: w_sw * switch_count */
    cost->j_switch = q15_mul(weights->w_switch, (q15_t)(switch_count << 10));

    /* 电压误差 */
    q15_t v_alpha_err = q15_sub(v_vector->v_alpha, v_ref->v_alpha);
    q15_t v_beta_err = q15_sub(v_vector->v_beta, v_ref->v_beta);

    /* 电压代价: w_v * (v_alpha_err^2 + v_beta_err^2) */
    q15_t v_alpha_err_sq = q15_square_lut(v_alpha_err);
    q15_t v_beta_err_sq = q15_square_lut(v_beta_err);
    q15_t v_err_sum = q15_add(v_alpha_err_sq, v_beta_err_sq);
    cost->j_voltage = q15_mul(weights->w_voltage, v_err_sum);

    /* 总代价 */
    cost->j_total = q15_add(cost->j_current, cost->j_switch);
    cost->j_total = q15_add(cost->j_total, cost->j_voltage);
}

/* ============================================================================
 * 候选评估
 * ============================================================================ */

void mpc_evaluate_candidate_q15(
    const MpcControllerQ15 *ctrl,
    const CurrentStateQ15 *x_now,
    const CurrentRefQ15 *i_ref,
    q15_t v_ref_alpha,
    q15_t v_ref_beta,
    const SwitchState *sw,
    MpcCandidateQ15 *candidate) {

    VoltageVectorQ15 v_ref, v_vector;

    /* 保存开关状态 */
    memcpy(&candidate->sw, sw, sizeof(SwitchState));

    /* 开关状态到电压矢量 */
    switch_to_voltage_q15(sw, ctrl->vdc_measured, &v_vector);
    memcpy(&candidate->v_vector, &v_vector, sizeof(VoltageVectorQ15));

    /* 构建电压参考 */
    v_ref.v_alpha = v_ref_alpha;
    v_ref.v_beta = v_ref_beta;

    /* 电流预测: i(k+1) = a*i(k) + b*v(k) */
    candidate->x_pred = motor_predict_current_q15(x_now, &v_vector, &ctrl->motor_params);

    /* 计算开关翻转次数 */
    int8_t switch_count = 0;
    switch_count += (sw->sa != ctrl->last_switch.sa) ? 1 : 0;
    switch_count += (sw->sb != ctrl->last_switch.sb) ? 1 : 0;
    switch_count += (sw->sc != ctrl->last_switch.sc) ? 1 : 0;

    /* 计算代价 */
    compute_cost_q15(
        &candidate->x_pred,
        i_ref,
        &v_vector,
        &v_ref,
        &ctrl->weights,
        switch_count,
        &candidate->cost
    );

    candidate->valid = 1;
}

/* ============================================================================
 * Top-K 筛选
 * ============================================================================ */

void mpc_insert_topk_q15(
    MpcCandidateQ15 *topk,
    int8_t top_k,
    const MpcCandidateQ15 *candidate) {

    int8_t worst_idx = 0;
    q15_t worst_cost = topk[0].cost.j_total;
    int8_t i;

    /* 查找当前 Top-K 中的最差候选 */
    for (i = 0; i < top_k; i++) {
        if (!topk[i].valid) {
            /* 空槽位，直接插入 */
            topk[i] = *candidate;
            return;
        }
        if (topk[i].cost.j_total > worst_cost) {
            worst_cost = topk[i].cost.j_total;
            worst_idx = i;
        }
    }

    /* 如果新候选比最差候选更好，替换它 */
    if (candidate->cost.j_total < worst_cost) {
        topk[worst_idx] = *candidate;
    }
}

/**
 * @brief 动态选择 Top-K 值
 *
 * 根据电流误差大小选择 K 值
 * - 误差小: K = k_steady (快速)
 * - 误差大: K = k_dynamic (精确)
 */
static int8_t select_topk_q15(
    const AdaptiveTopKQ15 *topk_cfg,
    const CurrentStateQ15 *x_now,
    const CurrentRefQ15 *i_ref) {

    /* 计算电流误差 */
    q15_t i_alpha_err = q15_sub(x_now->i_alpha, i_ref->i_alpha_ref);
    q15_t i_beta_err = q15_sub(x_now->i_beta, i_ref->i_beta_ref);

    /* 误差平方和 */
    q15_t err_sq = q15_add(
        q15_square_lut(i_alpha_err),
        q15_square_lut(i_beta_err)
    );

    /* 阈值比较 */
    if (err_sq > topk_cfg->err_threshold) {
        return topk_cfg->k_dynamic;
    } else {
        return topk_cfg->k_steady;
    }
}

/* ============================================================================
 * 死区补偿参考电压计算
 * ============================================================================ */

/**
 * @brief 死区补偿参考电压计算 (定点化)
 *
 * 公式: v_ref = (i_ref - a * i) / b
 */
void deadbeat_voltage_ref_q15(
    const CurrentStateQ15 *x,
    const CurrentRefQ15 *r,
    const MotorParamsQ15 *params,
    VoltageQ15 *v_ref) {

    /* v_alpha_ref = (i_alpha_ref - a * i_alpha) / b */
    q15_t a_i_alpha = q30_mul_q15(params->a_coeff, x->i_alpha);
    q15_t num_alpha = q15_sub(r->i_alpha_ref, a_i_alpha);

    /* 除以 b: 乘以 b 的倒数近似 */
    /* 实际应用中，b 通常在 (0, 1) 范围内 */
    /* 这里使用简化的除法实现 */
    if (params->b_coeff != 0) {
        /* 近似: v_ref ≈ num * (1/b) * scale */
        /* 更精确的做法是使用牛顿迭代求 1/b */
        v_ref->v_alpha = q15_div_lut(num_alpha, FLOAT_TO_Q15(Q30_TO_FLOAT(params->b_coeff)));
    } else {
        v_ref->v_alpha = 0;
    }

    /* v_beta_ref 同理 */
    q15_t a_i_beta = q30_mul_q15(params->a_coeff, x->i_beta);
    q15_t num_beta = q15_sub(r->i_beta_ref, a_i_beta);

    if (params->b_coeff != 0) {
        v_ref->v_beta = q15_div_lut(num_beta, FLOAT_TO_Q15(Q30_TO_FLOAT(params->b_coeff)));
    } else {
        v_ref->v_beta = 0;
    }
}

/* ============================================================================
 * MPC 单步控制
 * ============================================================================ */

SwitchState mpc_controller_q15_step(
    MpcControllerQ15 *ctrl,
    const CurrentStateQ15 *x_now,
    const CurrentRefQ15 *i_ref) {

    MpcCandidateQ15 topk[8];
    int8_t top_k;
    int i;
    SwitchState best_switch;
    MpcCostQ15 best_cost;

    /* 初始化 Top-K 数组 */
    for (i = 0; i < 8; i++) {
        topk[i].valid = 0;
        topk[i].cost.j_total = Q15_ONE;  /* 最大代价 */
    }

    /* 动态选择 Top-K 值 */
    top_k = select_topk_q15(&ctrl->topk_cfg, x_now, i_ref);

    /* 计算参考电压 */
    VoltageVectorQ15 v_ref;
    deadbeat_voltage_ref_q15(x_now, i_ref, &ctrl->motor_params, &v_ref);

    /* 评估所有候选 */
    for (i = 0; i < 8; i++) {
        MpcCandidateQ15 candidate;

        mpc_evaluate_candidate_q15(
            ctrl,
            x_now,
            i_ref,
            v_ref.v_alpha,
            v_ref.v_beta,
            &switch_states[i],
            &candidate
        );

        mpc_insert_topk_q15(topk, top_k, &candidate);
    }

    /* 选择最优候选 */
    best_cost.j_total = Q15_ONE;
    for (i = 0; i < top_k; i++) {
        if (topk[i].valid && topk[i].cost.j_total < best_cost.j_total) {
            best_cost = topk[i].cost;
            best_switch = topk[i].sw;
        }
    }

    /* 更新状态 */
    memcpy(&ctrl->last_switch, &best_switch, sizeof(SwitchState));
    memcpy(&ctrl->i_prev, x_now, sizeof(CurrentStateQ15));

    return best_switch;
}

/* ============================================================================
 * 两步预测 MPC 控制
 * ============================================================================ */

SwitchState mpc_controller_q15_step_two_stage(
    MpcControllerQ15 *ctrl,
    const CurrentStateQ15 *x_now,
    const CurrentRefQ15 *i_ref) {

    MpcCandidateQ15 topk[8];
    MpcCandidateQ15 best_candidate;
    int8_t top_k;
    int i, j;
    VoltageVectorQ15 v_ref;

    /* 初始化 */
    for (i = 0; i < 8; i++) {
        topk[i].valid = 0;
        topk[i].cost.j_total = Q15_ONE;
    }
    best_candidate.cost.j_total = Q15_ONE;

    /* 动态选择 Top-K */
    top_k = select_topk_q15(&ctrl->topk_cfg, x_now, i_ref);

    /* 第一步参考电压 */
    deadbeat_voltage_ref_q15(x_now, i_ref, &ctrl->motor_params, &v_ref);

    /* 第一步：Top-K 预筛选 */
    for (i = 0; i < 8; i++) {
        MpcCandidateQ15 candidate;

        mpc_evaluate_candidate_q15(
            ctrl,
            x_now,
            i_ref,
            v_ref.v_alpha,
            v_ref.v_beta,
            &switch_states[i],
            &candidate
        );

        mpc_insert_topk_q15(topk, top_k, &candidate);
    }

    /* 第二步：对 Top-K 候选展开第二步预测 */
    for (i = 0; i < top_k; i++) {
        if (!topk[i].valid) continue;

        /* 基于 x(k+1) 计算第二步参考电压 */
        CurrentRefQ15 i_ref_step2;
        i_ref_step2.i_alpha_ref = i_ref->i_alpha_ref;
        i_ref_step2.i_beta_ref = i_ref->i_beta_ref;

        VoltageVectorQ15 v_ref_step2;
        deadbeat_voltage_ref_q15(&topk[i].x_pred, &i_ref_step2, &ctrl->motor_params, &v_ref_step2);

        /* 第二步最优代价 */
        q15_t best_step2_cost = Q15_ONE;

        for (j = 0; j < 8; j++) {
            MpcCandidateQ15 candidate2;

            /* 临时更新 last_switch 用于第二步评估 */
            SwitchState saved_last = ctrl->last_switch;
            ctrl->last_switch = topk[i].sw;

            mpc_evaluate_candidate_q15(
                ctrl,
                &topk[i].x_pred,
                &i_ref_step2,
                v_ref_step2.v_alpha,
                v_ref_step2.v_beta,
                &switch_states[j],
                &candidate2
            );

            /* 恢复 last_switch */
            ctrl->last_switch = saved_last;

            /* 两步总代价: J = J1 + w2 * J2 */
            q15_t step2_cost = q15_mul(ctrl->step2_weight, candidate2.cost.j_total);
            q15_t total_cost = q15_add(topk[i].cost.j_total, step2_cost);

            if (total_cost < best_step2_cost) {
                best_step2_cost = total_cost;
            }
        }

        /* 更新总代价 */
        if (best_step2_cost < best_candidate.cost.j_total) {
            best_candidate = topk[i];
            best_candidate.cost.j_total = best_step2_cost;
        }
    }

    /* 更新状态 */
    memcpy(&ctrl->last_switch, &best_candidate.sw, sizeof(SwitchState));
    memcpy(&ctrl->i_prev, x_now, sizeof(CurrentStateQ15));

    return best_candidate.sw;
}
