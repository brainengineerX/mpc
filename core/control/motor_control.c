#include "motor_control.h" /* MPC 控制器对外接口 */

/* ==========================================================================
 * core/control/motor_control.c
 * --------------------------------------------------------------------------
 * 这是电流环 MPC 核心：
 * 1) 候选电压矢量生成；
 * 2) 预测下一拍电流；
 * 3) 计算多目标代价函数；
 * 4) 选择最优矢量输出。
 * ======================================================================== */

/* 平方函数：代价函数会频繁使用，封装后可读性更好 */
static float squaref(float x)
{
    return x * x; /* 返回 x^2 */
}

/* 整数绝对值：用于统计开关翻转次数 */
static int abs_int(int x)
{
    return (x >= 0) ? x : -x; /* 三目运算实现绝对值 */
}

/* 限幅函数：约束占空比在 [0,1] 内 */
static float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) {      /* 小于下限 */
        return min_v;     /* 返回下限 */
    }
    if (x > max_v) {      /* 大于上限 */
        return max_v;     /* 返回上限 */
    }
    return x;             /* 区间内原样返回 */
}

/* 安全倒数：避免除以 0 */
static float inv_safe(float x)
{
    if (x > 1.0e-9f) return 1.0f / x;
    if (x < -1.0e-9f) return 1.0f / x;
    return 1.0f;
}

/* 候选缓存：用于“第一步预筛选 Top-K” */
typedef struct {
    int valid;               /* 该槽位是否有效 */
    int sa;                  /* 候选 A 相 */
    int sb;                  /* 候选 B 相 */
    int sc;                  /* 候选 C 相 */
    float duty_active;       /* 有功占空比 */
    float v_alpha_active;    /* 候选有功矢量 alpha */
    float v_beta_active;     /* 候选有功矢量 beta */
    float v_alpha_eq;        /* 候选等效 alpha 电压 */
    float v_beta_eq;         /* 候选等效 beta 电压 */
    float j_total;           /* 候选总代价 */
    float j_current;         /* 候选电流代价 */
    float j_switch;          /* 候选开关代价 */
    float j_voltage;         /* 候选电压代价 */
    CurrentState x_pred;     /* 一步预测电流（用于第二步展开） */
} MpcStageCandidate;

/* 评估单个候选开关，输出“等效电压 + 一步预测 + 一步代价” */
static void mpc_evaluate_single_candidate(const MpcController *ctrl,
                                          const CurrentState *x_now,
                                          const CurrentRef *r,
                                          float v_alpha_ref,
                                          float v_beta_ref,
                                          int sa,
                                          int sb,
                                          int sc,
                                          int prev_sa,
                                          int prev_sb,
                                          int prev_sc,
                                          MpcStageCandidate *out)
{
    float v_alpha = 0.0f;              /* 候选有功矢量 alpha */
    float v_beta = 0.0f;               /* 候选有功矢量 beta */
    float duty_active = 0.0f;          /* 候选有功矢量占空比 */
    float v_alpha_eq = 0.0f;           /* 候选等效 alpha 电压 */
    float v_beta_eq = 0.0f;            /* 候选等效 beta 电压 */
    float j_current = 0.0f;            /* 电流代价分量 */
    float j_switch = 0.0f;             /* 开关代价分量 */
    float j_voltage = 0.0f;            /* 电压代价分量 */
    CurrentState x_pred_eval;          /* 代价计算用电流（可能是标幺值） */
    CurrentRef r_eval;                 /* 代价计算用参考（可能是标幺值） */
    float v_alpha_eval = 0.0f;         /* 代价计算用 alpha 电压（可能是标幺值） */
    float v_beta_eval = 0.0f;          /* 代价计算用 beta 电压（可能是标幺值） */
    float v_alpha_ref_eval = 0.0f;     /* 代价计算用 alpha 参考（可能是标幺值） */
    float v_beta_ref_eval = 0.0f;      /* 代价计算用 beta 参考（可能是标幺值） */

    /* 开关状态映射成有功电压矢量 */
    mpc_candidate_switch_to_voltage(ctrl->motor.Vdc, sa, sb, sc, &v_alpha, &v_beta);

    {
        /* 通过投影得到“有功矢量占空比”，实现有功+零矢量等效输出 */
        const float mag2 = v_alpha * v_alpha + v_beta * v_beta;
        if (mag2 > 1.0e-9f) {
            const float dot = v_alpha_ref * v_alpha + v_beta_ref * v_beta;
            duty_active = clampf(dot / mag2, 0.0f, 1.0f);
            v_alpha_eq = duty_active * v_alpha;
            v_beta_eq = duty_active * v_beta;
        }
    }

    /* 一步预测电流 */
    out->x_pred = motor_model_predict_next(x_now, v_alpha_eq, v_beta_eq, ctrl->a, ctrl->b);

    /* 默认使用物理量直接计算代价 */
    x_pred_eval = out->x_pred;
    r_eval = *r;
    v_alpha_eval = v_alpha_eq;
    v_beta_eval = v_beta_eq;
    v_alpha_ref_eval = v_alpha_ref;
    v_beta_ref_eval = v_beta_ref;

    if (ctrl->pu_enable) { /* 启用标幺化时，统一先归一化再算代价 */
        const float inv_i = inv_safe(ctrl->i_base); /* 电流基值倒数 */
        const float inv_v = inv_safe(ctrl->v_base); /* 电压基值倒数 */

        x_pred_eval.i_alpha *= inv_i;
        x_pred_eval.i_beta *= inv_i;
        r_eval.i_alpha *= inv_i;
        r_eval.i_beta *= inv_i;
        v_alpha_eval *= inv_v;
        v_beta_eval *= inv_v;
        v_alpha_ref_eval *= inv_v;
        v_beta_ref_eval *= inv_v;
    }

    /* 一步代价 */
    out->j_total = mpc_cost_evaluate(&x_pred_eval, &r_eval, &ctrl->weights,
                                     v_alpha_eval, v_beta_eval,
                                     v_alpha_ref_eval, v_beta_ref_eval,
                                     duty_active,
                                     sa, sb, sc,
                                     prev_sa, prev_sb, prev_sc,
                                     &j_current, &j_switch, &j_voltage);

    out->valid = 1;                    /* 标记有效 */
    out->sa = sa;                      /* 保存候选开关状态 */
    out->sb = sb;
    out->sc = sc;
    out->duty_active = duty_active;    /* 保存候选占空比 */
    out->v_alpha_active = v_alpha;     /* 保存候选有功矢量 */
    out->v_beta_active = v_beta;
    out->v_alpha_eq = v_alpha_eq;      /* 保存候选等效电压 */
    out->v_beta_eq = v_beta_eq;
    out->j_current = j_current;        /* 保存分项代价 */
    out->j_switch = j_switch;
    out->j_voltage = j_voltage;
}

/* 把候选插入 Top-K 列表（按总代价从小到大） */
static void mpc_insert_topk(MpcStageCandidate *topk, int top_k, const MpcStageCandidate *cand)
{
    int worst_idx = 0;                           /* 当前最差槽位下标 */
    float worst_cost = topk[0].j_total;          /* 当前最差代价 */

    for (int i = 0; i < top_k; ++i) {            /* 遍历 Top-K 槽位 */
        if (!topk[i].valid) {                    /* 空槽直接占用 */
            topk[i] = *cand;                     /* 写入新候选 */
            return;                              /* 结束 */
        }
        if (topk[i].j_total > worst_cost) {      /* 记录更差候选 */
            worst_cost = topk[i].j_total;        /* 更新最差代价 */
            worst_idx = i;                       /* 更新最差下标 */
        }
    }

    if (cand->j_total < worst_cost) {            /* 只有更优才替换最差槽位 */
        topk[worst_idx] = *cand;                 /* 替换最差候选 */
    }
}

/* 开关状态 -> alpha-beta 电压映射 */
void mpc_candidate_switch_to_voltage(float vdc,
                                     int sa, int sb, int sc,
                                     float *v_alpha, float *v_beta)
{
    *v_alpha = (2.0f * sa - sb - sc) * (vdc / 3.0f);      /* alpha 分量 */
    *v_beta = 0.57735026919f * (sb - sc) * vdc;           /* beta 分量，1/sqrt(3) 系数 */
}

/* 代价函数计算 */
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
                        float *cost_voltage)
{
    const float i_err_raw = squaref(r->i_alpha - x_pred->i_alpha)    /* alpha 电流误差平方 */
                          + squaref(r->i_beta - x_pred->i_beta);      /* beta 电流误差平方 */

    const float sw_count_raw = switch_activity * (float)(abs_int(sa - last_sa) /* A 相翻转 */
                             + abs_int(sb - last_sb)                            /* B 相翻转 */
                             + abs_int(sc - last_sc));                          /* C 相翻转 */

    const float v_err_raw = squaref(v_alpha - v_alpha_ref)            /* alpha 电压误差平方 */
                          + squaref(v_beta - v_beta_ref);              /* beta 电压误差平方 */

    const float j_current = weights->w_current * i_err_raw;            /* 电流项加权 */

    float switch_scale = 1.0f;                                         /* 默认无松弛 */
    if (weights->switch_relax_gain > 0.0f) {                           /* 如果启用自适应开关惩罚 */
        switch_scale = 1.0f / (1.0f + weights->switch_relax_gain * i_err_raw); /* 误差越大，惩罚越弱 */
    }
    const float j_switch = weights->w_switch * switch_scale * sw_count_raw; /* 开关项 */
    const float j_voltage = weights->w_voltage * v_err_raw;                 /* 电压项 */

    if (cost_current != 0) {           /* 调试输出指针有效 */
        *cost_current = j_current;     /* 写回电流代价分量 */
    }
    if (cost_switch != 0) {            /* 调试输出指针有效 */
        *cost_switch = j_switch;       /* 写回开关代价分量 */
    }
    if (cost_voltage != 0) {           /* 调试输出指针有效 */
        *cost_voltage = j_voltage;     /* 写回电压代价分量 */
    }

    return j_current + j_switch + j_voltage; /* 返回总代价 */
}

/* 控制器初始化 */
void mpc_controller_init(MpcController *ctrl,
                         const MotorParams *motor,
                         const MpcWeights *weights,
                         int init_sa, int init_sb, int init_sc)
{
    ctrl->motor = *motor;                 /* 保存电机参数 */
    ctrl->weights = *weights;             /* 保存权重参数 */
    ctrl->last_sa = init_sa;              /* 初始化上拍开关 A */
    ctrl->last_sb = init_sb;              /* 初始化上拍开关 B */
    ctrl->last_sc = init_sc;              /* 初始化上拍开关 C */

    motor_model_discrete_coeff(&ctrl->motor, &ctrl->a, &ctrl->b); /* 预计算离散系数 */

    /* 默认启用预筛选二步预测：
     * - Top-K=2：第一步保留 2 个候选；
     * - second_weight=1.0：第二步代价与第一步同权。 */
    ctrl->two_step_enable = 1;
    ctrl->two_step_top_k = 2;
    ctrl->two_step_weight = 1.0f;

    /* 默认关闭标幺化，保持与历史工程行为一致 */
    ctrl->pu_enable = 0;
    ctrl->i_base = 1.0f;
    ctrl->v_base = 1.0f;
}

/* 母线实时前馈 */
void mpc_controller_update_vdc(MpcController *ctrl, float vdc_meas)
{
    ctrl->motor.Vdc = vdc_meas;           /* 每拍更新 Vdc，匹配真实母线 */
}

void mpc_controller_set_two_step(MpcController *ctrl,
                                 int enable,
                                 int top_k,
                                 float second_weight)
{
    ctrl->two_step_enable = enable ? 1 : 0; /* 规范化使能值 */

    if (top_k < 1) top_k = 1;               /* Top-K 下限 */
    if (top_k > 8) top_k = 8;               /* Top-K 上限 */
    ctrl->two_step_top_k = top_k;           /* 写入 Top-K */

    if (second_weight < 0.0f) second_weight = 0.0f; /* 权重不允许负值 */
    ctrl->two_step_weight = second_weight;          /* 写入第二步权重 */
}

void mpc_controller_set_per_unit(MpcController *ctrl,
                                 int enable,
                                 float i_base,
                                 float v_base)
{
    ctrl->pu_enable = enable ? 1 : 0;      /* 写入标幺化开关 */
    if (i_base < 1.0e-6f) i_base = 1.0f;   /* 电流基值下限保护 */
    if (v_base < 1.0e-6f) v_base = 1.0f;   /* 电压基值下限保护 */
    ctrl->i_base = i_base;                 /* 写入电流基值 */
    ctrl->v_base = v_base;                 /* 写入电压基值 */
}

/* MPC 优化单步 */
MpcOutput mpc_controller_step(MpcController *ctrl,
                              const CurrentState *x,
                              const CurrentRef *r)
{
    static const int states[8][3] = {     /* 两电平三相逆变器 8 个候选状态 */
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 1, 1}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}
    };

    MpcOutput best;                        /* 最优候选缓存 */
    best.switch_a = ctrl->last_sa;         /* 默认继承上拍 */
    best.switch_b = ctrl->last_sb;         /* 默认继承上拍 */
    best.switch_c = ctrl->last_sc;         /* 默认继承上拍 */
    best.v_alpha = 0.0f;                   /* 默认电压 */
    best.v_beta = 0.0f;                    /* 默认电压 */
    best.duty_active = 0.0f;               /* 默认占空比 */
    best.v_alpha_active = 0.0f;            /* 默认有功矢量 */
    best.v_beta_active = 0.0f;             /* 默认有功矢量 */
    best.cost = 1.0e30f;                   /* 初值设极大，便于比较 */
    best.cost_current = 0.0f;              /* 初始化调试分量 */
    best.cost_switch = 0.0f;               /* 初始化调试分量 */
    best.cost_voltage = 0.0f;              /* 初始化调试分量 */
    best.v_alpha_ref = 0.0f;               /* 初始化参考电压 */
    best.v_beta_ref = 0.0f;                /* 初始化参考电压 */

    float v_alpha_ref = 0.0f;              /* 连续理想 alpha 电压 */
    float v_beta_ref = 0.0f;               /* 连续理想 beta 电压 */
    motor_model_deadbeat_voltage_ref(x, r, ctrl->a, ctrl->b, ctrl->motor.Vdc,
                                     &v_alpha_ref, &v_beta_ref); /* 每拍仅计算一次理想参考 */

    if (!ctrl->two_step_enable) {          /* 关闭二步预测时，退回一步枚举 */
        for (int i = 0; i < 8; ++i) {      /* 遍历所有候选开关状态 */
            MpcStageCandidate c;           /* 单候选缓存 */
            const int sa = states[i][0];   /* 当前候选 A 相状态 */
            const int sb = states[i][1];   /* 当前候选 B 相状态 */
            const int sc = states[i][2];   /* 当前候选 C 相状态 */

            mpc_evaluate_single_candidate(ctrl,
                                          x,
                                          r,
                                          v_alpha_ref,
                                          v_beta_ref,
                                          sa, sb, sc,
                                          ctrl->last_sa, ctrl->last_sb, ctrl->last_sc,
                                          &c);

            if (c.j_total < best.cost) {   /* 若当前候选更优 */
                best.switch_a = c.sa;
                best.switch_b = c.sb;
                best.switch_c = c.sc;
                best.v_alpha = c.v_alpha_eq;
                best.v_beta = c.v_beta_eq;
                best.duty_active = c.duty_active;
                best.v_alpha_active = c.v_alpha_active;
                best.v_beta_active = c.v_beta_active;
                best.cost = c.j_total;
                best.cost_current = c.j_current;
                best.cost_switch = c.j_switch;
                best.cost_voltage = c.j_voltage;
                best.v_alpha_ref = v_alpha_ref;
                best.v_beta_ref = v_beta_ref;
            }
        }
    } else {                               /* 启用预筛选二步预测 */
        MpcStageCandidate topk[8];         /* 第一阶段 Top-K 缓存（最多 8 个） */
        int top_k = ctrl->two_step_top_k;  /* 读取当前 Top-K 配置 */

        for (int i = 0; i < 8; ++i) {      /* 初始化 Top-K 槽位 */
            topk[i].valid = 0;
            topk[i].j_total = 1.0e30f;
        }

        for (int i = 0; i < 8; ++i) {      /* 第一阶段：一步评估并做 Top-K 预筛选 */
            MpcStageCandidate c;
            const int sa = states[i][0];
            const int sb = states[i][1];
            const int sc = states[i][2];

            mpc_evaluate_single_candidate(ctrl,
                                          x,
                                          r,
                                          v_alpha_ref,
                                          v_beta_ref,
                                          sa, sb, sc,
                                          ctrl->last_sa, ctrl->last_sb, ctrl->last_sc,
                                          &c);
            mpc_insert_topk(topk, top_k, &c); /* 只保留最优前 K 个 */
        }

        for (int i = 0; i < top_k; ++i) {  /* 第二阶段：对每个 Top-K 候选展开第二步 */
            float v_alpha_ref_2 = 0.0f;    /* 第二步理想 alpha 电压 */
            float v_beta_ref_2 = 0.0f;     /* 第二步理想 beta 电压 */
            float best_seq_cost = 1.0e30f; /* 当前第一步候选对应的最优序列代价 */

            if (!topk[i].valid) {          /* 空槽跳过 */
                continue;
            }

            motor_model_deadbeat_voltage_ref(&topk[i].x_pred, r, ctrl->a, ctrl->b, ctrl->motor.Vdc,
                                             &v_alpha_ref_2, &v_beta_ref_2); /* 基于 x(k+1) 求第二步参考 */

            for (int j = 0; j < 8; ++j) {  /* 第二步展开 8 个候选 */
                MpcStageCandidate c2;
                const int sa2 = states[j][0];
                const int sb2 = states[j][1];
                const int sc2 = states[j][2];
                float seq_cost = 0.0f;

                mpc_evaluate_single_candidate(ctrl,
                                              &topk[i].x_pred,
                                              r,
                                              v_alpha_ref_2,
                                              v_beta_ref_2,
                                              sa2, sb2, sc2,
                                              topk[i].sa, topk[i].sb, topk[i].sc,
                                              &c2);

                seq_cost = topk[i].j_total + ctrl->two_step_weight * c2.j_total; /* 两步序列总代价 */
                if (seq_cost < best_seq_cost) { /* 保留该第一步分支下最佳第二步 */
                    best_seq_cost = seq_cost;
                }
            }

            if (best_seq_cost < best.cost) { /* 跨分支比较，决定最终第一步动作 */
                best.switch_a = topk[i].sa;
                best.switch_b = topk[i].sb;
                best.switch_c = topk[i].sc;
                best.v_alpha = topk[i].v_alpha_eq;
                best.v_beta = topk[i].v_beta_eq;
                best.duty_active = topk[i].duty_active;
                best.v_alpha_active = topk[i].v_alpha_active;
                best.v_beta_active = topk[i].v_beta_active;
                best.cost = best_seq_cost;         /* 注意：这里是两步序列代价 */
                best.cost_current = topk[i].j_current;
                best.cost_switch = topk[i].j_switch;
                best.cost_voltage = topk[i].j_voltage;
                best.v_alpha_ref = v_alpha_ref;
                best.v_beta_ref = v_beta_ref;
            }
        }
    }

    ctrl->last_sa = best.switch_a; /* 保存本拍最优状态供下一拍开关惩罚使用 */
    ctrl->last_sb = best.switch_b;
    ctrl->last_sc = best.switch_c;

    return best;                   /* 输出最优结果 */
}
