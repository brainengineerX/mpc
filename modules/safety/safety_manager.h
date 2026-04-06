#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：safety_manager
 * 职责：统一管理保护与降级策略。
 *
 * 设计原则：
 * 1) 保护优先于性能；
 * 2) 可恢复故障优先进入降级模式（DEGRADED_PI）；
 * 3) 严重故障直接进入FAULT并切断输出。
 * ========================================================================== */

typedef enum {
    SAFETY_MODE_NORMAL = 0,
    SAFETY_MODE_DEGRADED_PI = 1,
    SAFETY_MODE_FAULT = 2
} SafetyMode;

typedef enum {
    SAFETY_CAUSE_NONE = 0,
    SAFETY_CAUSE_OVERCURRENT,
    SAFETY_CAUSE_OVERVOLTAGE,
    SAFETY_CAUSE_STALL,
    SAFETY_CAUSE_MPC_DIVERGED
} SafetyCause;

typedef struct {
    /* 电流保护阈值 */
    float i_limit;
    /* 母线过压阈值 */
    float vdc_limit;
    /* 失速判定速度下限 */
    float stall_speed_min;
    /* 失速计数门限（抗瞬态误报） */
    int stall_count_limit;
    /* MPC代价异常阈值 */
    float cost_limit;
} SafetyConfig;

typedef struct {
    SafetyMode mode;
    SafetyCause cause;
    int stall_counter;
} SafetyState;

void safety_init(SafetyState *s);
void safety_update(SafetyState *s,
                   const SafetyConfig *cfg,
                   float i_alpha,
                   float i_beta,
                   float vdc,
                   float omega_m,
                   float mpc_cost);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MANAGER_H */
