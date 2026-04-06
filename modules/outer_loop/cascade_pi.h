#ifndef CASCADE_PI_H
#define CASCADE_PI_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：cascade_pi
 * 作用：提供“位置环 + 速度环”可复用 PI 组件。
 *
 * 设计目标：
 * 1) 与电流环 MPC 解耦，便于单独调参与替换。
 * 2) 提供统一抗积分饱和逻辑，避免积分项失控。
 * 3) 代码尽量简单直接，适合嵌入式实时控制。
 * ========================================================================== */

/* 通用 PI 控制器数据结构 */
typedef struct {
    float kp;              /* 比例增益 */
    float ki;              /* 积分增益 */
    float kaw;             /* 抗积分饱和反算增益 */
    float integrator;      /* 积分状态 */
    float out_min;         /* 输出下限 */
    float out_max;         /* 输出上限 */
} CascadePiController;

/* 位置/速度级联控制上下文 */
typedef struct {
    CascadePiController pos_pi;     /* 位置环 PI（也可设 Ki=0 退化成 P） */
    CascadePiController spd_pi;     /* 速度环 PI */
    float theta_ref_mech;           /* 机械角位置参考（rad） */
    float omega_ref;                /* 机械角速度参考（rad/s） */
    float iq_ref;                   /* q轴电流参考（A） */
    float iq_ff;                    /* q轴前馈电流（A） */
    float id_ref;                   /* d轴电流参考（A），常用 0 */
} CascadeLoopState;

/* 常用数学工具：限幅 */
float cascade_clampf(float x, float min_v, float max_v);

/* 常用数学工具：角度包裹到 [-pi, pi] */
float cascade_wrap_pm_pi(float x);

/* PI 初始化：一次性写入参数与输出限幅 */
void cascade_pi_init(CascadePiController *pi,
                     float kp,
                     float ki,
                     float kaw,
                     float out_min,
                     float out_max);

/* PI 状态清零：上电、切模式、故障恢复时调用 */
void cascade_pi_reset(CascadePiController *pi, float integrator_init);

/* PI 单步更新：
 * err：当前误差
 * ff：前馈量（直接加在控制输出中）
 * Ts：当前控制步长（秒） */
float cascade_pi_step(CascadePiController *pi,
                      float err,
                      float ff,
                      float Ts);

#ifdef __cplusplus
}
#endif

#endif /* CASCADE_PI_H */
