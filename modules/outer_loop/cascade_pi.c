#include "cascade_pi.h" /* 对应头文件 */

#define CASCADE_TWO_PI 6.28318530718f /* 2*pi 常量 */
#define CASCADE_PI     3.14159265359f /* pi 常量 */

float cascade_clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;           /* 小于下限时钳到下限 */
    if (x > max_v) return max_v;           /* 大于上限时钳到上限 */
    return x;                              /* 其余情况原值返回 */
}

float cascade_wrap_pm_pi(float x)
{
    while (x > CASCADE_PI) x -= CASCADE_TWO_PI;   /* 大于 +pi 就减 2pi */
    while (x < -CASCADE_PI) x += CASCADE_TWO_PI;  /* 小于 -pi 就加 2pi */
    return x;                                     /* 返回包裹后的角度 */
}

void cascade_pi_init(CascadePiController *pi,
                     float kp,
                     float ki,
                     float kaw,
                     float out_min,
                     float out_max)
{
    pi->kp = kp;                     /* 写入比例增益 */
    pi->ki = ki;                     /* 写入积分增益 */
    pi->kaw = kaw;                   /* 写入抗饱和反算增益 */
    pi->integrator = 0.0f;           /* 积分状态清零 */
    pi->out_min = out_min;           /* 写入输出下限 */
    pi->out_max = out_max;           /* 写入输出上限 */
}

void cascade_pi_reset(CascadePiController *pi, float integrator_init)
{
    pi->integrator = integrator_init; /* 外部可指定积分初值 */
}

float cascade_pi_step(CascadePiController *pi,
                      float err,
                      float ff,
                      float Ts)
{
    const float p_term = pi->kp * err;                /* 计算比例项 */
    const float u_unsat = p_term + pi->integrator + ff; /* 饱和前总输出 */
    const float u_sat = cascade_clampf(u_unsat, pi->out_min, pi->out_max); /* 饱和后输出 */

    const float aw_err = u_sat - u_unsat;            /* 反算误差（饱和差） */
    pi->integrator += pi->ki * Ts * err + pi->kaw * aw_err; /* 更新积分并做抗饱和 */

    return u_sat;                                    /* 返回最终输出 */
}
