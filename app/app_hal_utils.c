#include "app_hal_utils.h"

#define APP_TWO_PI 6.28318530718f

float app_clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

float app_wrap_pm_pi(float x)
{
    while (x > 3.14159265359f) x -= APP_TWO_PI;
    while (x < -3.14159265359f) x += APP_TWO_PI;
    return x;
}

float app_slew_step(float current, float target, float max_delta)
{
    const float delta = target - current;
    if (delta > max_delta) return current + max_delta;
    if (delta < -max_delta) return current - max_delta;
    return target;
}

void app_encoder_estimator_update(EncoderStateEst *e,
                                  int32_t count,
                                  int ppr,
                                  float pole_pairs,
                                  float Ts)
{
    /* 每个编码器计数对应的机械角度增量（AB 四倍频） */
    const float mech_rad_per_count = APP_TWO_PI / (4.0f * (float)ppr);
    /* 机械角 -> 电角：theta_e = pole_pairs * theta_m */
    const float theta_m = (float)count * mech_rad_per_count;
    e->theta_e = app_wrap_pm_pi(theta_m * pole_pairs);

    /* 第一次进入时只做初始化，不计算速度差分 */
    if (!e->inited) {
        e->omega_m = 0.0f;
        e->prev_count = count;
        e->inited = 1;
        return;
    }

    {
        /* 机械角速度离散估计：omega = delta_theta / Ts */
        const int32_t dcount = count - e->prev_count;
        e->omega_m = ((float)dcount * mech_rad_per_count) / Ts;
        e->prev_count = count;
    }
}

float app_encoder_count_to_mech_rad(int32_t count, int ppr)
{
    /* 机械角每个计数对应增量：2*pi / (AB四倍频总计数) */
    const float mech_rad_per_count = APP_TWO_PI / (4.0f * (float)ppr);
    /* 机械角采用“多圈累计角”，不做 [-pi, pi] 包裹，便于位置环闭环 */
    return (float)count * mech_rad_per_count;
}

Vec2 app_phase_to_alphabeta(float vu, float vv, float vw)
{
    Vec2 out;
    out.x = (2.0f / 3.0f) * (vu - 0.5f * vv - 0.5f * vw);
    out.y = (2.0f / 3.0f) * (0.86602540378f * (vv - vw));
    return out;
}

void app_build_pwm_cmd_for_motor(HalPwmCmd *cmd,
                                 int motor_idx,
                                 float v_alpha,
                                 float v_beta,
                                 float vdc,
                                 int enable)
{
    /* 先把双电机默认都置为中点占空比并禁能，避免脏数据带入 */
    for (int m = 0; m < HAL_MOTOR_COUNT; ++m) {
        cmd->motor[m].duty_u = 0.5f;
        cmd->motor[m].duty_v = 0.5f;
        cmd->motor[m].duty_w = 0.5f;
        cmd->motor[m].enable = 0;
    }

    {
        /* alpha-beta -> 三相占空比（简化线性映射，适合仿真/演示） */
        const float du = app_clampf(0.5f + v_alpha / (vdc + 1.0e-6f), 0.0f, 1.0f);
        const float dv = app_clampf(0.5f + (-0.5f * v_alpha + 0.86602540378f * v_beta) / (vdc + 1.0e-6f), 0.0f, 1.0f);
        const float dw = app_clampf(0.5f + (-0.5f * v_alpha - 0.86602540378f * v_beta) / (vdc + 1.0e-6f), 0.0f, 1.0f);

        cmd->motor[motor_idx].duty_u = du;
        cmd->motor[motor_idx].duty_v = dv;
        cmd->motor[motor_idx].duty_w = dw;
        cmd->motor[motor_idx].enable = enable ? 1u : 0u;
    }
}
