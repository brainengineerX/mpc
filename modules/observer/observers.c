#include "observers.h"

/* ----------------------------------------------------------------------------
 * 实现说明：
 * - DisturbanceObserver：估计电流方程未建模项，输出 d_hat 做电压前馈补偿；
 * - LoadTorqueObserver：利用机械平衡方程估计负载转矩；
 * - rs_adaptation_update：用电压方程残差调整Rs；
 * - SpeedPosEstimator：低通滤波速度并连续积分位置角。
 * -------------------------------------------------------------------------- */

static float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void disturbance_observer_init(DisturbanceObserver *obs, float a, float b, float l_i, float l_d)
{
    obs->a = a;
    obs->b = b;
    obs->l_i = l_i;
    obs->l_d = l_d;
    obs->i_hat.i_alpha = 0.0f;
    obs->i_hat.i_beta = 0.0f;
    obs->d_hat.x = 0.0f;
    obs->d_hat.y = 0.0f;
}

void disturbance_observer_update(DisturbanceObserver *obs,
                                 const CurrentState *i_meas,
                                 const Vec2 *v_applied_ab)
{
    /* 观测误差注入：e = i_meas - i_hat */
    const float e_alpha = i_meas->i_alpha - obs->i_hat.i_alpha;
    const float e_beta = i_meas->i_beta - obs->i_hat.i_beta;

    obs->i_hat.i_alpha = obs->a * obs->i_hat.i_alpha + obs->b * (v_applied_ab->x + obs->d_hat.x) + obs->l_i * e_alpha;
    obs->i_hat.i_beta = obs->a * obs->i_hat.i_beta + obs->b * (v_applied_ab->y + obs->d_hat.y) + obs->l_i * e_beta;

    obs->d_hat.x += obs->l_d * e_alpha;
    obs->d_hat.y += obs->l_d * e_beta;
}

void load_torque_observer_init(LoadTorqueObserver *obs, float J, float B, float lpf_gain)
{
    obs->J = J;
    obs->B = B;
    obs->lpf_gain = lpf_gain;
    obs->tl_hat = 0.0f;
    obs->omega_prev = 0.0f;
    obs->inited = 0;
}

float load_torque_observer_update(LoadTorqueObserver *obs,
                                  float omega_m,
                                  float torque_e,
                                  float Ts)
{
    float domega = 0.0f;
    if (obs->inited) {
        domega = (omega_m - obs->omega_prev) / Ts;
    } else {
        obs->inited = 1;
    }
    obs->omega_prev = omega_m;

    {
        /* Tl = Te - J*domega - B*omega，再做低通过滤 */
        const float tl_inst = torque_e - obs->J * domega - obs->B * omega_m;
        obs->tl_hat += obs->lpf_gain * (tl_inst - obs->tl_hat);
    }
    return obs->tl_hat;
}

float rs_adaptation_update(float rs_prev,
                           const RsAdaptiveConfig *cfg,
                           float vd, float vq,
                           float id, float iq,
                           float did, float diq,
                           float omega_e,
                           float Ld, float Lq,
                           float psi_f)
{
    /* 利用d/q轴电压模型残差构造梯度，更新Rs */
    const float v_d_model = rs_prev * id + Ld * did - omega_e * Lq * iq;
    const float v_q_model = rs_prev * iq + Lq * diq + omega_e * (Ld * id + psi_f);

    const float err_d = vd - v_d_model;
    const float err_q = vq - v_q_model;

    const float denom = id * id + iq * iq + 1.0e-4f;
    const float grad = (err_d * id + err_q * iq) / denom;

    return clampf(rs_prev + cfg->gamma * grad, cfg->rs_min, cfg->rs_max);
}

void speed_pos_estimator_init(SpeedPosEstimator *est, float alpha)
{
    est->alpha = alpha;
    est->omega_filt = 0.0f;
    est->theta_interp = 0.0f;
    est->inited = 0;
}

void speed_pos_estimator_update(SpeedPosEstimator *est,
                                float omega_meas,
                                float Ts,
                                float pole_pairs)
{
    if (!est->inited) {
        est->omega_filt = omega_meas;
        est->inited = 1;
    }

    est->omega_filt += est->alpha * (omega_meas - est->omega_filt);
    est->theta_interp += Ts * pole_pairs * est->omega_filt;

    if (est->theta_interp > 3.14159265359f) est->theta_interp -= 6.28318530718f;
    if (est->theta_interp < -3.14159265359f) est->theta_interp += 6.28318530718f;
}
