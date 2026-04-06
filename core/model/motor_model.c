#include <math.h>
#include "motor_model.h"

/* ----------------------------------------------------------------------------
 * 说明：
 * 本文件实现了“完整模型 + 简化模型”两套方程。
 * - 完整模型用于真实对象、辨识、控制器验证；
 * - 简化模型用于电流环MPC实时预测（计算量更小）。
 *
 * 注意：
 * 真实固件中，完整模型函数一般不会在实时中断内全量运行，
 * 更多用于离线仿真、HIL、调参与辨识阶段。
 * -------------------------------------------------------------------------- */

static float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) {
        return min_v;
    }
    if (x > max_v) {
        return max_v;
    }
    return x;
}

Vec2 dq_to_alphabeta(float d, float q, float theta_e)
{
    Vec2 out;
    /* 逆Park变换矩阵：
     * [alpha]   [ cos -sin][d]
     * [ beta] = [ sin  cos][q] */
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);
    out.x = c * d - s * q;
    out.y = s * d + c * q;
    return out;
}

Vec2 alphabeta_to_dq(float alpha, float beta, float theta_e)
{
    Vec2 out;
    /* Park变换矩阵：
     * [d]   [ cos  sin][alpha]
     * [q] = [-sin  cos][ beta] */
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);
    out.x = c * alpha + s * beta;
    out.y = -s * alpha + c * beta;
    return out;
}

float pmsm_electrical_speed(const PmsmParams *p, const PmsmState *x)
{
    return p->pole_pairs * x->omega_m;
}

float pmsm_electromagnetic_torque(const PmsmParams *p, const PmsmState *x)
{
    return 1.5f * p->pole_pairs * (p->psi_f * x->iq + (p->Ld - p->Lq) * x->id * x->iq);
}

PmsmState pmsm_step_dq(const PmsmParams *p,
                       const PmsmState *x,
                       float vd, float vq,
                       float load_torque)
{
    PmsmState nx = *x;

    const float we = pmsm_electrical_speed(p, x);
    const float te = pmsm_electromagnetic_torque(p, x);

    /* 完整 dq 电流动态（电压方程离散化前的连续形式） */
    const float did = (vd - p->Rs * x->id + we * p->Lq * x->iq) / p->Ld;
    const float diq = (vq - p->Rs * x->iq - we * (p->Ld * x->id + p->psi_f)) / p->Lq;

    /* 机械动态（转矩平衡方程） */
    const float domega_m = (te - load_torque - p->B * x->omega_m) / p->J;

    /* 前向欧拉离散：x(k+1)=x(k)+Ts*dx/dt */
    nx.id = x->id + p->Ts * did;
    nx.iq = x->iq + p->Ts * diq;
    nx.omega_m = x->omega_m + p->Ts * domega_m;
    nx.theta_e = x->theta_e + p->Ts * we;

    /* 角度归一化到 [-pi, pi]，便于数值稳定 */
    if (nx.theta_e > 3.14159265359f) {
        nx.theta_e -= 6.28318530718f;
    } else if (nx.theta_e < -3.14159265359f) {
        nx.theta_e += 6.28318530718f;
    }

    return nx;
}

void motor_model_discrete_coeff(const MotorParams *motor, float *a, float *b)
{
    /* 简化RL离散模型：
     * di/dt = (v - Rs*i)/Ls
     * i(k+1)= (1 - Ts*Rs/Ls)*i(k) + (Ts/Ls)*v(k) */
    *a = 1.0f - (motor->Ts * motor->Rs / motor->Ls);
    *b = motor->Ts / motor->Ls;
}

CurrentState motor_model_predict_next(const CurrentState *x,
                                      float v_alpha, float v_beta,
                                      float a, float b)
{
    CurrentState x_next;
    x_next.i_alpha = a * x->i_alpha + b * v_alpha;
    x_next.i_beta = a * x->i_beta + b * v_beta;
    return x_next;
}

void motor_model_deadbeat_voltage_ref(const CurrentState *x,
                                      const CurrentRef *r,
                                      float a, float b,
                                      float vdc,
                                      float *v_alpha_ref,
                                      float *v_beta_ref)
{
    float va_ref = 0.0f;
    float vb_ref = 0.0f;

    /* deadbeat思想：让下一拍尽量直接到参考值，反解得到理想电压。 */
    if (b > 1.0e-12f) {
        va_ref = (r->i_alpha - a * x->i_alpha) / b;
        vb_ref = (r->i_beta - a * x->i_beta) / b;
    }

    {
        /* 逆变器能力边界（两电平SVPWM常用近似边界） */
        const float v_alpha_lim = (2.0f / 3.0f) * vdc;
        const float v_beta_lim = 0.57735026919f * vdc;
        *v_alpha_ref = clampf(va_ref, -v_alpha_lim, v_alpha_lim);
        *v_beta_ref = clampf(vb_ref, -v_beta_lim, v_beta_lim);
    }
}

CurrentState motor_model_plant_step(const CurrentState *x,
                                    float v_alpha, float v_beta,
                                    float disturbance_alpha,
                                    float disturbance_beta,
                                    float a, float b)
{
    CurrentState x_next;
    x_next.i_alpha = a * x->i_alpha + b * (v_alpha + disturbance_alpha);
    x_next.i_beta = a * x->i_beta + b * (v_beta + disturbance_beta);
    return x_next;
}
