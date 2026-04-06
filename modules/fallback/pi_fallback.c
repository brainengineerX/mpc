#include "pi_fallback.h"

/* ----------------------------------------------------------------------------
 * PI后备控制实现：
 * - 带积分限幅，避免长时间饱和导致积分暴涨；
 * - 输出限幅，避免给逆变器不合理指令。
 * -------------------------------------------------------------------------- */

static float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float pi_step(PIReg *r, float err, float Ts)
{
    /* 标准PI离散实现：u = kp*e + integral(ki*e) */
    r->integ += r->ki * Ts * err;
    r->integ = clampf(r->integ, -r->out_limit, r->out_limit);
    return clampf(r->kp * err + r->integ, -r->out_limit, r->out_limit);
}

void pi_current_controller_init(PiCurrentController *c,
                                float kp,
                                float ki,
                                float out_limit)
{
    c->alpha.kp = kp;
    c->alpha.ki = ki;
    c->alpha.integ = 0.0f;
    c->alpha.out_limit = out_limit;

    c->beta.kp = kp;
    c->beta.ki = ki;
    c->beta.integ = 0.0f;
    c->beta.out_limit = out_limit;
}

Vec2 pi_current_step(PiCurrentController *c,
                     const CurrentRef *r,
                     const CurrentState *x,
                     float Ts)
{
    Vec2 out;
    out.x = pi_step(&c->alpha, r->i_alpha - x->i_alpha, Ts);
    out.y = pi_step(&c->beta, r->i_beta - x->i_beta, Ts);
    return out;
}
