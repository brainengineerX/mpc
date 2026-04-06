#include <math.h>
#include "safety_manager.h"

/* ----------------------------------------------------------------------------
 * 策略说明：
 * - OVERCURRENT / OVERVOLTAGE：直接FAULT；
 * - STALL / MPC_DIVERGED：先降级到PI，避免直接停机；
 * - 正常后可恢复NORMAL。
 * -------------------------------------------------------------------------- */

void safety_init(SafetyState *s)
{
    s->mode = SAFETY_MODE_NORMAL;
    s->cause = SAFETY_CAUSE_NONE;
    s->stall_counter = 0;
}

void safety_update(SafetyState *s,
                   const SafetyConfig *cfg,
                   float i_alpha,
                   float i_beta,
                   float vdc,
                   float omega_m,
                   float mpc_cost)
{
    const float i_mag = sqrtf(i_alpha * i_alpha + i_beta * i_beta);

    /* 一级硬保护：电流/电压超限立即故障 */
    if (i_mag > cfg->i_limit) {
        s->mode = SAFETY_MODE_FAULT;
        s->cause = SAFETY_CAUSE_OVERCURRENT;
        return;
    }
    if (vdc > cfg->vdc_limit) {
        s->mode = SAFETY_MODE_FAULT;
        s->cause = SAFETY_CAUSE_OVERVOLTAGE;
        return;
    }

    /* 失速判定：低速 + 高电流持续一段时间 */
    if (fabsf(omega_m) < cfg->stall_speed_min && i_mag > 0.6f * cfg->i_limit) {
        s->stall_counter += 1;
    } else {
        if (s->stall_counter > 0) s->stall_counter -= 1;
    }

    if (s->stall_counter > cfg->stall_count_limit) {
        s->mode = SAFETY_MODE_DEGRADED_PI;
        s->cause = SAFETY_CAUSE_STALL;
        return;
    }

    /* 二级保护：MPC行为异常，降级PI */
    if (mpc_cost > cfg->cost_limit) {
        s->mode = SAFETY_MODE_DEGRADED_PI;
        s->cause = SAFETY_CAUSE_MPC_DIVERGED;
        return;
    }

    s->mode = SAFETY_MODE_NORMAL;
    s->cause = SAFETY_CAUSE_NONE;
}
