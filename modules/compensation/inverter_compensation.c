#include <math.h>
#include "inverter_compensation.h"

/* ----------------------------------------------------------------------------
 * 实现说明：
 * 1) 先根据 alpha-beta 电流反推三相电流；
 * 2) 计算每相补偿电压（死区 + 压降）；
 * 3) 再变换回 alpha-beta 并叠加到命令电压。
 * -------------------------------------------------------------------------- */

static float sign_with_eps(float x, float eps)
{
    if (x > eps) {
        return 1.0f;
    }
    if (x < -eps) {
        return -1.0f;
    }
    return 0.0f;
}

Vec2 inverter_compensate_voltage_ab(const Vec2 *v_cmd_ab,
                                    const CurrentState *i_ab,
                                    float vdc_meas,
                                    const InverterCompConfig *cfg)
{
    Vec2 out = *v_cmd_ab;

    /* alpha-beta -> 三相电流（假设三相平衡、零序电流为0） */
    const float ia = i_ab->i_alpha;
    const float ib = -0.5f * i_ab->i_alpha + 0.86602540378f * i_ab->i_beta;
    const float ic = -0.5f * i_ab->i_alpha - 0.86602540378f * i_ab->i_beta;

    /* 每相补偿电压：死区项 + 器件压降项
     * 死区项幅值约等于 (deadtime / pwm_period) * Vdc。 */
    const float v_dt = (cfg->deadtime_s / cfg->pwm_period_s) * vdc_meas;
    const float va_comp = sign_with_eps(ia, cfg->current_epsilon) * (v_dt + cfg->v_diode + cfg->r_on * fabsf(ia));
    const float vb_comp = sign_with_eps(ib, cfg->current_epsilon) * (v_dt + cfg->v_diode + cfg->r_on * fabsf(ib));
    const float vc_comp = sign_with_eps(ic, cfg->current_epsilon) * (v_dt + cfg->v_diode + cfg->r_on * fabsf(ic));

    /* 三相补偿 -> alpha-beta，再叠加到命令电压 */
    const float v_alpha_comp = (2.0f / 3.0f) * (va_comp - 0.5f * vb_comp - 0.5f * vc_comp);
    const float v_beta_comp = (2.0f / 3.0f) * (0.86602540378f * (vb_comp - vc_comp));

    out.x += v_alpha_comp;
    out.y += v_beta_comp;
    return out;
}
