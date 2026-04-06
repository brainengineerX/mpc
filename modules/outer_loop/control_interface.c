#include "control_interface.h"  /* 控制接口头文件 */

/* 局部限幅函数：保证变量在合法范围内 */
static float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;  /* 小于下限时返回下限 */
    if (x > max_v) return max_v;  /* 大于上限时返回上限 */
    return x;                     /* 范围内直接返回 */
}

/* 局部斜坡函数：限制参考值每拍变化量，降低命令突变 */
static float slew_step(float current, float target, float max_delta)
{
    const float delta = target - current;      /* 先算目标差值 */
    if (delta > max_delta) return current + max_delta;   /* 正向超限 */
    if (delta < -max_delta) return current - max_delta;  /* 反向超限 */
    return target;                                            /* 未超限直接到位 */
}

/* 模式切换无扰初始化：
 * 目标：切换时保持输出连续，避免“积分清零”造成的电流冲击。 */
static void control_interface_bumpless_on_mode_change(ControlInterface *ci,
                                                      ControlMode old_mode,
                                                      ControlMode new_mode,
                                                      const ControlFeedback *fb)
{
    const float spd_err = ci->omega_ref_int - fb->omega_mech; /* 当前速度误差 */
    /* 让速度环在切换后首拍输出接近当前 iq_ref_out */
    ci->spd_pi.integrator = ci->iq_ref_out - ci->spd_pi.kp * spd_err - fb->iq_feedforward;

    if (new_mode == CTRL_MODE_POSITION) { /* 位置模式还要对齐位置环积分 */
        const float e_pos_raw = ci->cmd.theta_ref_cmd - fb->theta_mech;
        const float e_pos = clampf(e_pos_raw, -ci->cfg.theta_err_max, ci->cfg.theta_err_max);
        if (old_mode == CTRL_MODE_SPEED || old_mode == CTRL_MODE_POSITION) {
            /* 从速度/位置模式切到位置模式：做无扰对齐，避免突变 */
            ci->omega_ref_int = fb->omega_mech;
            ci->pos_pi.integrator = ci->omega_ref_int - ci->pos_pi.kp * e_pos;
        } else {
            /* 从电流模式进入位置模式：不要抵消位置环比例项，否则会“推不动” */
            ci->omega_ref_int = 0.0f;
            ci->pos_pi.integrator = 0.0f;
        }
    }
}

/* 自动弱磁计算：
 * 1) 基于机械速度绝对值做进入/退出迟滞；
 * 2) 激活后按超速量线性注入负 id；
 * 3) 对注入量做一阶滤波，降低抖动。 */
static float control_interface_flux_weakening_step(ControlInterface *ci,
                                                   const ControlFeedback *fb)
{
    float id_fw_target = 0.0f;                            /* 目标弱磁注入电流 */
    const float omega_abs = (fb->omega_mech >= 0.0f) ? fb->omega_mech : -fb->omega_mech; /* 速度绝对值 */

    if (!ci->cfg.fw_enable_auto) {                        /* 自动弱磁关闭时直接退场 */
        ci->fw_active = 0;
        ci->id_fw_state = 0.0f;
        return 0.0f;
    }

    if (!ci->fw_active) {                                 /* 当前未激活：判断是否进入 */
        if (omega_abs >= ci->cfg.omega_fw_enter) {
            ci->fw_active = 1;
        }
    } else {                                              /* 当前已激活：判断是否退出 */
        if (omega_abs <= ci->cfg.omega_fw_exit) {
            ci->fw_active = 0;
        }
    }

    if (ci->fw_active) {                                  /* 激活时按超速量注入负 id */
        const float over_speed = omega_abs - ci->cfg.omega_fw_enter;
        id_fw_target = -ci->cfg.id_fw_gain * over_speed;
        id_fw_target = clampf(id_fw_target, ci->cfg.id_fw_min, 0.0f);
    } else {                                              /* 未激活时回零 */
        id_fw_target = 0.0f;
    }

    ci->id_fw_state += ci->cfg.id_fw_lpf * (id_fw_target - ci->id_fw_state); /* 一阶滤波 */
    return ci->id_fw_state;                               /* 返回当前弱磁注入值 */
}

void control_interface_init(ControlInterface *ci,
                            const ControlInterfaceConfig *cfg)
{
    ci->cfg = *cfg;                                  /* 保存配置副本，后续统一引用 */

    ci->cmd.mode = CTRL_MODE_CURRENT;                /* 默认进入电流模式，最保守 */
    ci->cmd.id_ref_cmd = 0.0f;                       /* 默认 d轴目标为 0 */
    ci->cmd.iq_ref_cmd = 0.0f;                       /* 默认 q轴目标为 0 */
    ci->cmd.omega_ref_cmd = 0.0f;                    /* 默认速度目标为 0 */
    ci->cmd.theta_ref_cmd = 0.0f;                    /* 默认位置目标为 0 */

    /* 初始化位置环 PI：输出是 omega_ref，所以上下限取速度限幅 */
    cascade_pi_init(&ci->pos_pi,
                    cfg->pos_kp,
                    cfg->pos_ki,
                    cfg->pos_kaw,
                    -cfg->omega_ref_max,
                    cfg->omega_ref_max);

    /* 初始化速度环 PI：输出是 iq_ref，所以上下限取电流限幅 */
    cascade_pi_init(&ci->spd_pi,
                    cfg->spd_kp,
                    cfg->spd_ki,
                    cfg->spd_kaw,
                    -cfg->iq_ref_max,
                    cfg->iq_ref_max);

    ci->omega_ref_int = 0.0f;                        /* 内部速度参考清零 */
    ci->iq_ref_int = 0.0f;                           /* 内部 iq 参考清零 */
    ci->id_ref_out = 0.0f;                           /* 对外 d轴输出清零 */
    ci->iq_ref_out = 0.0f;                           /* 对外 q轴输出清零 */
    ci->id_fw_state = 0.0f;                          /* 弱磁注入状态清零 */
    ci->fw_active = 0;                               /* 弱磁激活标志清零 */
    ci->tick = 0;                                    /* 节拍计数器清零 */
    ci->last_mode = ci->cmd.mode;                    /* 上一拍模式初始化 */
}

void control_interface_set_mode(ControlInterface *ci, ControlMode mode)
{
    ci->cmd.mode = mode;  /* 只写模式，具体无扰处理在 step 中完成 */
}

void control_interface_set_current_target(ControlInterface *ci,
                                          float id_ref,
                                          float iq_ref)
{
    ci->cmd.mode = CTRL_MODE_CURRENT;                           /* 切换到电流模式 */
    ci->cmd.id_ref_cmd = id_ref;                                /* 写入 d轴目标 */
    ci->cmd.iq_ref_cmd = clampf(iq_ref,                         /* 写入并限幅 q轴目标 */
                                -ci->cfg.iq_ref_max,
                                ci->cfg.iq_ref_max);
}

void control_interface_set_speed_target(ControlInterface *ci,
                                        float omega_ref)
{
    ci->cmd.mode = CTRL_MODE_SPEED;                             /* 切换到速度模式 */
    ci->cmd.omega_ref_cmd = clampf(omega_ref,                   /* 写入并限幅速度目标 */
                                   -ci->cfg.omega_ref_max,
                                   ci->cfg.omega_ref_max);
}

void control_interface_set_position_target(ControlInterface *ci,
                                           float theta_ref)
{
    ci->cmd.mode = CTRL_MODE_POSITION;                          /* 切换到位置模式 */
    ci->cmd.theta_ref_cmd = theta_ref;                          /* 写入多圈位置目标 */
}

void control_interface_step(ControlInterface *ci,
                            const ControlFeedback *fb,
                            ControlRefOutput *out)
{
    float id_target = 0.0f;                                     /* 本拍 d轴目标 */
    float iq_target = 0.0f;                                     /* 本拍 q轴目标 */
    float id_fw = 0.0f;                                         /* 本拍弱磁注入量 */
    float theta_err_dbg = 0.0f;                                 /* 调试：位置误差 */
    float omega_err_dbg = 0.0f;                                 /* 调试：速度误差 */

    /* 保护分频参数：防止外部配置成 0 导致除零或失效 */
    const int spd_div = (ci->cfg.speed_loop_div > 0) ? ci->cfg.speed_loop_div : 1;
    const int pos_div = (ci->cfg.pos_loop_div > 0) ? ci->cfg.pos_loop_div : 1;

    if (ci->cmd.mode != ci->last_mode) { /* 检测到模式切换 */
        control_interface_bumpless_on_mode_change(ci, ci->last_mode, ci->cmd.mode, fb); /* 做积分对齐 */
        ci->last_mode = ci->cmd.mode;   /* 记录新模式 */
    }

    if (ci->cmd.mode == CTRL_MODE_CURRENT) {
        /* 电流模式：直接透传命令，不经过位置/速度 PI */
        id_target = ci->cmd.id_ref_cmd;
        iq_target = ci->cmd.iq_ref_cmd;
        ci->omega_ref_int = ci->cmd.omega_ref_cmd;
    } else if (ci->cmd.mode == CTRL_MODE_SPEED) {
        /* 速度模式：速度参考直接来自命令 */
        ci->omega_ref_int = clampf(ci->cmd.omega_ref_cmd,
                                   -ci->cfg.omega_ref_max,
                                   ci->cfg.omega_ref_max);

        /* 按分频执行速度 PI，输出 iq_ref */
        if ((ci->tick % spd_div) == 0) {
            const float e_spd = ci->omega_ref_int - fb->omega_mech;
            omega_err_dbg = e_spd;
            ci->iq_ref_int = cascade_pi_step(&ci->spd_pi,
                                             e_spd,
                                             fb->iq_feedforward,
                                             fb->Ts * (float)spd_div);
        }

        id_target = ci->cmd.id_ref_cmd;
        iq_target = ci->iq_ref_int;
    } else {
        /* 位置模式：先位置 PI 算 omega_ref，再速度 PI 算 iq_ref */
        if ((ci->tick % pos_div) == 0) {
            const float e_pos_raw = ci->cmd.theta_ref_cmd - fb->theta_mech;
            const float e_pos = clampf(e_pos_raw,
                                       -ci->cfg.theta_err_max,
                                       ci->cfg.theta_err_max);
            theta_err_dbg = e_pos;
            ci->omega_ref_int = cascade_pi_step(&ci->pos_pi,
                                                e_pos,
                                                0.0f,
                                                fb->Ts * (float)pos_div);
        }

        if ((ci->tick % spd_div) == 0) {
            const float e_spd = ci->omega_ref_int - fb->omega_mech;
            omega_err_dbg = e_spd;
            ci->iq_ref_int = cascade_pi_step(&ci->spd_pi,
                                             e_spd,
                                             fb->iq_feedforward,
                                             fb->Ts * (float)spd_div);
        }

        id_target = ci->cmd.id_ref_cmd;
        iq_target = ci->iq_ref_int;
    }

    /* 自动弱磁：只在速度模式/位置模式生效，电流模式保持命令直通 */
    if (ci->cmd.mode != CTRL_MODE_CURRENT) {
        id_fw = control_interface_flux_weakening_step(ci, fb);
        id_target += id_fw;                                     /* 把弱磁注入叠加到 d轴目标 */
    } else {
        ci->fw_active = 0;                                      /* 电流模式下不启用自动弱磁 */
        ci->id_fw_state = 0.0f;
    }

    /* 最终输出都经过斜坡，降低参考突变 */
    ci->id_ref_out = slew_step(ci->id_ref_out, id_target, ci->cfg.id_slew_step);
    ci->iq_ref_out = slew_step(ci->iq_ref_out, iq_target, ci->cfg.iq_slew_step);

    /* 再做一次硬限幅，防止任何路径溢出 */
    ci->id_ref_out = clampf(ci->id_ref_out, -ci->cfg.iq_ref_max, ci->cfg.iq_ref_max);
    ci->iq_ref_out = clampf(ci->iq_ref_out, -ci->cfg.iq_ref_max, ci->cfg.iq_ref_max);

    /* 打包输出给调用层 */
    out->id_ref = ci->id_ref_out;
    out->iq_ref = ci->iq_ref_out;
    out->omega_ref_dbg = ci->omega_ref_int;
    out->mode_dbg = ci->cmd.mode;
    out->id_fw_dbg = ci->id_fw_state;
    out->fw_active_dbg = ci->fw_active;
    out->theta_err_dbg = theta_err_dbg;
    out->omega_err_dbg = omega_err_dbg;

    ci->tick++;  /* 节拍推进 */
}
