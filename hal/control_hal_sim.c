#include <math.h>            /* sinf/cosf/fabsf */
#include "control_hal_sim.h" /* 仿真HAL头文件 */

#define TWO_PI 6.28318530718f /* 2*pi 常量 */

/* 三相相电压 -> alpha-beta 电压（Clarke） */
static Vec2 phase_to_alphabeta(float vu, float vv, float vw)
{
    Vec2 out;
    out.x = (2.0f / 3.0f) * (vu - 0.5f * vv - 0.5f * vw);
    out.y = (2.0f / 3.0f) * (0.86602540378f * (vv - vw));
    return out;
}

/* 占空比限幅 */
static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

/* 从连续机械角估算 AB 编码器累计计数（四倍频） */
static int32_t theta_m_to_enc_count(float theta_m_acc, int32_t ppr)
{
    const float rev = theta_m_acc / TWO_PI;
    return (int32_t)(rev * (float)(4 * ppr)); /* AB四倍频计数 */
}

/* 读反馈：把当前仿真状态转换为 HAL 反馈格式 */
static int sim_read_feedback(void *ctx, HalFeedback *fb)
{
    ControlHalSimCtx *sim = (ControlHalSimCtx *)ctx;

    fb->vdc = sim->vdc;
    fb->time_s = sim->time_s;

    for (int m = 0; m < HAL_MOTOR_COUNT; ++m) {
        const Vec2 i_ab_true = dq_to_alphabeta(sim->state[m].id, sim->state[m].iq, sim->state[m].theta_e);

        /* 电流原始采样：加入比例误差、零偏与噪声 */
        fb->motor[m].i_alpha_raw = i_ab_true.x / sim->sensor_model.gain_alpha + sim->sensor_model.offset_alpha
                                  + 0.04f * sinf(TWO_PI * (900.0f + 30.0f * (float)m) * sim->time_s);
        fb->motor[m].i_beta_raw = i_ab_true.y / sim->sensor_model.gain_beta + sim->sensor_model.offset_beta
                                 + 0.04f * cosf(TWO_PI * (860.0f + 20.0f * (float)m) * sim->time_s);

        /* 回读三相电压 */
        fb->motor[m].v_u = sim->v_u[m];
        fb->motor[m].v_v = sim->v_v[m];
        fb->motor[m].v_w = sim->v_w[m];

        /* 编码器 AB 回读 */
        fb->motor[m].enc_ab.count = sim->enc_count_acc[m];
        fb->motor[m].enc_ab.phase_a = (uint8_t)((fb->motor[m].enc_ab.count >> 0) & 1);
        fb->motor[m].enc_ab.phase_b = (uint8_t)((fb->motor[m].enc_ab.count >> 1) & 1);

        /* 角度/速度测量噪声 */
        fb->motor[m].theta_e = sim->state[m].theta_e + 0.002f * sinf(TWO_PI * (300.0f + 10.0f * (float)m) * sim->time_s);
        fb->motor[m].omega_m = sim->state[m].omega_m + 0.8f * sinf(TWO_PI * (220.0f + 5.0f * (float)m) * sim->time_s);
    }

    return 0;
}

/* 写命令：缓存双电机 PWM 命令并换算三相电压 */
static int sim_write_pwm_cmd(void *ctx, const HalPwmCmd *cmd)
{
    ControlHalSimCtx *sim = (ControlHalSimCtx *)ctx;

    sim->pwm_cmd = *cmd;

    for (int m = 0; m < HAL_MOTOR_COUNT; ++m) {
        const float du = clamp01(cmd->motor[m].duty_u);
        const float dv = clamp01(cmd->motor[m].duty_v);
        const float dw = clamp01(cmd->motor[m].duty_w);

        /* 两电平逆变器相电压近似：v_phase = (duty-0.5)*Vdc */
        sim->v_u[m] = cmd->motor[m].enable ? (du - 0.5f) * sim->vdc : 0.0f;
        sim->v_v[m] = cmd->motor[m].enable ? (dv - 0.5f) * sim->vdc : 0.0f;
        sim->v_w[m] = cmd->motor[m].enable ? (dw - 0.5f) * sim->vdc : 0.0f;

        sim->v_cmd_ab[m] = phase_to_alphabeta(sim->v_u[m], sim->v_v[m], sim->v_w[m]);
    }

    return 0;
}

/* 读故障位 */
static int sim_read_fault_flags(void *ctx, unsigned *flags)
{
    ControlHalSimCtx *sim = (ControlHalSimCtx *)ctx;
    *flags = sim->fault_flags;
    return 0;
}

/* 推进一个控制周期 */
static void sim_wait_next_cycle(void *ctx)
{
    ControlHalSimCtx *sim = (ControlHalSimCtx *)ctx;

    for (int m = 0; m < HAL_MOTOR_COUNT; ++m) {
        const Vec2 v_dq = alphabeta_to_dq(sim->v_cmd_ab[m].x, sim->v_cmd_ab[m].y, sim->state[m].theta_e);
        const float load_torque = 0.18f + 0.08f * sinf(TWO_PI * (6.0f + (float)m) * sim->time_s);
        sim->state[m] = pmsm_step_dq(&sim->plant[m], &sim->state[m], v_dq.x, v_dq.y, load_torque);

        /* 连续机械角累计积分：不做包裹，供编码器累计计数使用 */
        sim->theta_m_acc[m] += sim->state[m].omega_m * sim->plant[m].Ts;
        sim->enc_count_acc[m] = theta_m_to_enc_count(sim->theta_m_acc[m], 1024);
    }

    sim->time_s += sim->plant[0].Ts;

    sim->vdc = 48.0f + 1.8f * sinf(TWO_PI * 120.0f * sim->time_s)
                    + 0.3f * cosf(TWO_PI * 37.0f * sim->time_s);

    sim->fault_flags = HAL_FAULT_NONE;
    for (int m = 0; m < HAL_MOTOR_COUNT; ++m) {
        if (fabsf(sim->v_u[m]) > 100.0f || fabsf(sim->v_v[m]) > 100.0f || fabsf(sim->v_w[m]) > 100.0f) {
            sim->fault_flags |= HAL_FAULT_PWM_TRIP;
            break;
        }
    }
}

/* 初始化仿真 HAL 上下文 */
void control_hal_sim_init(ControlHalSimCtx *sim,
                          const PmsmParams *plant,
                          const CurrentSensorCalib *sensor_model,
                          float init_omega_m)
{
    for (int m = 0; m < HAL_MOTOR_COUNT; ++m) {
        sim->plant[m] = *plant;
        sim->state[m].id = 0.0f;
        sim->state[m].iq = 0.0f;
        sim->state[m].theta_e = 0.0f;
        sim->state[m].omega_m = init_omega_m + 5.0f * (float)m;

        sim->pwm_cmd.motor[m].duty_u = 0.5f;
        sim->pwm_cmd.motor[m].duty_v = 0.5f;
        sim->pwm_cmd.motor[m].duty_w = 0.5f;
        sim->pwm_cmd.motor[m].enable = 0;

        sim->v_u[m] = 0.0f;
        sim->v_v[m] = 0.0f;
        sim->v_w[m] = 0.0f;
        sim->v_cmd_ab[m].x = 0.0f;
        sim->v_cmd_ab[m].y = 0.0f;
        sim->theta_m_acc[m] = 0.0f;
        sim->enc_count_acc[m] = 0;
    }

    sim->sensor_model = *sensor_model;
    sim->vdc = plant->Vdc;
    sim->time_s = 0.0f;
    sim->fault_flags = HAL_FAULT_NONE;
}

/* 绑定 HAL 接口 */
void control_hal_sim_bind(ControlHal *hal, ControlHalSimCtx *sim)
{
    hal->ctx = sim;
    hal->read_feedback = sim_read_feedback;
    hal->write_pwm_cmd = sim_write_pwm_cmd;
    hal->read_fault_flags = sim_read_fault_flags;
    hal->wait_next_cycle = sim_wait_next_cycle;
}
