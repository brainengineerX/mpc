#include <stdlib.h>
#include <math.h>

#include "app_identification.h"
#include "app_hal_utils.h"

#define APP_TWO_PI 6.28318530718f

int app_identify_electrical_via_hal(ControlHal *hal,
                                    int motor_idx,
                                    int ppr,
                                    float pole_pairs,
                                    float Ts,
                                    CurrentSensorCalib *cal,
                                    int sample_count,
                                    MotorIdResult *out)
{
    /* 入参合法性检查：尽早失败，避免后续出现野指针或空采样问题 */
    if (hal == 0 || out == 0 || cal == 0) {
        return -10;
    }
    if (sample_count < 12) {
        /* 最小二乘至少需要足够样本覆盖参数维度 */
        return -11;
    }
    if (motor_idx < 0 || motor_idx >= HAL_MOTOR_COUNT) {
        return -12;
    }

    /* 动态申请样本缓存，让 sample_count 可配置，而不是写死宏 */
    MotorIdSample *samples = (MotorIdSample *)malloc((size_t)sample_count * sizeof(MotorIdSample));
    if (samples == 0) {
        return -13;
    }

    EncoderStateEst enc = {0};      /* 编码器状态估计器 */
    Vec2 idq_prev = {0.0f, 0.0f};   /* 上一拍 dq 电流（用于差分求导） */
    int valid = 0;                  /* 已采集有效样本数 */

    /* 循环 sample_count+1 次：第一拍只建立 prev，后续才能形成导数 */
    for (int k = 0; k < sample_count + 1; ++k) {
        HalFeedback fb;
        HalPwmCmd pwm;
        const float t = (float)k * Ts;

        if (hal->read_feedback(hal->ctx, &fb) != 0) {
            free(samples);
            return -20;
        }

        /* 用 AB 计数更新电角速度/角度，作为 dq 变换角度输入 */
        app_encoder_estimator_update(&enc,
                                     fb.motor[motor_idx].enc_ab.count,
                                     ppr,
                                     pole_pairs,
                                     Ts);

        {
            /* 原始电流先做零偏/比例修正，再进 dq 坐标 */
            const CurrentState raw = {
                fb.motor[motor_idx].i_alpha_raw,
                fb.motor[motor_idx].i_beta_raw
            };
            const CurrentState i_cal = current_sensor_apply_calib(&raw, cal);
            const Vec2 idq = alphabeta_to_dq(i_cal.i_alpha, i_cal.i_beta, enc.theta_e);

            /* 相电压回读转 alpha-beta，再转 dq，作为辨识方程输入 */
            const Vec2 vab = app_phase_to_alphabeta(fb.motor[motor_idx].v_u,
                                                    fb.motor[motor_idx].v_v,
                                                    fb.motor[motor_idx].v_w);
            const Vec2 vdq = alphabeta_to_dq(vab.x, vab.y, enc.theta_e);

            if (k > 0 && valid < sample_count) {
                /* 样本对齐：第 k-1 拍电流作为状态，第 k 拍用于构造导数 */
                samples[valid].id = idq_prev.x;
                samples[valid].iq = idq_prev.y;
                samples[valid].did = (idq.x - idq_prev.x) / Ts;
                samples[valid].diq = (idq.y - idq_prev.y) / Ts;
                samples[valid].vd = vdq.x;
                samples[valid].vq = vdq.y;
                samples[valid].omega_e = pole_pairs * enc.omega_m;
                valid++;
            }

            idq_prev = idq;
        }

        {
            /* 激励电压设计：叠加多频正余弦，提升参数可观测性 */
            const float vd_cmd = 6.0f * sinf(APP_TWO_PI * 180.0f * t)
                               + 2.0f * cosf(APP_TWO_PI * 70.0f * t);
            const float vq_cmd = 8.0f * cosf(APP_TWO_PI * 130.0f * t)
                               + 3.0f * sinf(APP_TWO_PI * 55.0f * t);
            const Vec2 vab_cmd = dq_to_alphabeta(vd_cmd, vq_cmd, enc.theta_e);

            /* 通过 HAL 下发三相 PWM，占空比输出到指定电机 */
            app_build_pwm_cmd_for_motor(&pwm,
                                        motor_idx,
                                        vab_cmd.x,
                                        vab_cmd.y,
                                        fb.vdc,
                                        1);
            hal->write_pwm_cmd(hal->ctx, &pwm);
            hal->wait_next_cycle(hal->ctx);
        }
    }

    {
        /* 调用核心最小二乘求解器得到电机参数 */
        const int rc = motor_identify_pmsm_params(samples, valid, out);
        free(samples);
        return rc;
    }
}
