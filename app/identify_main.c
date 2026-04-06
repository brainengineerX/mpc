#include <stdio.h>

#include "motor_model.h"
#include "motor_identification.h"
#include "adc_calibration.h"
#include "control_hal.h"
#include "control_hal_sim.h"
#include "app_identification.h"

/* ============================================================================
 * 文件：identify_main.c
 * 作用：参数辨识独立入口（只辨识，不跑MPC控制）
 *
 * 使用方式：
 * 1) 编译后运行 identify_demo；
 * 2) 读取输出的 Rs/Ld/Lq/psi_f；
 * 3) 将输出参数填入 control_main 的手工参数区，或写入配置文件。
 * ========================================================================== */

/* AB 编码器参数（示例） */
#define ENCODER_PPR 1024

/* 电参数辨识样本数：样本越多，抗噪能力通常越好，但耗时更长 */
#define ID_SAMPLE_COUNT 800

int main(void)
{
    /* 仿真电机真实参数（实机时由真实硬件决定，这里仅用于 SIM HAL） */
    PmsmParams plant_nominal = {
        .Rs = 0.45f,
        .Ld = 0.0017f,
        .Lq = 0.0020f,
        .psi_f = 0.060f,
        .pole_pairs = 4.0f,
        .J = 0.0007f,
        .B = 0.0002f,
        .Ts = 0.00005f,
        .Vdc = 48.0f
    };

    /* 电流传感器误差模型（用于“原始采样 -> 校准电流”） */
    CurrentSensorCalib adc_cal = {
        .adc_delay_s = 2.0e-6f,
        .lpf_gain = 0.001f,
        .offset_alpha = 0.22f,
        .offset_beta = -0.18f,
        .gain_alpha = 0.94f,
        .gain_beta = 1.07f
    };

    /* 绑定 HAL（当前示例用仿真实现，迁移 GD32 时换成真实 HAL） */
    ControlHal hal;
    ControlHalSimCtx hal_sim;
    control_hal_sim_init(&hal_sim, &plant_nominal, &adc_cal, 90.0f);
    control_hal_sim_bind(&hal, &hal_sim);

    printf("Identification start: sample_count=%d, Ts=%.8f\n", ID_SAMPLE_COUNT, plant_nominal.Ts);

    /* 双电机分别辨识，输出每台电机参数模板 */
    for (int m = 0; m < HAL_MOTOR_COUNT; ++m) {
        MotorIdResult id_res;
        const int rc = app_identify_electrical_via_hal(&hal,
                                                       m,
                                                       ENCODER_PPR,
                                                       plant_nominal.pole_pairs,
                                                       plant_nominal.Ts,
                                                       &adc_cal,
                                                       ID_SAMPLE_COUNT,
                                                       &id_res);
        if (rc != 0) {
            printf("[ID] Motor%d electrical identification failed, rc=%d\n", m, rc);
            return 1;
        }

        printf("[ID] Motor%d elec: Rs=%.5f Ld=%.6f Lq=%.6f psi=%.5f | rms_v=%.6f\n",
               m,
               id_res.Rs,
               id_res.Ld,
               id_res.Lq,
               id_res.psi_f,
               id_res.rms_error);

        /* 输出可直接复制到控制工程配置中的参数模板 */
        printf("[ID] Motor%d MPC params => { .Rs=%.5ff, .Ls=%.6ff, .psi_f=%.5ff, .Ts=%.8ff, .Vdc=%.3ff }\n",
               m,
               id_res.Rs,
               0.5f * (id_res.Ld + id_res.Lq),
               id_res.psi_f,
               plant_nominal.Ts,
               plant_nominal.Vdc);
    }

    printf("Identification done.\n");
    return 0;
}
