#ifndef CONTROL_HAL_SIM_H
#define CONTROL_HAL_SIM_H

#include "control_hal.h"
#include "adc_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：control_hal_sim
 * 职责：提供一个“HAL仿真实现”，用于：
 * 1) 在PC环境验证算法逻辑；
 * 2) 在不依赖真实硬件前提下联调控制链路；
 * 3) 作为GD32 BSP实现的参考模板。
 * ========================================================================== */

typedef struct {
    /* 两个电机对象参数与状态（双电机） */
    PmsmParams plant[HAL_MOTOR_COUNT];
    PmsmState state[HAL_MOTOR_COUNT];

    /* 传感器误差模型（offset/gain） */
    CurrentSensorCalib sensor_model;

    /* 最近一次下发的双电机三相 PWM 命令 */
    HalPwmCmd pwm_cmd;

    /* 由PWM换算得到的三相相电压与 alpha-beta 电压（每电机一份） */
    float v_u[HAL_MOTOR_COUNT];
    float v_v[HAL_MOTOR_COUNT];
    float v_w[HAL_MOTOR_COUNT];
    Vec2 v_cmd_ab[HAL_MOTOR_COUNT];
    /* 连续机械角累计值与对应编码器累计计数（用于避免角度包裹导致的计数跳变） */
    float theta_m_acc[HAL_MOTOR_COUNT];
    int32_t enc_count_acc[HAL_MOTOR_COUNT];
    /* 当前母线电压与时间 */
    float vdc;
    float time_s;
    /* 故障位（模拟硬件故障） */
    unsigned fault_flags;
} ControlHalSimCtx;

void control_hal_sim_init(ControlHalSimCtx *sim,
                          const PmsmParams *plant,
                          const CurrentSensorCalib *sensor_model,
                          float init_omega_m);

void control_hal_sim_bind(ControlHal *hal, ControlHalSimCtx *sim);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_HAL_SIM_H */
