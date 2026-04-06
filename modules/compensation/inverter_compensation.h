#ifndef INVERTER_COMPENSATION_H
#define INVERTER_COMPENSATION_H

#include "motor_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：inverter_compensation
 * 职责：补偿逆变器非理想效应（死区、器件压降）。
 *
 * 背景：
 * 理想模型认为输出电压等于调制定值，但实际逆变器受以下影响：
 * 1) 死区时间导致有效占空偏差（与电流方向相关）；
 * 2) 器件导通压降/导通电阻导致电压损失。
 *
 * 不补偿会出现：
 * - 小电流段畸变明显；
 * - 电流过零附近跟踪误差大；
 * - 低速工况扭矩脉动上升。
 * ========================================================================== */

/* 逆变器非理想补偿参数 */
typedef struct {
    float pwm_period_s;    /* PWM周期 */
    float deadtime_s;      /* 死区时间 */
    float v_diode;         /* 二极管压降近似 */
    float r_on;            /* 开关导通等效电阻 */
    float current_epsilon; /* 电流符号判定阈值 */
} InverterCompConfig;

/* 对 alpha-beta 指令电压做补偿：
 * 1) 死区补偿（与电流方向相关）
 * 2) 器件压降补偿（Vd + Ron*|I|）
 * 输出是补偿后的 alpha-beta 电压。
 *
 * 调参建议：
 * - deadtime_s 先按硬件定时器配置；
 * - v_diode / r_on 可先查器件手册，再做在线修正；
 * - current_epsilon 设成小电流阈值，避免过零抖动。 */
Vec2 inverter_compensate_voltage_ab(const Vec2 *v_cmd_ab,
                                    const CurrentState *i_ab,
                                    float vdc_meas,
                                    const InverterCompConfig *cfg);

#ifdef __cplusplus
}
#endif

#endif /* INVERTER_COMPENSATION_H */
