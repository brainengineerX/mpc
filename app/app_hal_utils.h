#ifndef APP_HAL_UTILS_H
#define APP_HAL_UTILS_H

#include <stdint.h>
#include "motor_model.h"
#include "control_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：app_hal_utils
 * 职责：放置“应用层公共工具函数”，避免在多个 main 入口重复写同样逻辑。
 *
 * 这里的函数都属于“工程胶水层”：
 * 1) 不属于控制核心算法（core/control）；
 * 2) 也不属于硬件抽象层（hal）；
 * 3) 主要用于入口程序中的数据转换与命令组包。
 * ========================================================================== */

/* 编码器估计器状态：由 AB 计数推导电角度和机械角速度 */
typedef struct {
    int inited;         /* 是否已完成首样本初始化 */
    int32_t prev_count; /* 上一拍编码器计数 */
    float theta_e;      /* 当前电角度（rad） */
    float omega_m;      /* 当前机械角速度（rad/s） */
} EncoderStateEst;

/* 常用数学工具：限幅 */
float app_clampf(float x, float min_v, float max_v);

/* 常用数学工具：角度包裹到 [-pi, pi] */
float app_wrap_pm_pi(float x);

/* 常用工程工具：斜坡限速（给参考值做平滑过渡） */
float app_slew_step(float current, float target, float max_delta);

/* AB 编码器计数 -> 电角度/机械角速度 */
void app_encoder_estimator_update(EncoderStateEst *e,
                                  int32_t count,
                                  int ppr,
                                  float pole_pairs,
                                  float Ts);

/* AB 编码器累计计数 -> 机械角位置（rad，多圈不包裹） */
float app_encoder_count_to_mech_rad(int32_t count, int ppr);

/* 三相相电压 -> alpha-beta 电压（Clarke 变换） */
Vec2 app_phase_to_alphabeta(float vu, float vv, float vw);

/* 把 alpha-beta 电压目标组装成双电机 PWM 命令（仅指定一个电机输出） */
void app_build_pwm_cmd_for_motor(HalPwmCmd *cmd,
                                 int motor_idx,
                                 float v_alpha,
                                 float v_beta,
                                 float vdc,
                                 int enable);

#ifdef __cplusplus
}
#endif

#endif /* APP_HAL_UTILS_H */
