#ifndef PI_FALLBACK_H
#define PI_FALLBACK_H

#include "motor_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：pi_fallback
 * 职责：当MPC不可用或被保护策略降级时，提供可运行的后备电流控制器。
 *
 * 说明：
 * - 这里是 alpha-beta 双通道PI；
 * - 不追求最优性能，但追求“稳态可控、易收敛、易验证”。
 * ========================================================================== */

typedef struct {
    float kp;
    float ki;
    float integ;
    float out_limit;
} PIReg;

typedef struct {
    PIReg alpha;
    PIReg beta;
} PiCurrentController;

void pi_current_controller_init(PiCurrentController *c,
                                float kp,
                                float ki,
                                float out_limit);

Vec2 pi_current_step(PiCurrentController *c,
                     const CurrentRef *r,
                     const CurrentState *x,
                     float Ts);

#ifdef __cplusplus
}
#endif

#endif /* PI_FALLBACK_H */
