#ifndef MOTOR_PU_PROFILE_H
#define MOTOR_PU_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：motor_pu_profile
 * 作用：提供“电机标幺化基值模板”。
 *
 * 使用方式：
 * 1) 选择一个模板（或自定义模板）；
 * 2) 仅修改 I_base/V_base/omega_base；
 * 3) 控制器权重和限幅按统一规则自动映射。
 * ========================================================================== */

/* 标幺化基值：
 * - i_base：电流基值（A）
 * - v_base：电压基值（V）
 * - omega_base：机械角速度基值（rad/s） */
typedef struct {
    float i_base;
    float v_base;
    float omega_base;
} MotorPuBase;

/* 预置电机模板编号 */
typedef enum {
    MOTOR_PU_TEMPLATE_LIGHT_48V = 0,   /* 轻载小功率 48V 平台 */
    MOTOR_PU_TEMPLATE_MID_48V = 1,     /* 中等功率 48V 平台 */
    MOTOR_PU_TEMPLATE_HEAVY_48V = 2    /* 重载较大功率 48V 平台 */
} MotorPuTemplate;

/* 根据模板编号获取基值（若编号非法，回落到 MID_48V） */
MotorPuBase motor_pu_template_get(MotorPuTemplate id);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PU_PROFILE_H */
