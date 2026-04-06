#include "motor_pu_profile.h"

MotorPuBase motor_pu_template_get(MotorPuTemplate id)
{
    /* 轻载模板：电流基值较小，速度基值较高 */
    static const MotorPuBase light_48v = {
        .i_base = 4.0f,
        .v_base = 24.0f,
        .omega_base = 260.0f
    };

    /* 中功率模板：默认推荐模板 */
    static const MotorPuBase mid_48v = {
        .i_base = 8.0f,
        .v_base = 32.0f,
        .omega_base = 200.0f
    };

    /* 重载模板：电流基值较大，速度基值较低 */
    static const MotorPuBase heavy_48v = {
        .i_base = 15.0f,
        .v_base = 32.0f,
        .omega_base = 140.0f
    };

    switch (id) {
        case MOTOR_PU_TEMPLATE_LIGHT_48V:
            return light_48v;
        case MOTOR_PU_TEMPLATE_MID_48V:
            return mid_48v;
        case MOTOR_PU_TEMPLATE_HEAVY_48V:
            return heavy_48v;
        default:
            return mid_48v;
    }
}
