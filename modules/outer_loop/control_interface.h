#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "cascade_pi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：control_interface
 * 作用：对外提供统一控制命令接口。
 *
 * 设计目标：
 * 1) 上位机/任务层只设置“模式 + 目标量”，不直接碰内部 PI 状态；
 * 2) 支持电流模式、速度模式、位置模式三类常用命令；
 * 3) 输出统一为电流环可直接使用的 id_ref / iq_ref。
 * ========================================================================== */

/* 控制模式枚举：
 * - CURRENT：直接给电流参考；
 * - SPEED：给速度参考，由速度环算 iq_ref；
 * - POSITION：给位置参考，由位置环算速度，再由速度环算 iq_ref。 */
typedef enum {
    CTRL_MODE_CURRENT = 0,
    CTRL_MODE_SPEED = 1,
    CTRL_MODE_POSITION = 2
} ControlMode;

/* 控制接口配置：
 * 这些参数通常在系统初始化时配置一次。 */
typedef struct {
    float omega_ref_max;     /* 速度参考绝对值上限（rad/s） */
    float iq_ref_max;        /* q轴电流参考绝对值上限（A） */
    float theta_err_max;     /* 位置误差限幅（rad） */

    int speed_loop_div;      /* 速度环执行分频：每 N 个电流周期执行一次 */
    int pos_loop_div;        /* 位置环执行分频：每 N 个电流周期执行一次 */

    float id_slew_step;      /* id 命令每拍最大变化量（A/step） */
    float iq_slew_step;      /* iq 命令每拍最大变化量（A/step） */

    float pos_kp;            /* 位置环 Kp */
    float pos_ki;            /* 位置环 Ki */
    float pos_kaw;           /* 位置环抗积分饱和 */

    float spd_kp;            /* 速度环 Kp */
    float spd_ki;            /* 速度环 Ki */
    float spd_kaw;           /* 速度环抗积分饱和 */

    /* 自动弱磁配置：
     * 说明：弱磁通过注入负 id 来降低等效磁链，常用于基速以上扩速。 */
    int fw_enable_auto;      /* 1=启用自动弱磁，0=关闭 */
    float omega_fw_enter;    /* 弱磁进入阈值（|omega|，rad/s） */
    float omega_fw_exit;     /* 弱磁退出阈值（|omega|，rad/s），应小于 enter 形成迟滞 */
    float id_fw_min;         /* 弱磁 id 下限（A，负值） */
    float id_fw_gain;        /* 弱磁增益（A/(rad/s)） */
    float id_fw_lpf;         /* 弱磁一阶滤波系数 0~1 */
} ControlInterfaceConfig;

/* 控制命令：外部只需要设置这个结构体即可改变控制目标。 */
typedef struct {
    ControlMode mode;        /* 当前模式 */
    float id_ref_cmd;        /* 电流模式下 d轴参考（A） */
    float iq_ref_cmd;        /* 电流模式下 q轴参考（A） */
    float omega_ref_cmd;     /* 速度模式下机械角速度参考（rad/s） */
    float theta_ref_cmd;     /* 位置模式下机械角位置参考（rad，多圈） */
} ControlCommand;

/* 控制接口内部状态：
 * 应用层通常只创建对象，不直接改内部字段。 */
typedef struct {
    ControlInterfaceConfig cfg;  /* 配置副本 */
    ControlCommand cmd;          /* 当前命令 */

    CascadePiController pos_pi;  /* 位置环 PI 对象 */
    CascadePiController spd_pi;  /* 速度环 PI 对象 */

    float omega_ref_int;         /* 内部速度参考（位置环输出） */
    float iq_ref_int;            /* 内部 iq 参考（速度环输出） */
    float id_ref_out;            /* 斜坡后 d轴输出参考 */
    float iq_ref_out;            /* 斜坡后 q轴输出参考 */
    float id_fw_state;           /* 弱磁注入 id 状态（A） */
    int fw_active;               /* 弱磁当前激活标志 */

    int tick;                    /* 控制节拍计数器 */
    ControlMode last_mode;       /* 上一拍模式（用于检测模式切换） */
} ControlInterface;

/* 反馈输入：由控制主循环每拍传入。 */
typedef struct {
    float theta_mech;            /* 当前机械角位置（rad，多圈） */
    float omega_mech;            /* 当前机械角速度（rad/s） */
    float iq_feedforward;        /* q轴前馈电流（A），如负载观测前馈 */
    float Ts;                    /* 当前控制周期（s） */
} ControlFeedback;

/* 接口输出：给电流环的最终参考。 */
typedef struct {
    float id_ref;                /* d轴参考（A） */
    float iq_ref;                /* q轴参考（A） */
    float omega_ref_dbg;         /* 调试用：当前内部速度参考（rad/s） */
    ControlMode mode_dbg;        /* 调试用：当前模式 */
    float id_fw_dbg;             /* 调试用：弱磁注入 id（A） */
    int fw_active_dbg;           /* 调试用：弱磁是否激活 */
    float theta_err_dbg;         /* 调试用：位置误差（rad） */
    float omega_err_dbg;         /* 调试用：速度误差（rad/s） */
} ControlRefOutput;

/* 初始化控制接口：写入配置，清空状态。 */
void control_interface_init(ControlInterface *ci,
                            const ControlInterfaceConfig *cfg);

/* 设置控制模式。 */
void control_interface_set_mode(ControlInterface *ci, ControlMode mode);

/* 设置“电流模式”命令。 */
void control_interface_set_current_target(ControlInterface *ci,
                                          float id_ref,
                                          float iq_ref);

/* 设置“速度模式”命令。 */
void control_interface_set_speed_target(ControlInterface *ci,
                                        float omega_ref);

/* 设置“位置模式”命令。 */
void control_interface_set_position_target(ControlInterface *ci,
                                           float theta_ref);

/* 单步更新：输入反馈，输出电流参考。 */
void control_interface_step(ControlInterface *ci,
                            const ControlFeedback *fb,
                            ControlRefOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_INTERFACE_H */
