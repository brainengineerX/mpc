#ifndef CONTROL_HAL_H
#define CONTROL_HAL_H

#include <stdint.h>    /* int32_t / uint8_t */
#include "motor_model.h" /* 复用 Vec2/CurrentState 等基础类型 */

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * HAL 抽象层（控制侧）
 * --------------------------------------------------------------------------
 * 目标：让算法层只依赖“统一接口”，不依赖具体芯片寄存器或驱动库。
 *
 * 你后续迁移到 GD32F450 时，只需：
 * 1) 新增 control_hal_gd32.c/.h；
 * 2) 实现与这里一致的函数签名；
 * 3) 在 main 中替换绑定函数。
 * ======================================================================== */

/* 双电机数量常量 */
#define HAL_MOTOR_COUNT 2

/* 单电机 AB 编码器反馈 */
typedef struct {
    int32_t count;   /* 编码器累计计数（AB正交解码后） */
    uint8_t phase_a; /* 当前 A 相电平（0/1） */
    uint8_t phase_b; /* 当前 B 相电平（0/1） */
} HalEncoderAbFeedback;

/* 单电机反馈（电流、相电压、编码器与估计状态） */
typedef struct {
    float i_alpha_raw; /* 原始 alpha 电流采样（未校准） */
    float i_beta_raw;  /* 原始 beta 电流采样（未校准） */

    float v_u;         /* U 相相电压（相对中性点，或等效相电压） */
    float v_v;         /* V 相相电压 */
    float v_w;         /* W 相相电压 */

    HalEncoderAbFeedback enc_ab; /* AB 编码器原始反馈 */

    float theta_e;     /* 电角度（由编码器/观测器估算后提供） */
    float omega_m;     /* 机械角速度 */
} HalMotorFeedback;

/* 整体硬件反馈（共享母线 + 双电机） */
typedef struct {
    float vdc;                         /* 实时母线电压（共享） */
    HalMotorFeedback motor[HAL_MOTOR_COUNT]; /* 两个电机反馈槽 */
    float time_s;                      /* 系统时间戳（秒） */
} HalFeedback;

/* 单电机三相PWM命令（高级定时器典型接口） */
typedef struct {
    float duty_u; /* U 相占空比 0~1 */
    float duty_v; /* V 相占空比 0~1 */
    float duty_w; /* W 相占空比 0~1 */
    uint8_t enable; /* 1=使能输出，0=关闭输出 */
} HalMotorPwmCmd;

/* 双电机 PWM 输出命令 */
typedef struct {
    HalMotorPwmCmd motor[HAL_MOTOR_COUNT];
} HalPwmCmd;

/* HAL 统一函数表：
 * - ctx：底层实现上下文（驱动句柄、缓冲区、状态机等）
 * - read_feedback：采样/观测值输入
 * - write_pwm_cmd：双电机三相PWM输出
 * - read_fault_flags：硬件故障输入
 * - wait_next_cycle：阻塞等待或推进到下个控制周期 */
typedef struct {
    void *ctx;                                                       /* 底层上下文指针 */
    int (*read_feedback)(void *ctx, HalFeedback *fb);               /* 读取反馈 */
    int (*write_pwm_cmd)(void *ctx, const HalPwmCmd *cmd);          /* 下发双电机 PWM 命令 */
    int (*read_fault_flags)(void *ctx, unsigned *flags);            /* 读取故障位 */
    void (*wait_next_cycle)(void *ctx);                             /* 周期推进 */
} ControlHal;

/* 通用故障位定义（上层安全模块可直接使用） */
enum {
    HAL_FAULT_NONE = 0,            /* 无故障 */
    HAL_FAULT_ADC_OVERRANGE = 1u << 0, /* ADC 量程越界 */
    HAL_FAULT_PWM_TRIP = 1u << 1,      /* PWM 保护触发 */
    HAL_FAULT_DRIVER_UVLO = 1u << 2    /* 驱动欠压保护 */
};

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_HAL_H */
