#ifndef APP_IDENTIFICATION_H
#define APP_IDENTIFICATION_H

#include "control_hal.h"
#include "adc_calibration.h"
#include "motor_identification.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：app_identification
 * 职责：封装“基于 HAL 的电参数辨识流程”。
 *
 * 设计目的：
 * 1) 让辨识流程可被 identify_main 与 control_main 复用；
 * 2) 把与入口相关的激励生成、样本组装从 main 中解耦；
 * 3) 保持 core/identify 只关心“数学求解”，不关心 HAL 读写细节。
 * ========================================================================== */

/* 通过 HAL 对指定电机做电参数辨识。
 * - motor_idx：电机索引（0 或 1）
 * - ppr：AB 编码器每圈线数（注意函数内部按 AB 四倍频处理）
 * - pole_pairs：极对数（通常手工输入）
 * - Ts：控制周期
 * - cal：电流传感器在线标定参数（用于补偿原始采样）
 * - sample_count：采样数量（建议 >= 200）
 * - out：辨识输出（Rs/Ld/Lq/psi_f）
 * 返回值：0 成功，非 0 失败。 */
int app_identify_electrical_via_hal(ControlHal *hal,
                                    int motor_idx,
                                    int ppr,
                                    float pole_pairs,
                                    float Ts,
                                    CurrentSensorCalib *cal,
                                    int sample_count,
                                    MotorIdResult *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_IDENTIFICATION_H */
