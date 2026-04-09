/**
 * @file fixed_point_math.h
 * @brief 定点数数学运算库
 *
 * 本文件提供定点数的数学运算函数，包括：
 * - 基本运算：乘法、乘累加、移位
 * - 三角函数：查表法 sin/cos
 * - 坐标变换：Clarke/Park 变换支持函数
 */

#ifndef FIXED_POINT_MATH_H
#define FIXED_POINT_MATH_H

#include "fixed_point_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 基本定点运算
 * ============================================================================ */

/**
 * @brief Q15 乘法 (Q15 * Q15 -> Q15)
 * @details 两个 Q15 数相乘，结果右移 15 位回到 Q15 格式
 * @param a 乘数 (Q15)
 * @param b 被乘数 (Q15)
 * @return 乘积 (Q15)，结果自动饱和限幅
 */
q15_t q15_mul(q15_t a, q15_t b);

/**
 * @brief Q30 与 Q15 乘法 (Q30 * Q15 -> Q15)
 * @details 用于电流预测中的系数相乘: result = coeff * val >> 30
 * @param coeff 系数 (Q30)
 * @param val 变量值 (Q15)
 * @return 乘积结果 (Q15): (coeff * val) >> 30
 */
q15_t q30_mul_q15(q30_t coeff, q15_t val);

/**
 * @brief Q30 与 Q15 乘法累加 (Q30 * Q15 -> Q15)
 * @details 用于电流预测: i_next = a*i + b*v，其中 a,b 是 Q30，i,v 是 Q15
 * @param acc 累加器初始值 (Q15)
 * @param coeff 系数 (Q30)
 * @param val 变量值 (Q15)
 * @return 累加结果 (Q15): acc + (coeff * val >> 30)
 */
q15_t q30_mac_q15(q15_t acc, q30_t coeff, q15_t val);

/**
 * @brief Q15 饱和加法
 * @param a 加数 (Q15)
 * @param b 加数 (Q15)
 * @return 和 (Q15)，饱和限幅到 [-1, 1-2^-15]
 */
q15_t q15_add(q15_t a, q15_t b);

/**
 * @brief Q15 取反
 * @param a 输入 (Q15)
 * @return -a (Q15)，注意 -(-32768) = -32768 (饱和)
 */
q15_t q15_neg(q15_t a);

/**
 * @brief Q15 右移（算术右移）
 * @param a 输入 (Q15)
 * @param shift 右移位数
 * @return a >> shift (Q15)
 */
q15_t q15_shr(q15_t a, int shift);

/**
 * @brief Q15 左移
 * @param a 输入 (Q15)
 * @param shift 左移位数
 * @return a << shift (Q15)，溢出部分截断
 */
q15_t q15_shl(q15_t a, int shift);

/**
 * @brief Q15 减法
 * @param a 被减数 (Q15)
 * @param b 减数 (Q15)
 * @return a - b (Q15)，带饱和
 */
q15_t q15_sub(q15_t a, q15_t b);

/**
 * @brief Q15 平方（查表法）
 * @param x 输入 (Q15)
 * @return x * x (Q15)
 */
q15_t q15_square_lut(q15_t x);

/**
 * @brief Q15 除法（查表法）
 * @param num 被除数 (Q15)
 * @param den 除数 (Q15)
 * @return num / den (Q15)
 */
q15_t q15_div_lut(q15_t num, q15_t den);

/**
 * @brief Q15 二维向量点积
 * @details 计算两个二维向量的点积: a1*b1 + a2*b2
 * @param a1 向量A的第一个分量 (Q15)
 * @param a2 向量A的第二个分量 (Q15)
 * @param b1 向量B的第一个分量 (Q15)
 * @param b2 向量B的第二个分量 (Q15)
 * @return 点积结果 (Q15)
 */
q15_t q15_dot2(q15_t a1, q15_t a2, q15_t b1, q15_t b2);

/* ============================================================================
 * 查表法三角函数
 * ============================================================================ */

/**
 * @brief 查表法 sin 函数
 * @details 使用 256 点查表，theta 范围 [0, 2*pi) 映射到 [0, 65536)
 * @param theta 角度 (Q15)，范围 [-32768, 32767] 对应 [-pi, pi)
 * @return sin(theta) (Q15)，范围 [-32767, 32767]
 */
q15_t q15_sin_lut(q15_t theta);

/**
 * @brief 查表法 cos 函数
 * @details cos(theta) = sin(theta + pi/2)
 * @param theta 角度 (Q15)，范围 [-32768, 32767]
 * @return cos(theta) (Q15)，范围 [-32767, 32767]
 */
q15_t q15_cos_lut(q15_t theta);

/**
 * @brief 初始化三角函数查表
 * @details 在系统启动时调用一次，初始化 sin/cos 查表
 */
void q15_trig_lut_init(void);

/**
 * @brief 获取三角函数表指针（调试用）
 * @return sin 表指针
 */
const q15_t* q15_get_sin_lut(void);

/* ============================================================================
 * 定点数转换辅助函数
 * ============================================================================ */

/**
 * @brief 浮点数组转换为 Q15 数组
 * @param dst 目标 Q15 数组
 * @param src 源浮点数组
 * @param len 数组长度
 * @param scale 缩放因子 (浮点值 = Q15值 / scale)
 */
void float_array_to_q15(q15_t *dst, const float *src, int len, float scale);

/**
 * @brief Q15 数组转换为浮点数组
 * @param dst 目标浮点数组
 * @param src 源 Q15 数组
 * @param len 数组长度
 * @param scale 缩放因子 (浮点值 = Q15值 / scale)
 */
void q15_array_to_float(float *dst, const q15_t *src, int len, float scale);

#ifdef __cplusplus
}
#endif

#endif /* FIXED_POINT_MATH_H */
