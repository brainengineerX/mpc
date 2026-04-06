/**
 * @file fixed_point_math.c
 * @brief 定点数数学运算库实现
 */

#include "fixed_point_math.h"
#include <math.h>

/* ============================================================================
 * 三角函数查表 (256 点)
 * ============================================================================ */

#define TRIG_LUT_SIZE 256
#define TRIG_LUT_MASK 0xFF

/* sin 查表: 256 点覆盖 [0, 2*pi)，每步 2*pi/256 = pi/128 */
static q15_t sin_lut[TRIG_LUT_SIZE];
static bool trig_lut_initialized = false;

/**
 * @brief 初始化三角函数查表
 */
void q15_trig_lut_init(void) {
    if (trig_lut_initialized) {
        return;
    }

    for (int i = 0; i < TRIG_LUT_SIZE; i++) {
        /* 角度: i * 2*pi / 256 = i * pi / 128 */
        float angle = (float)i * 6.28318530718f / 256.0f;
        float sin_val = sinf(angle);
        /* 转换为 Q15，饱和处理 */
        int32_t q_val = (int32_t)(sin_val * 32768.0f);
        if (q_val > 32767) q_val = 32767;
        if (q_val < -32768) q_val = -32768;
        sin_lut[i] = (q15_t)q_val;
    }

    trig_lut_initialized = true;
}

/**
 * @brief 获取 sin 表指针
 */
const q15_t* q15_get_sin_lut(void) {
    if (!trig_lut_initialized) {
        q15_trig_lut_init();
    }
    return sin_lut;
}

/* ============================================================================
 * 三角函数查表实现
 * ============================================================================ */

/**
 * @brief 查表法 sin 函数
 * @details theta 范围 [-32768, 32767] 对应 [-pi, pi)
 *          映射到 256 点查表: index = (theta + 32768) >> 8
 */
q15_t q15_sin_lut(q15_t theta) {
    if (!trig_lut_initialized) {
        q15_trig_lut_init();
    }

    /* 将 [-32768, 32767] 映射到 [0, 255] */
    /* theta + 32768 范围 [0, 65535]，右移 8 位得到 [0, 255] */
    uint16_t index = (uint16_t)((int32_t)theta + 32768) >> 8;
    return sin_lut[index & TRIG_LUT_MASK];
}

/**
 * @brief 查表法 cos 函数
 * @details cos(theta) = sin(theta + pi/2)
 *          pi/2 对应 32768/2 = 16384 (Q15 中 pi = 32768)
 */
q15_t q15_cos_lut(q15_t theta) {
    /* cos(theta) = sin(theta + pi/2) */
    /* pi/2 在 Q15 表示为 16384 (32768/2) */
    q15_t theta_shifted = theta + 16384;
    return q15_sin_lut(theta_shifted);
}

/* ============================================================================
 * 基本定点运算实现
 * ============================================================================ */

/**
 * @brief Q15 乘法 (Q15 * Q15 -> Q15)
 * @details (a * b) >> 15，使用 32 位中间结果防止溢出
 */
q15_t q15_mul(q15_t a, q15_t b) {
    int32_t temp = (int32_t)a * (int32_t)b;
    /* 右移 15 位，四舍五入 */
    temp = (temp + (1 << 14)) >> 15;
    /* 饱和处理 */
    if (temp > 32767) temp = 32767;
    if (temp < -32768) temp = -32768;
    return (q15_t)temp;
}

/**
 * @brief Q30 与 Q15 乘法累加 (Q30 * Q15 -> Q15)
 * @details acc + (coeff * val >> 30)
 *          用于电流预测: i_next = a*i + b*v
 */
q15_t q30_mac_q15(q15_t acc, q30_t coeff, q15_t val) {
    /* coeff (Q30) * val (Q15) = 结果 (Q45) */
    int64_t product = (int64_t)coeff * (int64_t)val;
    /* 右移 30 位得到 Q15 */
    product = (product + (1LL << 29)) >> 30;
    /* 加上累加器 */
    int32_t result = (int32_t)acc + (int32_t)product;
    /* 饱和处理 */
    if (result > 32767) result = 32767;
    if (result < -32768) result = -32768;
    return (q15_t)result;
}

/**
 * @brief Q15 饱和加法
 */
q15_t q15_add(q15_t a, q15_t b) {
    int32_t sum = (int32_t)a + (int32_t)b;
    if (sum > 32767) sum = 32767;
    if (sum < -32768) sum = -32768;
    return (q15_t)sum;
}

/**
 * @brief Q15 取反
 */
q15_t q15_neg(q15_t a) {
    if (a == -32768) {
        /* -(-32768) 会溢出，饱和到 32767 */
        return 32767;
    }
    return -a;
}

/**
 * @brief Q15 算术右移
 */
q15_t q15_shr(q15_t a, int shift) {
    if (shift <= 0) {
        return a;
    }
    if (shift >= 15) {
        return (a < 0) ? -1 : 0;
    }
    return a >> shift;
}

/**
 * @brief Q15 左移
 */
q15_t q15_shl(q15_t a, int shift) {
    if (shift <= 0) {
        return a;
    }
    int32_t result = (int32_t)a << shift;
    /* 饱和处理 */
    if (result > 32767) result = 32767;
    if (result < -32768) result = -32768;
    return (q15_t)result;
}

/* ============================================================================
 * 数组转换函数
 * ============================================================================ */

void float_array_to_q15(q15_t *dst, const float *src, int len, float scale) {
    for (int i = 0; i < len; i++) {
        float val = src[i] * scale;
        if (val > 32767.0f) val = 32767.0f;
        if (val < -32768.0f) val = -32768.0f;
        dst[i] = (q15_t)val;
    }
}

void q15_array_to_float(float *dst, const q15_t *src, int len, float scale) {
    for (int i = 0; i < len; i++) {
        dst[i] = (float)src[i] / scale;
    }
}
