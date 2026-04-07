/**
 * @file test_fixed_point_math.c
 * @brief 定点数运算单元测试
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../core/control/fixed_point_types.h"
#include "../core/math/fixed_point_math.h"

/* 简单的测试框架宏 */
#define TEST(name) static int test_##name(void)
#define RUN_TEST(name) run_test(#name, test_##name)

static int tests_passed = 0;
static int tests_failed = 0;

static void run_test(const char *name, int (*test_func)(void)) {
    printf("  Running %s ... ", name);
    if (test_func()) {
        printf("✓ PASSED\n");
        tests_passed++;
    } else {
        printf("✗ FAILED\n");
        tests_failed++;
    }
}

#define ASSERT(cond) do { if (!(cond)) { \
    printf("\n    [ASSERT FAILED] %s:%d: %s", __FILE__, __LINE__, #cond); \
    return 0; \
} } while (0)

#define ASSERT_INT_EQ(expected, actual) do { \
    if ((expected) != (actual)) { \
        printf("\n    [ASSERT FAILED] %s:%d: Expected %d, got %d", \
               __FILE__, __LINE__, (int)(expected), (int)(actual)); \
        return 0; \
    } \
} while (0)

#define ASSERT_FLOAT_NEAR(expected, actual, tolerance) do { \
    if (fabsf((expected) - (actual)) > (tolerance)) { \
        printf("\n    [ASSERT FAILED] %s:%d: Expected %.6f, got %.6f (tol %.6f)", \
               __FILE__, __LINE__, (float)(expected), (float)(actual), (float)(tolerance)); \
        return 0; \
    } \
} while (0)

/* ============================================================================
 * 测试用例
 ============================================================================ */

TEST(q15_basic_ops) {
    /* 测试基本运算 */
    q15_t a = AMP_TO_Q15(5.0f);   /* 5A */
    q15_t b = AMP_TO_Q15(3.0f);   /* 3A */

    /* 加法: 5 + 3 = 8 */
    q15_t sum = q15_add(a, b);
    ASSERT_FLOAT_NEAR(8.0f, Q15_TO_AMP(sum), 0.1f);

    /* 减法: 5 - 3 = 2 */
    q15_t diff = q15_sub(a, b);
    ASSERT_FLOAT_NEAR(2.0f, Q15_TO_AMP(diff), 0.1f);

    /* 乘法: 0.5 * 0.5 = 0.25 */
    q15_t half = Q15_ONE / 2;  /* 约 0.5 */
    q15_t prod = q15_mul(half, half);
    ASSERT_FLOAT_NEAR(0.25f, Q15_TO_FLOAT(prod), 0.02f);

    return 1;
}

TEST(q15_saturation) {
    /* 测试饱和限幅 */
    int32_t large = 50000;  /* 超出 Q15 范围 */
    q15_t sat = Q15_SATURATE(large);
    ASSERT_INT_EQ(Q15_ONE, sat);

    int32_t small = -50000;
    sat = Q15_SATURATE(small);
    ASSERT_INT_EQ(Q15_NEG_ONE, sat);

    /* 加法溢出 */
    q15_t a = Q15_ONE;
    q15_t b = Q15_ONE;
    q15_t sum = q15_add(a, b);
    ASSERT_INT_EQ(Q15_ONE, sum);  /* 饱和到最大值 */

    return 1;
}

TEST(q15_trig_accuracy) {
    /* 测试三角函数精度 (与 math.h 对比) */
    const float angles[] = {0.0f, 0.1f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f};
    const int num_angles = sizeof(angles) / sizeof(angles[0]);

    for (int i = 0; i < num_angles; i++) {
        q15_t angle_q15 = RAD_TO_Q15(angles[i]);

        /* sin 精度检查 */
        q15_t sin_q = q15_sin_lut(angle_q15);
        float sin_f = sinf(angles[i]);
        float sin_q_f = Q15_TO_FLOAT(sin_q);
        ASSERT_FLOAT_NEAR(sin_f, sin_q_f, 0.015f);  /* 误差 < 1.5% */

        /* cos 精度检查 */
        q15_t cos_q = q15_cos_lut(angle_q15);
        float cos_f = cosf(angles[i]);
        float cos_q_f = Q15_TO_FLOAT(cos_q);
        ASSERT_FLOAT_NEAR(cos_f, cos_q_f, 0.015f);
    }

    return 1;
}

TEST(q15_square_accuracy) {
    /* 测试平方精度 */
    q15_t x = FLOAT_TO_Q15(0.5f);  /* 0.5 in Q15 ≈ 16384 */
    q15_t x_sq = q15_square_lut(x);

    /* 0.5^2 = 0.25 */
    float expected = 0.25f;
    float actual = Q15_TO_FLOAT(x_sq);
    ASSERT_FLOAT_NEAR(expected, actual, 0.02f);

    /* 测试负数的平方 */
    q15_t neg_x = -x;
    q15_t neg_x_sq = q15_square_lut(neg_x);
    ASSERT_INT_EQ(x_sq, neg_x_sq);  /* 平方后正负结果相同 */

    return 1;
}

TEST(q15_conversion) {
    /* 测试物理量转换 */
    float amp_in = 10.5f;
    q15_t amp_q = AMP_TO_Q15(amp_in);
    float amp_out = Q15_TO_AMP(amp_q);
    ASSERT_FLOAT_NEAR(amp_in, amp_out, 0.05f);  /* 误差 < 0.05A */

    float volt_in = 45.3f;
    q15_t volt_q = VOLT_TO_Q15(volt_in);
    float volt_out = Q15_TO_VOLT(volt_q);
    ASSERT_FLOAT_NEAR(volt_in, volt_out, 0.2f);  /* 误差 < 0.2V */

    float rad_in = 1.23f;
    q15_t rad_q = RAD_TO_Q15(rad_in);
    float rad_out = Q15_TO_RAD(rad_q);
    ASSERT_FLOAT_NEAR(rad_in, rad_out, 0.02f);  /* 误差 < 0.02 rad */

    return 1;
}

TEST(q30_mac_q15) {
    /* 测试 Q30 * Q15 -> Q15 乘累加 */
    /* 模拟电流预测: i_next = a*i + b*v */

    /* a = 0.95 (Q30), b = 0.05 (Q30) */
    q30_t a_q30 = FLOAT_TO_Q30(0.95f);
    q30_t b_q30 = FLOAT_TO_Q30(0.05f);

    /* i = 5A, v = 10V */
    q15_t i_q = AMP_TO_Q15(5.0f);
    q15_t v_q = VOLT_TO_Q15(10.0f);

    /* i_next = a*i + b*v */
    q15_t i_next = q30_mac_q15(0, a_q30, i_q);
    i_next = q30_mac_q15(i_next, b_q30, v_q);

    /* 浮点参考计算 */
    float i_f = 5.0f, v_f = 10.0f;
    float a_f = 0.95f, b_f = 0.05f;
    float i_next_f = a_f * i_f + b_f * v_f;
    float i_next_q = Q15_TO_AMP(i_next);

    ASSERT_FLOAT_NEAR(i_next_f, i_next_q, 0.15f);

    return 1;
}

TEST(q15_dot2) {
    /* 测试二维向量点积 */
    /* [3, 4] · [3, 4] = 9 + 16 = 25 */
    q15_t a1 = FLOAT_TO_Q15(0.3f);  /* 0.3 in Q15 */
    q15_t a2 = FLOAT_TO_Q15(0.4f);  /* 0.4 in Q15 */

    q15_t dot = q15_dot2(a1, a2, a1, a2);

    /* 结果应该约等于 0.09 + 0.16 = 0.25 in Q15 */
    float expected = 0.09f + 0.16f;
    float actual = Q15_TO_FLOAT(dot);
    ASSERT_FLOAT_NEAR(expected, actual, 0.05f);

    return 1;
}

/* ============================================================================
 * 主函数
 ============================================================================ */

int main(void) {
    printf("========================================\n");
    printf("  Fixed-Point Math Unit Tests\n");
    printf("========================================\n\n");

    RUN_TEST(q15_basic_ops);
    RUN_TEST(q15_saturation);
    RUN_TEST(q15_trig_accuracy);
    RUN_TEST(q15_square_accuracy);
    RUN_TEST(q15_conversion);
    RUN_TEST(q30_mac_q15);
    RUN_TEST(q15_dot2);

    printf("\n========================================\n");
    printf("  Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");

    return (tests_failed == 0) ? 0 : 1;
}
