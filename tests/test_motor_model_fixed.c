/**
 * @file test_motor_model_fixed.c
 * @brief 定点化电机模型单元测试
 *
 * 使用 Unity 测试框架测试：
 * - 电机参数初始化精度
 * - 电流预测精度（与浮点实现对比）
 * - Park/Clarke 变换精度
 * - 边界条件处理
 */

#include "unity.h"
#include "motor_model_fixed.h"
#include "fixed_point_math.h"
#include <math.h>

void setUp(void) {
    /* 每个测试前初始化三角函数查表 */
    q15_trig_lut_init();
}

void tearDown(void) {
    /* 清理工作 */
}

/* ============================================================================
 * 测试 1: 电机参数初始化
 * ============================================================================ */

void test_motor_params_q15_init(void) {
    MotorParamsQ15 params;

    /* 典型参数: Rs=0.45Ω, Ls=1.85mH, Ts=50μs, Vdc=48V
     * 基准值: i_base=32A, v_base=100V
     */
    motor_params_q15_init(&params, 0.45f, 0.00185f, 50e-6f, 48.0f, 32.0f, 100.0f);

    /* 验证 a_coeff: a = 1 - Ts*Rs/Ls = 1 - 50e-6*0.45/0.00185 ≈ 0.9878
     * Q30 值 ≈ 0.9878 * 2^30 ≈ 1060308700
     */
    int32_t expected_a = (int32_t)(0.9878f * 1073741824.0f);  /* 2^30 = 1073741824 */
    TEST_ASSERT_INT32_WITHIN(2000000, expected_a, params.a_coeff);

    /* 验证 b_coeff: b = Ts/Ls = 50e-6/0.00185 ≈ 0.027027
     * Q30 值 ≈ 0.027027 * 2^30 ≈ 29000000
     */
    int32_t expected_b = (int32_t)(0.027027f * 1073741824.0f);
    TEST_ASSERT_INT32_WITHIN(500000, expected_b, params.b_coeff);

    /* 验证 v_dc: Vdc / v_base = 48/100 = 0.48
     * Q15 值 = 0.48 * 32767 ≈ 15728
     */
    int16_t expected_vdc = (int16_t)(0.48f * 32767.0f);
    TEST_ASSERT_INT16_WITHIN(10, expected_vdc, params.v_dc);
}

/* ============================================================================
 * 测试 2: 电流预测精度（与浮点实现对比）
 * ============================================================================ */

void test_motor_predict_current_q15_accuracy(void) {
    MotorParamsQ15 params;
    CurrentStateQ15 x, x_next;
    VoltageQ15 v;

    /* 初始化电机参数 */
    motor_params_q15_init(&params, 0.45f, 0.00185f, 50e-6f, 48.0f, 32.0f, 100.0f);

    /* 测试条件: i_alpha = 5A, i_beta = 3A, v_alpha = 10V, v_beta = 5V
     * 基准值: i_base = 32A, v_base = 100V
     */
    x.i_alpha = AMP_TO_Q15(5.0f);   /* 5A */
    x.i_beta = AMP_TO_Q15(3.0f);    /* 3A */
    v.v_alpha = VOLT_TO_Q15(10.0f);  /* 10V */
    v.v_beta = VOLT_TO_Q15(5.0f);    /* 5V */

    /* 执行定点预测 */
    x_next = motor_predict_current_q15(&x, &v, &params);

    /* 浮点参考计算 */
    float i_alpha_f = 5.0f, i_beta_f = 3.0f;
    float v_alpha_f = 10.0f, v_beta_f = 5.0f;
    float a_f = 1.0f - 50e-6f * 0.45f / 0.00185f;  /* ≈ 0.9878 */
    float b_f = 50e-6f / 0.00185f;                  /* ≈ 0.027027 */

    float i_alpha_next_f = a_f * i_alpha_f + b_f * v_alpha_f;
    float i_beta_next_f = a_f * i_beta_f + b_f * v_beta_f;

    /* 转换定点结果为浮点比较 */
    float i_alpha_next_q = Q15_TO_AMP(x_next.i_alpha);
    float i_beta_next_q = Q15_TO_AMP(x_next.i_beta);

    /* 计算误差 */
    float err_alpha = fabsf(i_alpha_next_q - i_alpha_next_f);
    float err_beta = fabsf(i_beta_next_q - i_beta_next_f);

    /* 断言误差 < 0.1A (考虑到 Q15 精度限制) */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, i_alpha_next_f, i_alpha_next_q);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, i_beta_next_f, i_beta_next_q);
}

/* ============================================================================
 * 测试 3: Park 变换精度
 * ============================================================================ */

void test_alphabeta_to_dq_q15_accuracy(void) {
    q15_t i_alpha = AMP_TO_Q15(5.0f);    /* 5A */
    q15_t i_beta = AMP_TO_Q15(3.0f);     /* 3A */
    q15_t theta = RAD_TO_Q15(0.5f);      /* 0.5 rad ≈ 28.6° */
    q15_t i_d, i_q;

    /* 执行 Park 变换 */
    alphabeta_to_dq_q15(i_alpha, i_beta, theta, &i_d, &i_q);

    /* 浮点参考计算 */
    float i_alpha_f = 5.0f, i_beta_f = 3.0f, theta_f = 0.5f;
    float cos_t = cosf(theta_f);
    float sin_t = sinf(theta_f);
    float i_d_f = i_alpha_f * cos_t + i_beta_f * sin_t;
    float i_q_f = -i_alpha_f * sin_t + i_beta_f * cos_t;

    /* 转换回安培比较 */
    float i_d_q = Q15_TO_AMP(i_d);
    float i_q_q = Q15_TO_AMP(i_q);

    /* 断言误差 < 0.15A (查表法引入的额外误差) */
    TEST_ASSERT_FLOAT_WITHIN(0.15f, i_d_f, i_d_q);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, i_q_f, i_q_q);
}

/* ============================================================================
 * 测试 4: 逆 Park 变换精度
 * ============================================================================ */

void test_dq_to_alphabeta_q15_accuracy(void) {
    q15_t i_d = AMP_TO_Q15(4.0f);        /* 4A */
    q15_t i_q = AMP_TO_Q15(2.0f);        /* 2A */
    q15_t theta = RAD_TO_Q15(-0.3f);     /* -0.3 rad */
    q15_t i_alpha, i_beta;

    /* 执行逆 Park 变换 */
    dq_to_alphabeta_q15(i_d, i_q, theta, &i_alpha, &i_beta);

    /* 浮点参考计算 */
    float i_d_f = 4.0f, i_q_f = 2.0f, theta_f = -0.3f;
    float cos_t = cosf(theta_f);
    float sin_t = sinf(theta_f);
    float i_alpha_f = i_d_f * cos_t - i_q_f * sin_t;
    float i_beta_f = i_d_f * sin_t + i_q_f * cos_t;

    /* 转换回安培比较 */
    float i_alpha_q = Q15_TO_AMP(i_alpha);
    float i_beta_q = Q15_TO_AMP(i_beta);

    /* 断言误差 < 0.15A */
    TEST_ASSERT_FLOAT_WITHIN(0.15f, i_alpha_f, i_alpha_q);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, i_beta_f, i_beta_q);
}

/* ============================================================================
 * 测试 5: Clarke 变换精度
 * ============================================================================ */

void test_abc_to_alphabeta_q15_accuracy(void) {
    /* 三相电流: i_a = 5A, i_b = -3A, i_c = -2A (满足平衡条件) */
    q15_t i_a = AMP_TO_Q15(5.0f);
    q15_t i_b = AMP_TO_Q15(-3.0f);
    q15_t i_c = AMP_TO_Q15(-2.0f);
    q15_t i_alpha, i_beta;

    /* 执行 Clarke 变换 */
    abc_to_alphabeta_q15(i_a, i_b, i_c, &i_alpha, &i_beta);

    /* 浮点参考计算 (简化公式，假设三相平衡) */
    /* i_alpha = i_a */
    /* i_beta = (i_a + 2*i_b) / sqrt(3) */
    float i_a_f = 5.0f, i_b_f = -3.0f;
    float i_alpha_f = i_a_f;
    float i_beta_f = (i_a_f + 2.0f * i_b_f) / 1.73205080757f;  /* sqrt(3) */

    /* 转换回安培比较 */
    float i_alpha_q = Q15_TO_AMP(i_alpha);
    float i_beta_q = Q15_TO_AMP(i_beta);

    /* 断言误差 < 0.2A (Clarke 变换有更多中间步骤) */
    TEST_ASSERT_FLOAT_WITHIN(0.2f, i_alpha_f, i_alpha_q);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, i_beta_f, i_beta_q);
}

/* ============================================================================
 * 测试 6: 边界条件
 * ============================================================================ */

void test_motor_model_q15_edge_cases(void) {
    MotorParamsQ15 params;
    motor_params_q15_init(&params, 0.45f, 0.00185f, 50e-6f, 48.0f, 32.0f, 100.0f);

    /* 测试 1: 零输入 */
    CurrentStateQ15 x_zero = {0, 0};
    VoltageQ15 v_zero = {0, 0};
    CurrentStateQ15 x_next = motor_predict_current_q15(&x_zero, &v_zero, &params);
    TEST_ASSERT_INT16_WITHIN(10, 0, x_next.i_alpha);
    TEST_ASSERT_INT16_WITHIN(10, 0, x_next.i_beta);

    /* 测试 2: 最大电流输入（饱和条件） */
    /* Q15_ONE = 32767 ≈ 32A (使用 AMP_TO_Q15 宏) */
    CurrentStateQ15 x_max = {Q15_ONE, Q15_ONE};  /* 约 32A */
    VoltageQ15 v_max = {Q15_ONE, Q15_ONE};       /* 约 100V */
    x_next = motor_predict_current_q15(&x_max, &v_max, &params);

    /* 检查输出是否有效（不溢出） */
    TEST_ASSERT_TRUE(x_next.i_alpha >= Q15_NEG_ONE && x_next.i_alpha <= Q15_ONE);
    TEST_ASSERT_TRUE(x_next.i_beta >= Q15_NEG_ONE && x_next.i_beta <= Q15_ONE);

    /* 测试 3: 负电流 */
    CurrentStateQ15 x_neg = {Q15_NEG_ONE, (q15_t)(-16384)};  /* -32A, -16A */
    VoltageQ15 v_neg = {(q15_t)(-16384), Q15_NEG_ONE};       /* -50V, -100V */
    x_next = motor_predict_current_q15(&x_neg, &v_neg, &params);

    /* 检查输出有效 */
    TEST_ASSERT_TRUE(x_next.i_alpha >= Q15_NEG_ONE && x_next.i_alpha <= Q15_ONE);
    TEST_ASSERT_TRUE(x_next.i_beta >= Q15_NEG_ONE && x_next.i_beta <= Q15_ONE);
}

/* ============================================================================
 * 测试 7: 死区补偿参考电压计算
 * ============================================================================ */

void test_deadbeat_voltage_ref_q15(void) {
    MotorParamsQ15 params;
    motor_params_q15_init(&params, 0.45f, 0.00185f, 50e-6f, 48.0f, 32.0f, 100.0f);

    /* 测试条件 */
    CurrentStateQ15 x = {
        .i_alpha = AMP_TO_Q15(5.0f),   /* 5A */
        .i_beta = AMP_TO_Q15(3.0f)     /* 3A */
    };
    CurrentRefQ15 r = {
        .i_alpha_ref = AMP_TO_Q15(6.0f),  /* 目标 6A */
        .i_beta_ref = AMP_TO_Q15(4.0f)    /* 目标 4A */
    };
    VoltageQ15 v_ref;

    /* 计算死区补偿参考电压 */
    deadbeat_voltage_ref_q15(&x, &r, &params, &v_ref);

    /* 验证输出电压在有效范围内 */
    /* 母线电压 48V，最大输出约为 48/sqrt(3) ≈ 27.7V (线电压)
     * 相电压最大约为 27.7/sqrt(3) ≈ 16V
     * 但我们的限幅使用 v_dc/2 = 24V 作为保守估计
     */
    q15_t v_limit = VOLT_TO_Q15(24.0f);  /* 24V 限幅 */

    TEST_ASSERT_TRUE(v_ref.v_alpha >= -v_limit && v_ref.v_alpha <= v_limit);
    TEST_ASSERT_TRUE(v_ref.v_beta >= -v_limit && v_ref.v_beta <= v_limit);
}

/* ============================================================================
 * 测试 8: 不同角度下的 Park 变换
 * ============================================================================ */

void test_park_transform_various_angles(void) {
    /* 测试向量 */
    float i_alpha_f = 5.0f, i_beta_f = 0.0f;  /* 沿 alpha 轴的电流 */

    /* 测试不同角度 */
    float test_angles[] = {0.0f, 0.5f, 1.0f, 1.5708f, -0.5f, 3.14159f};
    int num_angles = sizeof(test_angles) / sizeof(test_angles[0]);

    for (int i = 0; i < num_angles; i++) {
        q15_t theta = RAD_TO_Q15(test_angles[i]);
        q15_t i_alpha = AMP_TO_Q15(i_alpha_f);
        q15_t i_beta = AMP_TO_Q15(i_beta_f);
        q15_t i_d, i_q;

        alphabeta_to_dq_q15(i_alpha, i_beta, theta, &i_d, &i_q);

        /* 浮点参考计算 */
        float cos_t = cosf(test_angles[i]);
        float sin_t = sinf(test_angles[i]);
        float i_d_f = i_alpha_f * cos_t + i_beta_f * sin_t;
        float i_q_f = -i_alpha_f * sin_t + i_beta_f * cos_t;

        float i_d_q = Q15_TO_AMP(i_d);
        float i_q_q = Q15_TO_AMP(i_q);

        /* 断言误差 < 0.15A */
        char msg[64];
        sprintf(msg, "Angle %d (%.4f rad): i_d error too large", i, test_angles[i]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15f, i_d_f, i_d_q, msg);

        sprintf(msg, "Angle %d (%.4f rad): i_q error too large", i, test_angles[i]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15f, i_q_f, i_q_q, msg);
    }
}

/* ============================================================================
 * 主函数
 * ============================================================================ */

int main(void) {
    UNITY_BEGIN();

    /* 参数初始化测试 */
    RUN_TEST(test_motor_params_q15_init);

    /* 电流预测精度测试 */
    RUN_TEST(test_motor_predict_current_q15_accuracy);

    /* Park 变换精度测试 */
    RUN_TEST(test_alphabeta_to_dq_q15_accuracy);

    /* 逆 Park 变换精度测试 */
    RUN_TEST(test_dq_to_alphabeta_q15_accuracy);

    /* Clarke 变换精度测试 */
    RUN_TEST(test_abc_to_alphabeta_q15_accuracy);

    /* 边界条件测试 */
    RUN_TEST(test_motor_model_q15_edge_cases);

    /* 死区补偿参考电压测试 */
    RUN_TEST(test_deadbeat_voltage_ref_q15);

    /* 不同角度 Park 变换测试 */
    RUN_TEST(test_park_transform_various_angles);

    return UNITY_END();
}
