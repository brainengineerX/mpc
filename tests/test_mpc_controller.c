/**
 * @file test_mpc_controller.c
 * @brief MPC 控制器单元测试
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../core/control/fixed_point_types.h"
#include "../core/control/mpc_controller_fixed.h"
#include "../core/control/motor_model_fixed.h"

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

TEST(mpc_controller_init) {
    /* 测试控制器初始化 */
    MpcControllerQ15 ctrl;
    MotorParamsQ15 motor_params;
    MpcWeightsQ15 weights;
    SwitchState init_switch = {0, 0, 0};

    /* 初始化电机参数 */
    memset(&motor_params, 0, sizeof(motor_params));
    motor_params.a_coeff = FLOAT_TO_Q30(0.95f);
    motor_params.b_coeff = FLOAT_TO_Q30(0.05f);
    motor_params.v_dc = VOLT_TO_Q15(48.0f);

    /* 初始化权重 */
    memset(&weights, 0, sizeof(weights));
    weights.w_current = FLOAT_TO_Q15(0.8f);
    weights.w_switch = FLOAT_TO_Q15(0.1f);
    weights.w_voltage = FLOAT_TO_Q15(0.1f);

    /* 测试初始化 */
    mpc_controller_q15_init(&ctrl, &motor_params, &weights, &init_switch);

    /* 验证初始化结果 */
    ASSERT_INT_EQ(ctrl.topk_cfg.k_steady, MPC_TOP_K_STEADY);
    ASSERT_INT_EQ(ctrl.topk_cfg.k_dynamic, MPC_TOP_K_DYNAMIC);
    ASSERT_INT_EQ(ctrl.topk_cfg.err_threshold, MPC_ERR_THRESHOLD_Q15);
    ASSERT_INT_EQ(ctrl.two_step_enable, 1);
    ASSERT_INT_EQ(ctrl.step2_weight, MPC_STEP2_WEIGHT_Q15);
    ASSERT_INT_EQ(ctrl.i_prev.i_alpha, 0);
    ASSERT_INT_EQ(ctrl.i_prev.i_beta, 0);

    return 1;
}

TEST(mpc_controller_set_topk) {
    /* 测试 Top-K 配置 */
    MpcControllerQ15 ctrl;

    /* 先初始化一个基础控制器 */
    memset(&ctrl, 0, sizeof(ctrl));

    /* 设置 Top-K 参数 */
    mpc_controller_q15_set_topk(&ctrl, 2, 6, 2048);

    /* 验证设置结果 */
    ASSERT_INT_EQ(ctrl.topk_cfg.k_steady, 2);
    ASSERT_INT_EQ(ctrl.topk_cfg.k_dynamic, 6);
    ASSERT_INT_EQ(ctrl.topk_cfg.err_threshold, 2048);
    ASSERT_INT_EQ(ctrl.topk_cfg.k_current, 2);  /* k_current 应该初始化为 k_steady */

    return 1;
}

TEST(mpc_controller_update_vdc) {
    /* 测试直流母线电压更新 */
    MpcControllerQ15 ctrl;
    q15_t new_vdc;

    /* 初始化控制器 */
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.vdc_measured = VOLT_TO_Q15(48.0f);
    ctrl.motor_params.v_dc = VOLT_TO_Q15(48.0f);

    /* 更新母线电压到 60V */
    new_vdc = VOLT_TO_Q15(60.0f);
    mpc_controller_q15_update_vdc(&ctrl, new_vdc);

    /* 验证更新结果 */
    ASSERT_INT_EQ(ctrl.vdc_measured, new_vdc);
    ASSERT_INT_EQ(ctrl.motor_params.v_dc, new_vdc);

    return 1;
}

TEST(mpc_controller_single_step) {
    /* 测试单步 MPC 控制 */
    MpcControllerQ15 ctrl;
    MotorParamsQ15 motor_params;
    MpcWeightsQ15 weights;
    SwitchState init_switch = {0, 0, 0};
    CurrentStateQ15 x_now;
    CurrentRefQ15 i_ref;
    SwitchState result;

    /* 初始化电机参数 */
    memset(&motor_params, 0, sizeof(motor_params));
    motor_params.a_coeff = FLOAT_TO_Q30(0.95f);
    motor_params.b_coeff = FLOAT_TO_Q30(0.05f);
    motor_params.v_dc = VOLT_TO_Q15(48.0f);

    /* 初始化权重 */
    memset(&weights, 0, sizeof(weights));
    weights.w_current = FLOAT_TO_Q15(0.8f);
    weights.w_switch = FLOAT_TO_Q15(0.1f);
    weights.w_voltage = FLOAT_TO_Q15(0.1f);

    /* 初始化控制器 */
    mpc_controller_q15_init(&ctrl, &motor_params, &weights, &init_switch);

    /* 设置当前电流状态 (5A, 0A) */
    x_now.i_alpha = AMP_TO_Q15(5.0f);
    x_now.i_beta = 0;

    /* 设置参考电流 (8A, 0A) */
    i_ref.i_alpha_ref = AMP_TO_Q15(8.0f);
    i_ref.i_beta_ref = 0;

    /* 执行单步 MPC 控制 */
    result = mpc_controller_q15_step(&ctrl, &x_now, &i_ref);

    /* 验证结果 - 开关状态应该是有效的 (0 或 1) */
    ASSERT(result.sa == 0 || result.sa == 1);
    ASSERT(result.sb == 0 || result.sb == 1);
    ASSERT(result.sc == 0 || result.sc == 1);

    return 1;
}

TEST(mpc_controller_two_stage) {
    /* 测试两步预测 MPC 控制 */
    MpcControllerQ15 ctrl;
    MotorParamsQ15 motor_params;
    MpcWeightsQ15 weights;
    SwitchState init_switch = {0, 0, 0};
    CurrentStateQ15 x_now;
    CurrentRefQ15 i_ref;
    SwitchState result;

    /* 初始化电机参数 */
    memset(&motor_params, 0, sizeof(motor_params));
    motor_params.a_coeff = FLOAT_TO_Q30(0.95f);
    motor_params.b_coeff = FLOAT_TO_Q30(0.05f);
    motor_params.v_dc = VOLT_TO_Q15(48.0f);

    /* 初始化权重 */
    memset(&weights, 0, sizeof(weights));
    weights.w_current = FLOAT_TO_Q15(0.8f);
    weights.w_switch = FLOAT_TO_Q15(0.1f);
    weights.w_voltage = FLOAT_TO_Q15(0.1f);

    /* 初始化控制器 */
    mpc_controller_q15_init(&ctrl, &motor_params, &weights, &init_switch);

    /* 启用两步预测 */
    ctrl.two_step_enable = 1;

    /* 设置当前电流状态 (3A, 2A) */
    x_now.i_alpha = AMP_TO_Q15(3.0f);
    x_now.i_beta = AMP_TO_Q15(2.0f);

    /* 设置参考电流 (6A, 4A) */
    i_ref.i_alpha_ref = AMP_TO_Q15(6.0f);
    i_ref.i_beta_ref = AMP_TO_Q15(4.0f);

    /* 执行两步预测 MPC 控制 */
    result = mpc_controller_q15_step_two_stage(&ctrl, &x_now, &i_ref);

    /* 验证结果 - 开关状态应该是有效的 (0 或 1) */
    ASSERT(result.sa == 0 || result.sa == 1);
    ASSERT(result.sb == 0 || result.sb == 1);
    ASSERT(result.sc == 0 || result.sc == 1);

    /* 验证 last_switch 被更新 */
    ASSERT(ctrl.last_switch.sa == result.sa);
    ASSERT(ctrl.last_switch.sb == result.sb);
    ASSERT(ctrl.last_switch.sc == result.sc);

    return 1;
}

/* ============================================================================
 * 主函数
 ============================================================================ */

int main(void) {
    printf("========================================\n");
    printf("  MPC Controller Unit Tests\n");
    printf("========================================\n\n");

    RUN_TEST(mpc_controller_init);
    RUN_TEST(mpc_controller_set_topk);
    RUN_TEST(mpc_controller_update_vdc);
    RUN_TEST(mpc_controller_single_step);
    RUN_TEST(mpc_controller_two_stage);

    printf("\n========================================\n");
    printf("  Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");

    return (tests_failed == 0) ? 0 : 1;
}
