#include <stdio.h>
#include <math.h>

#include "motor_model.h"
#include "motor_control.h"
#include "motor_identification.h"
#include "inverter_compensation.h"
#include "adc_calibration.h"
#include "observers.h"
#include "safety_manager.h"
#include "pi_fallback.h"
#include "control_interface.h"
#include "control_hal.h"
#include "control_hal_sim.h"
#include "app_hal_utils.h"
#include "app_identification.h"
#include "motor_pu_profile.h"
#include "ui_panel.h"

/* ============================================================================
 * 文件：control_main.c
 * 作用：MPC 控制独立入口（主流程）
 *
 * 架构说明：
 * 1) 第一层（可选）：启动前电参数辨识；
 * 2) 第二层：MPC 电流环 + 补偿 + 观测器 + 安全降级；
 * 3) HAL 负责采样/执行，算法层不直接依赖芯片寄存器。
 * ========================================================================== */

/* 是否在控制启动前做一次在线电参数辨识：
 * - 1：控制前先辨识，再把结果注入 MPC；
 * - 0：直接使用手工参数。 */
#define ENABLE_MOTOR_IDENTIFICATION 0

/* 编码器参数（AB 正交，4 倍频） */
#define ENCODER_PPR 1024

/* 辨识样本数（仅当 ENABLE_MOTOR_IDENTIFICATION=1 时使用） */
#define ID_SAMPLE_COUNT 800

/* 常规运行控制循环步数（仿真演示） */
#define CTRL_STEPS_NORMAL 5000

/* 频域扫频测试开关：
 * 1=启用 iq 注入扫频并记录 FRF 数据；
 * 0=普通控制运行。 */
#define ENABLE_FRF_SWEEP 1

/* 频域扫频运行步数（建议远大于常规运行时长） */
#define CTRL_STEPS_FRF 24000

/* FRF 设定：
 * - 前 FRF_SETTLE_STEPS 为稳态段（不注入）；
 * - 后续做线性扫频注入。 */
#define FRF_SETTLE_STEPS 2000
#define FRF_FREQ_START_HZ 2.0f
#define FRF_FREQ_END_HZ 120.0f
#define FRF_IQ_INJ_AMP_A 0.80f

/* 控制台日志分频：每 N 拍打印一次 */
#define CONSOLE_LOG_DIV 50

/* 文件日志开关：1=输出文本日志与 CSV 波形 */
#define ENABLE_FILE_LOG 1

/* 实时 GUI 开关（gnuplot）：
 * 1=启用实时窗口；0=关闭。 */
#define ENABLE_UI 1

/* 实时 GUI 刷新分频（每 N 拍刷新一次窗口） */
#define UI_REFRESH_DIV 100

/* 外环调度分频：电流环每拍执行，速度环每 SPEED_LOOP_DIV 拍执行 */
#define SPEED_LOOP_DIV 10

/* 外环调度分频：位置环每 POS_LOOP_DIV 拍执行（应慢于速度环） */
#define POS_LOOP_DIV 20

/* 位置环参数（建议先用 P 或弱积分，避免低频漂移） */
#define POS_KP 18.0f
#define POS_KI 2.0f
#define POS_KAW 0.10f

/* 速度环抗积分饱和增益（无量纲） */
#define SPD_KAW 0.20f

/* 外环限幅：位置环输出速度上限、速度环输出电流上限 */
#define OMEGA_REF_LIMIT_PU 0.75f
#define IQ_REF_LIMIT_PU 1.00f

/* MPC 标幺化开关：
 * 说明：开启后 MPC 代价函数内部会先做“除基值”的归一化。 */
#define ENABLE_MPC_PER_UNIT 1

/* 选择电机标幺化模板：
 * 换电机时，优先在 motor_pu_profile.c 里改基值模板。 */
#define ACTIVE_MOTOR_PU_TEMPLATE MOTOR_PU_TEMPLATE_MID_48V

/* 速度环 PI 的“标幺化增益”，会自动映射到实参 Kp/Ki：
 * Kp_actual = SPD_KP_PU * (i_base / omega_base)
 * Ki_actual = SPD_KI_PU * (i_base / omega_base) */
#define SPD_KP_PU 1.50f
#define SPD_KI_PU 30.0f

/* 自动弱磁配置（基于基速自动进出）：
 * - enter：进入阈值 = 1.00 * omega_base
 * - exit：退出阈值 = 0.90 * omega_base（迟滞避免抖动切换） */
#define ENABLE_AUTO_FW 0
#define FW_ENTER_PU 1.00f
#define FW_EXIT_PU 0.90f

/* 负载转矩前馈开关：
 * 调试闭环基础行为时建议先关闭，避免前馈误差主导 iq_ref。 */
#define ENABLE_LOAD_TORQUE_FF 0

/* Rs 在线更新门控：
 * 只有在速度、电流都处于可信区间时才更新 Rs，避免异常采样拉偏模型。 */
#define ENABLE_RS_UPDATE_GATING 1

/* 位置命令（机械角，多圈角度） */
#define THETA_REF_MECH_RAD 6.0f

/* 位置误差限幅：多圈模式下防止误差过大导致速度指令冲击 */
#define THETA_ERR_MAX 12.0f

/* 手工输入参数：当不启用辨识时，MPC 采用这些参数 */
#define MANUAL_RS      0.45f
#define MANUAL_LS      0.00185f
#define MANUAL_PSI_F   0.060f
#define MANUAL_LD      0.00170f
#define MANUAL_LQ      0.00200f
#define MANUAL_J       0.00070f
#define MANUAL_B       0.00020f

int main(void)
{
    /* 仿真电机真实对象参数（实机时由真实电机决定） */
    PmsmParams plant_nominal = {
        .Rs = 0.45f,
        .Ld = 0.0017f,
        .Lq = 0.0020f,
        .psi_f = 0.060f,
        .pole_pairs = 4.0f,
        .J = 0.0007f,
        .B = 0.0002f,
        .Ts = 0.00005f,
        .Vdc = 48.0f
    };

    /* 电流采样标定参数（工程中可由在线标定更新） */
    CurrentSensorCalib adc_cal = {
        .adc_delay_s = 2.0e-6f,
        .lpf_gain = 0.001f,
        .offset_alpha = 0.22f,
        .offset_beta = -0.18f,
        .gain_alpha = 0.94f,
        .gain_beta = 1.07f
    };

    /* 绑定 HAL（当前使用仿真 HAL，后续可直接切换到 GD32 HAL 实现） */
    ControlHal hal;
    ControlHalSimCtx hal_sim;
    /* 仿真初速设为 0，便于观察从静止到目标位置的收敛过程 */
    control_hal_sim_init(&hal_sim, &plant_nominal, &adc_cal, 0.0f);
    control_hal_sim_bind(&hal, &hal_sim);

    /* MPC 当前工作参数（默认先放手工值） */
    MotorParams mpc_motor = {
        .Rs = MANUAL_RS,
        .Ls = MANUAL_LS,
        .psi_f = MANUAL_PSI_F,
        .Ts = plant_nominal.Ts,
        .Vdc = plant_nominal.Vdc
    };

#if ENABLE_MOTOR_IDENTIFICATION
    /* 电参数辨识结果缓存：仅在启用辨识模式时使用 */
    MotorIdResult id_res[HAL_MOTOR_COUNT];
#endif

#if ENABLE_MOTOR_IDENTIFICATION
    for (int m = 0; m < HAL_MOTOR_COUNT; ++m) {
        int rc = app_identify_electrical_via_hal(&hal,
                                                 m,
                                                 ENCODER_PPR,
                                                 plant_nominal.pole_pairs,
                                                 plant_nominal.Ts,
                                                 &adc_cal,
                                                 ID_SAMPLE_COUNT,
                                                 &id_res[m]);
        if (rc != 0) {
            printf("[ID] Motor%d electrical identification failed, rc=%d\n", m, rc);
            return 1;
        }

        printf("[ID] Motor%d elec: Rs=%.5f Ld=%.6f Lq=%.6f psi=%.5f | rms_v=%.6f\n",
               m,
               id_res[m].Rs,
               id_res[m].Ld,
               id_res[m].Lq,
               id_res[m].psi_f,
               id_res[m].rms_error);
    }

    /* 默认把电机 0 的辨识参数注入控制器 */
    motor_id_result_to_mpc_params(&id_res[0], plant_nominal.Ts, plant_nominal.Vdc, &mpc_motor);
#endif

    /* 读取当前电机标幺化基值模板：
     * 后续多数默认参数会基于这个基值自动换算。 */
    const MotorPuBase pu_base = motor_pu_template_get(ACTIVE_MOTOR_PU_TEMPLATE);
    /* 速度参考限幅（机械角速度，rad/s）= 标幺上限 * 机械角速度基值 */
    const float omega_ref_max = OMEGA_REF_LIMIT_PU * pu_base.omega_base;
    /* q轴电流参考限幅（A）= 标幺上限 * 电流基值 */
    const float iq_ref_max = IQ_REF_LIMIT_PU * pu_base.i_base;
    /* 速度环 Kp/Ki 从标幺增益映射到物理单位 */
    const float spd_k_scale = pu_base.i_base / (pu_base.omega_base + 1.0e-6f);
    const float spd_kp = SPD_KP_PU * spd_k_scale;
    const float spd_ki = SPD_KI_PU * spd_k_scale;
    printf("PU base: I=%.2fA V=%.2fV omega=%.2frad/s | limits: iq=%.2fA omega=%.2f\n",
           pu_base.i_base,
           pu_base.v_base,
           pu_base.omega_base,
           iq_ref_max,
           omega_ref_max);

    /* ===========================
     * MPC 控制阶段（当前示例只控电机0）
     * =========================== */
    /* 标幺化后默认代价权重（无量纲）：
     * - w_current：电流跟踪主权重
     * - w_switch：开关惩罚（抑制抖动）
     * - w_voltage：电压参考逼近权重 */
    MpcWeights weights = {1.0f, 0.020f, 0.060f, 20.0f};
    MpcController mpc;
    mpc_controller_init(&mpc, &mpc_motor, &weights, 0, 0, 0);
    /* 启用“预筛选二步预测”：Top-K=2，第二步权重=1.0 */
    mpc_controller_set_two_step(&mpc, 1, 2, 1.0f);
    /* 启用 MPC 标幺化：电流/电压误差先归一化，再进入代价函数 */
    mpc_controller_set_per_unit(&mpc,
                                ENABLE_MPC_PER_UNIT,
                                pu_base.i_base,
                                pu_base.v_base);

    /* MPC 失效时的 PI 回退控制器 */
    PiCurrentController pi_backup;
    pi_current_controller_init(&pi_backup, 4.0f, 900.0f, 20.0f);

    /* 逆变器死区与器件压降补偿参数 */
    InverterCompConfig comp_cfg;
    comp_cfg.pwm_period_s = plant_nominal.Ts;
    comp_cfg.deadtime_s = 1.2e-6f;
    comp_cfg.v_diode = 0.8f;
    comp_cfg.r_on = 0.03f;
    comp_cfg.current_epsilon = 0.05f;

    /* 扰动观测器（用于总扰动估计与前馈补偿） */
    DisturbanceObserver dob;
    disturbance_observer_init(&dob, mpc.a, mpc.b, 0.12f, 0.02f);

    /* 负载转矩观测器（速度环前馈可用） */
    LoadTorqueObserver tl_obs;
    load_torque_observer_init(&tl_obs, MANUAL_J, MANUAL_B, 0.05f);

    /* Rs 温漂在线自适应参数 */
    RsAdaptiveConfig rs_cfg;
    rs_cfg.gamma = 5.0e-5f;
    rs_cfg.rs_min = 0.20f;
    rs_cfg.rs_max = 0.90f;

    /* 编码器速度滤波与位置插值 */
    SpeedPosEstimator sp_est;
    speed_pos_estimator_init(&sp_est, 0.12f);

    /* 保护阈值与降级规则 */
    SafetyConfig safety_cfg;
    safety_cfg.i_limit = 1.6f * pu_base.i_base;
    safety_cfg.vdc_limit = 62.0f;
    safety_cfg.stall_speed_min = 8.0f;
    safety_cfg.stall_count_limit = 60;
    safety_cfg.cost_limit = 50.0f;

    SafetyState safety;
    safety_init(&safety);

    /* 控制接口对象：统一管理“位置/速度/电流”模式与目标 */
    ControlInterface ctrl_if;
    /* 控制接口配置：把当前工程参数集中喂给接口层 */
    ControlInterfaceConfig ctrl_cfg;
    ctrl_cfg.omega_ref_max = omega_ref_max;
    ctrl_cfg.iq_ref_max = iq_ref_max;
    ctrl_cfg.theta_err_max = THETA_ERR_MAX;
    ctrl_cfg.speed_loop_div = SPEED_LOOP_DIV;
    ctrl_cfg.pos_loop_div = POS_LOOP_DIV;
    ctrl_cfg.id_slew_step = 0.06f;
    ctrl_cfg.iq_slew_step = 0.06f;
    ctrl_cfg.pos_kp = POS_KP;
    ctrl_cfg.pos_ki = POS_KI;
    ctrl_cfg.pos_kaw = POS_KAW;
    ctrl_cfg.spd_kp = spd_kp;
    ctrl_cfg.spd_ki = spd_ki;
    ctrl_cfg.spd_kaw = SPD_KAW;
    /* 自动弱磁参数：超过基速进入弱磁，降低反电势扩展高速能力 */
    ctrl_cfg.fw_enable_auto = ENABLE_AUTO_FW;
    ctrl_cfg.omega_fw_enter = FW_ENTER_PU * pu_base.omega_base;
    ctrl_cfg.omega_fw_exit = FW_EXIT_PU * pu_base.omega_base;
    ctrl_cfg.id_fw_min = -0.50f * iq_ref_max;
    ctrl_cfg.id_fw_gain = (0.50f * iq_ref_max) / (0.30f * pu_base.omega_base + 1.0e-6f);
    ctrl_cfg.id_fw_lpf = 0.08f;
    control_interface_init(&ctrl_if, &ctrl_cfg);
    /* 默认控制模式：
     * - FRF 模式：速度模式 + 零速基线（在 iq_ref 上叠加扫频注入）；
     * - 普通模式：位置模式。 */
#if ENABLE_FRF_SWEEP
    control_interface_set_speed_target(&ctrl_if, 0.0f);
#else
    control_interface_set_position_target(&ctrl_if, THETA_REF_MECH_RAD);
#endif

    EncoderStateEst enc_run = {0};
    float id_ref_cmd = 0.0f;
    float iq_ref_cmd = 0.0f;
    float omega_ref_dbg = 0.0f;
    ControlMode mode_dbg = CTRL_MODE_POSITION;
    float id_fw_dbg = 0.0f;
    int fw_active_dbg = 0;
    float theta_mech_dbg = 0.0f;
    float theta_err_dbg = 0.0f;
    float omega_err_dbg = 0.0f;

    Vec2 v_ab_applied_prev = {0.0f, 0.0f};
    float mpc_cost_last = 0.0f;
    CurrentState i_dq_prev_for_rs = {0.0f, 0.0f};
    float tl_hat_last = 0.0f;
    float frf_iq_inj = 0.0f;
    float frf_freq_hz = 0.0f;
    float frf_phase = 0.0f;
    FILE *log_txt = 0;
    FILE *log_csv = 0;
    UiPanel ui_panel;
    const int run_steps = ENABLE_FRF_SWEEP ? CTRL_STEPS_FRF : CTRL_STEPS_NORMAL;

    printf("Run start: HAL loop (motor0 controlled, motor1 idle)\n");
    printf("Run config: steps=%d, Ts=%.8f, total_time=%.4fs, frf=%d\n",
           run_steps,
           plant_nominal.Ts,
           (float)run_steps * plant_nominal.Ts,
           ENABLE_FRF_SWEEP);

#if ENABLE_FILE_LOG
    log_txt = fopen("build/control_debug.txt", "w");
    log_csv = fopen("build/control_wave.csv", "w");
    if (log_txt != 0) {
        fprintf(log_txt, "MPC control debug log\n");
        fprintf(log_txt, "steps=%d Ts=%.8f total_time=%.6f\n",
                run_steps,
                plant_nominal.Ts,
                (float)run_steps * plant_nominal.Ts);
        fprintf(log_txt, "PU base: I=%.2f V=%.2f omega=%.2f\n",
                pu_base.i_base,
                pu_base.v_base,
                pu_base.omega_base);
        fprintf(log_txt, "FRF: enable=%d settle=%d f_start=%.3fHz f_end=%.3fHz amp=%.3fA\n",
                ENABLE_FRF_SWEEP,
                FRF_SETTLE_STEPS,
                FRF_FREQ_START_HZ,
                FRF_FREQ_END_HZ,
                FRF_IQ_INJ_AMP_A);
    }
    if (log_csv != 0) {
        fprintf(log_csv,
                "k,time_s,safety_mode,ctrl_mode,fw,vdc,rs,wm,omega_ref,id_ref,iq_ref,id_fw,"
                "theta_ref,theta_mech,epos_live,epos_pi,espd,i_alpha,i_beta,tl_hat,mpcJ,enc,"
                "iq_inj,frf_freq_hz\n");
    }
    printf("Log files: build/control_debug.txt, build/control_wave.csv\n");
#endif

#if ENABLE_UI
    ui_panel_init(&ui_panel, "build/control_wave.csv", UI_REFRESH_DIV);
    if (ui_panel.enabled) {
        printf("UI enabled via gnuplot (refresh_div=%d)\n", UI_REFRESH_DIV);
    } else {
        printf("UI disabled: gnuplot not available.\n");
    }
    printf("UI debug log: build/ui_debug.txt\n");
#else
    ui_panel.enabled = 0;
#endif

    for (int k = 0; k < run_steps; ++k) {
        HalFeedback fb;
        unsigned hal_faults = 0;
        HalPwmCmd pwm;

        hal.read_feedback(hal.ctx, &fb);
        hal.read_fault_flags(hal.ctx, &hal_faults);

        app_encoder_estimator_update(&enc_run,
                                     fb.motor[0].enc_ab.count,
                                     ENCODER_PPR,
                                     plant_nominal.pole_pairs,
                                     plant_nominal.Ts);

        speed_pos_estimator_update(&sp_est,
                                   enc_run.omega_m,
                                   plant_nominal.Ts,
                                   plant_nominal.pole_pairs);

        {
            /* 控制接口反馈：位置模式/速度模式都需要机械角和机械角速度 */
            ControlFeedback ctrl_fb;
            /* 控制接口输出：统一给电流环的 id/iq 参考 */
            ControlRefOutput ctrl_out;
            /* 机械角位置（多圈角） */
            const float theta_mech = app_encoder_count_to_mech_rad(fb.motor[0].enc_ab.count, ENCODER_PPR);
            /* 力矩常数近似：Kt = 1.5 * pp * psi_f（忽略凸极磁阻力矩） */
            const float kt = 1.5f * plant_nominal.pole_pairs * (mpc.motor.psi_f + 1.0e-6f);
            /* 负载转矩观测 -> q轴前馈电流 */
            float iq_ff = tl_hat_last / kt;
            /* 前馈限幅，避免观测抖动导致电流命令突变 */
            iq_ff = app_clampf(iq_ff, -0.4f * iq_ref_max, 0.4f * iq_ref_max);
#if !ENABLE_LOAD_TORQUE_FF
            iq_ff = 0.0f;
#endif

            ctrl_fb.theta_mech = theta_mech;
            ctrl_fb.omega_mech = sp_est.omega_filt;
            ctrl_fb.iq_feedforward = iq_ff;
            ctrl_fb.Ts = plant_nominal.Ts;
            control_interface_step(&ctrl_if, &ctrl_fb, &ctrl_out);

            id_ref_cmd = ctrl_out.id_ref;
            iq_ref_cmd = ctrl_out.iq_ref;
            omega_ref_dbg = ctrl_out.omega_ref_dbg;
            mode_dbg = ctrl_out.mode_dbg;
            id_fw_dbg = ctrl_out.id_fw_dbg;
            fw_active_dbg = ctrl_out.fw_active_dbg;
            theta_mech_dbg = theta_mech;
            theta_err_dbg = ctrl_out.theta_err_dbg;
            omega_err_dbg = ctrl_out.omega_err_dbg;

#if ENABLE_FRF_SWEEP
            {
                /* FRF 注入逻辑：
                 * 在速度模式下对 iq_ref 叠加小幅扫频信号，记录响应。 */
                const int sweep_steps = run_steps - FRF_SETTLE_STEPS;
                if (k >= FRF_SETTLE_STEPS && sweep_steps > 10) {
                    const float p = (float)(k - FRF_SETTLE_STEPS) / (float)(sweep_steps - 1);
                    frf_freq_hz = FRF_FREQ_START_HZ + (FRF_FREQ_END_HZ - FRF_FREQ_START_HZ) * p;
                    frf_phase += 2.0f * 3.14159265359f * frf_freq_hz * plant_nominal.Ts;
                    if (frf_phase > 6.28318530718f) frf_phase -= 6.28318530718f;
                    frf_iq_inj = FRF_IQ_INJ_AMP_A * sinf(frf_phase);
                } else {
                    frf_freq_hz = 0.0f;
                    frf_iq_inj = 0.0f;
                }
                iq_ref_cmd = app_clampf(iq_ref_cmd + frf_iq_inj, -iq_ref_max, iq_ref_max);
            }
#else
            frf_freq_hz = 0.0f;
            frf_iq_inj = 0.0f;
#endif
        }

        {
            const Vec2 iref_ab = dq_to_alphabeta(id_ref_cmd, iq_ref_cmd, sp_est.theta_interp);
            CurrentRef r_ab = {iref_ab.x, iref_ab.y};

            /* 采样校准 + ADC 时延补偿 */
            const CurrentState raw = {fb.motor[0].i_alpha_raw, fb.motor[0].i_beta_raw};
            CurrentState i_cal = current_sensor_apply_calib(&raw, &adc_cal);
            i_cal = adc_delay_compensation(&i_cal,
                                           &v_ab_applied_prev,
                                           mpc.motor.Rs,
                                           mpc.motor.Ls,
                                           adc_cal.adc_delay_s);

            /* 扰动观测与母线电压前馈更新 */
            disturbance_observer_update(&dob, &i_cal, &v_ab_applied_prev);
            mpc_controller_update_vdc(&mpc, fb.vdc);

            /* 保护状态机评估 */
            safety_update(&safety,
                          &safety_cfg,
                          i_cal.i_alpha,
                          i_cal.i_beta,
                          fb.vdc,
                          sp_est.omega_filt,
                          mpc_cost_last);

            /* 底层 HAL 故障与控制故障统一接入保护状态机 */
            if (hal_faults != HAL_FAULT_NONE) {
                safety.mode = SAFETY_MODE_FAULT;
                safety.cause = SAFETY_CAUSE_MPC_DIVERGED;
            }

            {
                Vec2 v_cmd_ab = {0.0f, 0.0f};

                if (safety.mode == SAFETY_MODE_NORMAL) {
                    const MpcOutput u = mpc_controller_step(&mpc, &i_cal, &r_ab);
                    v_cmd_ab.x = u.v_alpha;
                    v_cmd_ab.y = u.v_beta;
                    mpc_cost_last = u.cost;
                } else if (safety.mode == SAFETY_MODE_DEGRADED_PI) {
                    /* 降级模式：MPC 不可用时切 PI 保底 */
                    v_cmd_ab = pi_current_step(&pi_backup, &r_ab, &i_cal, plant_nominal.Ts);
                    mpc_cost_last = 0.0f;
                }

                /* 扰动前馈补偿 */
                v_cmd_ab.x -= dob.d_hat.x;
                v_cmd_ab.y -= dob.d_hat.y;

                {
                    /* 死区与器件压降补偿后再做电压限幅 */
                    const Vec2 v_comp = inverter_compensate_voltage_ab(&v_cmd_ab,
                                                                        &i_cal,
                                                                        fb.vdc,
                                                                        &comp_cfg);
                    const float v_alpha_lim = (2.0f / 3.0f) * fb.vdc;
                    const float v_beta_lim = 0.57735026919f * fb.vdc;
                    v_ab_applied_prev.x = app_clampf(v_comp.x, -v_alpha_lim, v_alpha_lim);
                    v_ab_applied_prev.y = app_clampf(v_comp.y, -v_beta_lim, v_beta_lim);
                }
            }

            {
                /* Rs 在线更新：依赖 dq 电流导数与电压模型残差 */
                const Vec2 i_dq = alphabeta_to_dq(i_cal.i_alpha, i_cal.i_beta, enc_run.theta_e);
                const Vec2 v_dq = alphabeta_to_dq(v_ab_applied_prev.x, v_ab_applied_prev.y, enc_run.theta_e);
                const float we = plant_nominal.pole_pairs * enc_run.omega_m;
                const float did = (i_dq.x - i_dq_prev_for_rs.i_alpha) / plant_nominal.Ts;
                const float diq = (i_dq.y - i_dq_prev_for_rs.i_beta) / plant_nominal.Ts;

#if ENABLE_MOTOR_IDENTIFICATION
                const float psi_use = id_res[0].psi_f;
                const float ld_use = id_res[0].Ld;
                const float lq_use = id_res[0].Lq;
#else
                const float psi_use = MANUAL_PSI_F;
                const float ld_use = MANUAL_LD;
                const float lq_use = MANUAL_LQ;
#endif

                const float torque_e = 1.5f * plant_nominal.pole_pairs
                                     * (psi_use * i_dq.y + (ld_use - lq_use) * i_dq.x * i_dq.y);
                tl_hat_last = load_torque_observer_update(&tl_obs,
                                                          enc_run.omega_m,
                                                          torque_e,
                                                          plant_nominal.Ts);

                {
                    const float rs_prev = mpc.motor.Rs;
                    const float rs_next = rs_adaptation_update(rs_prev,
                                                               &rs_cfg,
                                                               v_dq.x,
                                                               v_dq.y,
                                                               i_dq.x,
                                                               i_dq.y,
                                                               did,
                                                               diq,
                                                               we,
                                                               ld_use,
                                                               lq_use,
                                                               psi_use);
#if ENABLE_RS_UPDATE_GATING
                    const float w_abs = fabsf(enc_run.omega_m);
                    const float i_abs = fabsf(i_dq.x) + fabsf(i_dq.y);
                    const float iq_abs = fabsf(iq_ref_cmd);
                    const int rs_update_ok =
                        (w_abs < 1.20f * omega_ref_max) &&
                        (i_abs < 1.80f * iq_ref_max) &&
                        (iq_abs < 0.95f * iq_ref_max) &&
                        (safety.mode == SAFETY_MODE_NORMAL);
                    mpc.motor.Rs = rs_update_ok ? rs_next : rs_prev;
#else
                    mpc.motor.Rs = rs_next;
#endif
                }

                /* Rs 变化后要同步重算离散模型系数 a/b */
                motor_model_discrete_coeff(&mpc.motor, &mpc.a, &mpc.b);
                i_dq_prev_for_rs.i_alpha = i_dq.x;
                i_dq_prev_for_rs.i_beta = i_dq.y;
            }

            /* 将目标 alpha-beta 电压写成三相 PWM 占空比并下发 */
            app_build_pwm_cmd_for_motor(&pwm,
                                        0,
                                        v_ab_applied_prev.x,
                                        v_ab_applied_prev.y,
                                        fb.vdc,
                                        1);
            hal.write_pwm_cmd(hal.ctx, &pwm);
            hal.wait_next_cycle(hal.ctx);

            {
                const float epos_live = ctrl_if.cmd.theta_ref_cmd - theta_mech_dbg;

#if ENABLE_FILE_LOG
                if (log_csv != 0) {
                    fprintf(log_csv,
                            "%d,%.8f,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%ld,%.6f,%.6f\n",
                            k,
                            fb.time_s,
                            (int)safety.mode,
                            (int)mode_dbg,
                            fw_active_dbg,
                            fb.vdc,
                            mpc.motor.Rs,
                            enc_run.omega_m,
                            omega_ref_dbg,
                            id_ref_cmd,
                            iq_ref_cmd,
                            id_fw_dbg,
                            ctrl_if.cmd.theta_ref_cmd,
                            theta_mech_dbg,
                            epos_live,
                            theta_err_dbg,
                            omega_err_dbg,
                            i_cal.i_alpha,
                            i_cal.i_beta,
                            tl_hat_last,
                            mpc_cost_last,
                            (long)fb.motor[0].enc_ab.count,
                            frf_iq_inj,
                            frf_freq_hz);
                }
#endif

#if ENABLE_UI
                ui_panel_refresh(&ui_panel, k);
#endif

                if ((k % CONSOLE_LOG_DIV) == 0 || safety.mode != SAFETY_MODE_NORMAL) {
                    printf("k=%04d mode=%d ctrl_mode=%d fw=%d Rs=%.4f Vdc=%.2f | i=(%.3f,%.3f) wm=%.2f | "
                           "th_ref=%.2f th=%.2f epos_live=%.2f epos_pi=%.2f espd=%.2f | "
                           "w_ref=%.2f id_ref=%.2f iq_ref=%.2f id_fw=%.2f Tl=%.3f | enc=%ld | mpcJ=%.4f\n",
                           k,
                           (int)safety.mode,
                           (int)mode_dbg,
                           fw_active_dbg,
                           mpc.motor.Rs,
                           fb.vdc,
                           i_cal.i_alpha,
                           i_cal.i_beta,
                           enc_run.omega_m,
                           ctrl_if.cmd.theta_ref_cmd,
                           theta_mech_dbg,
                           epos_live,
                           theta_err_dbg,
                           omega_err_dbg,
                           omega_ref_dbg,
                           id_ref_cmd,
                           iq_ref_cmd,
                           id_fw_dbg,
                           tl_hat_last,
                           (long)fb.motor[0].enc_ab.count,
                           mpc_cost_last);
                }

#if ENABLE_FILE_LOG
                if (log_txt != 0 && ((k % CONSOLE_LOG_DIV) == 0 || safety.mode != SAFETY_MODE_NORMAL)) {
                    fprintf(log_txt,
                            "k=%04d mode=%d ctrl_mode=%d fw=%d Rs=%.4f Vdc=%.2f | i=(%.3f,%.3f) wm=%.2f | "
                            "th_ref=%.2f th=%.2f epos_live=%.2f epos_pi=%.2f espd=%.2f | "
                            "w_ref=%.2f id_ref=%.2f iq_ref=%.2f id_fw=%.2f Tl=%.3f | enc=%ld | mpcJ=%.4f\n",
                            k,
                            (int)safety.mode,
                            (int)mode_dbg,
                            fw_active_dbg,
                            mpc.motor.Rs,
                            fb.vdc,
                            i_cal.i_alpha,
                            i_cal.i_beta,
                            enc_run.omega_m,
                            ctrl_if.cmd.theta_ref_cmd,
                            theta_mech_dbg,
                            epos_live,
                            theta_err_dbg,
                            omega_err_dbg,
                            omega_ref_dbg,
                            id_ref_cmd,
                            iq_ref_cmd,
                            id_fw_dbg,
                            tl_hat_last,
                            (long)fb.motor[0].enc_ab.count,
                            mpc_cost_last);
                }
                if ((k % 200) == 0) {
                    if (log_txt != 0) fflush(log_txt);
                    if (log_csv != 0) fflush(log_csv);
                }
#endif
            }

            if (safety.mode == SAFETY_MODE_FAULT) {
                printf("FAULT stop: cause=%d at k=%d\n", (int)safety.cause, k);
                break;
            }
        }
    }

#if ENABLE_FILE_LOG
    if (log_txt != 0) fclose(log_txt);
    if (log_csv != 0) fclose(log_csv);
    printf("Logs saved: build/control_debug.txt, build/control_wave.csv\n");
#endif

#if ENABLE_UI
    ui_panel_close(&ui_panel);
#endif

    return 0;
}
