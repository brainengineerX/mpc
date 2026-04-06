#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 模块名：motor_model
 * 职责：提供“电机数学模型”相关的纯算法接口，不依赖任何硬件外设。
 *
 * 为什么单独做这个模块：
 * 1) 控制器（MPC、PI、观测器）都要调用模型方程，集中管理可避免重复代码；
 * 2) 便于后续替换模型（例如从PMSM切换到感应电机模型）；
 * 3) 便于做“控制模型”和“真实对象模型”分离（控制器内简化，仿真对象完整）。
 *
 * 本模块包含两层模型：
 * A. 完整 PMSM dq + 机械模型（用于真实对象仿真和辨识）
 * B. 简化 alpha-beta 电流离散模型（用于电流环 MPC 快速预测）
 * ========================================================================== */

/* ===========================
 * 通用二维向量（alpha-beta / d-q）
 * =========================== */
typedef struct {
    float x;
    float y;
} Vec2;

/* ===========================
 * MPC 电流环简化参数（保留给控制器）
 * =========================== */
typedef struct {
    float Rs;      /* 定子电阻 */
    float Ls;      /* 等效电感（MPC 内核使用） */
    float psi_f;   /* 永磁体磁链 */
    float Ts;      /* 控制周期 */
    float Vdc;     /* 直流母线电压 */
} MotorParams;

typedef struct {
    float i_alpha;
    float i_beta;
} CurrentState;

typedef struct {
    float i_alpha;
    float i_beta;
} CurrentRef;

/* ===========================
 * 完整 PMSM 数学模型参数
 * ===========================
 * 电磁方程（dq）：
 *   did/dt = (vd - Rs*id + we*Lq*iq) / Ld
 *   diq/dt = (vq - Rs*iq - we*(Ld*id + psi_f)) / Lq
 * 转矩方程：
 *   Te = 1.5*p*(psi_f*iq + (Ld-Lq)*id*iq)
 * 机械方程：
 *   domega_m/dt = (Te - Tl - B*omega_m) / J
 *   dtheta_e/dt = we = p*omega_m
 *
 * 变量说明：
 * - Rs/Ld/Lq/psi_f：电磁参数
 * - pole_pairs：极对数 p
 * - J/B：机械转动惯量与粘性阻尼
 * - Ts：离散控制周期
 * - Vdc：母线电压（供限幅或逆变器模型使用）
 */
typedef struct {
    float Rs;
    float Ld;
    float Lq;
    float psi_f;

    float pole_pairs;
    float J;
    float B;

    float Ts;
    float Vdc;
} PmsmParams;

typedef struct {
    float id;
    float iq;
    float theta_e;
    float omega_m;
} PmsmState;

/* ===== 坐标变换 ===== */
/* dq -> alpha-beta（逆Park变换）
 * 输入：旋转坐标系的 d/q 分量 + 电角度 theta_e
 * 输出：静止坐标系 alpha/beta 分量 */
Vec2 dq_to_alphabeta(float d, float q, float theta_e);
/* alpha-beta -> dq（Park变换） */
Vec2 alphabeta_to_dq(float alpha, float beta, float theta_e);

/* ===== 完整 PMSM 模型 ===== */
/* 电角速度 we = p * omega_m */
float pmsm_electrical_speed(const PmsmParams *p, const PmsmState *x);
/* 电磁转矩方程 Te */
float pmsm_electromagnetic_torque(const PmsmParams *p, const PmsmState *x);
/* 完整对象一步离散更新（前向欧拉）：
 * 输入 dq 电压与负载转矩，输出下一拍状态 */
PmsmState pmsm_step_dq(const PmsmParams *p,
                       const PmsmState *x,
                       float vd, float vq,
                       float load_torque);

/* ===== MPC 简化模型（用于控制器内部） ===== */
/* 由 Rs/Ls/Ts 计算离散系数 a,b：
 * i(k+1)=a*i(k)+b*v(k) */
void motor_model_discrete_coeff(const MotorParams *motor, float *a, float *b);
/* 简化模型一步预测 */
CurrentState motor_model_predict_next(const CurrentState *x,
                                      float v_alpha, float v_beta,
                                      float a, float b);
/* deadbeat 反推理想电压参考并限幅 */
void motor_model_deadbeat_voltage_ref(const CurrentState *x,
                                      const CurrentRef *r,
                                      float a, float b,
                                      float vdc,
                                      float *v_alpha_ref,
                                      float *v_beta_ref);
/* 简化对象一步更新（用于轻量级仿真） */
CurrentState motor_model_plant_step(const CurrentState *x,
                                    float v_alpha, float v_beta,
                                    float disturbance_alpha,
                                    float disturbance_beta,
                                    float a, float b);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MODEL_H */
