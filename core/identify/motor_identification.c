#include <math.h>
#include "motor_identification.h"

/* ----------------------------------------------------------------------------
 * 本文件实现“参数辨识”：
 * 1) 电参数 Rs/Ld/Lq/psi_f（4维最小二乘）
 * 2) 机械参数 J/B（2维最小二乘）
 *
 * 实践建议：
 * - 电参数辨识使用高频电压激励，提升可观测性；
 * - 机械参数辨识使用扭矩激励，保证速度动态充分变化；
 * - 先电参数后机械参数，通常更稳定。
 * -------------------------------------------------------------------------- */

static int solve_linear_4x4(float a[4][4], float b[4], float x[4])
{
    /* 高斯消元（部分选主元）用于求解 ATA*x=ATb */
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        float max_abs = fabsf(a[col][col]);
        for (int r = col + 1; r < 4; ++r) {
            const float v = fabsf(a[r][col]);
            if (v > max_abs) {
                max_abs = v;
                pivot = r;
            }
        }

        if (max_abs < 1.0e-10f) {
            return -1;
        }

        if (pivot != col) {
            for (int c = col; c < 4; ++c) {
                const float tmp = a[col][c];
                a[col][c] = a[pivot][c];
                a[pivot][c] = tmp;
            }
            {
                const float tb = b[col];
                b[col] = b[pivot];
                b[pivot] = tb;
            }
        }

        {
            const float diag = a[col][col];
            for (int c = col; c < 4; ++c) {
                a[col][c] /= diag;
            }
            b[col] /= diag;
        }

        for (int r = 0; r < 4; ++r) {
            if (r == col) {
                continue;
            }
            const float f = a[r][col];
            for (int c = col; c < 4; ++c) {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }

    for (int i = 0; i < 4; ++i) {
        x[i] = b[i];
    }
    return 0;
}

int motor_identify_pmsm_params(const MotorIdSample *samples,
                               int sample_count,
                               MotorIdResult *out)
{
    float ata[4][4] = {{0}};
    float atb[4] = {0};

    if (samples == 0 || out == 0 || sample_count < 12) {
        return -1;
    }

    for (int k = 0; k < sample_count; ++k) {
        const MotorIdSample *s = &samples[k];

        /* 方程1: vd = Rs*id + Ld*did - Lq*(we*iq) */
        const float r1[4] = {
            s->id,
            s->did,
            -s->omega_e * s->iq,
            0.0f
        };

        /* 方程2: vq = Rs*iq + Ld*(we*id) + Lq*diq + psi_f*we */
        const float r2[4] = {
            s->iq,
            s->omega_e * s->id,
            s->diq,
            s->omega_e
        };

        for (int i = 0; i < 4; ++i) {
            atb[i] += r1[i] * s->vd + r2[i] * s->vq;
            for (int j = 0; j < 4; ++j) {
                ata[i][j] += r1[i] * r1[j] + r2[i] * r2[j];
            }
        }
    }

    {
        /* 解法：正规方程 */
        float x[4] = {0};
        float aa[4][4];
        float bb[4];
        for (int i = 0; i < 4; ++i) {
            bb[i] = atb[i];
            for (int j = 0; j < 4; ++j) {
                aa[i][j] = ata[i][j];
            }
        }

        if (solve_linear_4x4(aa, bb, x) != 0) {
            return -2;
        }

        out->Rs = x[0];
        out->Ld = x[1];
        out->Lq = x[2];
        out->psi_f = x[3];
    }

    {
        /* 计算电压拟合RMS误差，衡量辨识质量 */
        float se = 0.0f;
        int m = 0;
        for (int k = 0; k < sample_count; ++k) {
            const MotorIdSample *s = &samples[k];
            const float vd_hat = out->Rs * s->id + out->Ld * s->did - out->Lq * (s->omega_e * s->iq);
            const float vq_hat = out->Rs * s->iq + out->Ld * (s->omega_e * s->id)
                               + out->Lq * s->diq + out->psi_f * s->omega_e;
            const float evd = s->vd - vd_hat;
            const float evq = s->vq - vq_hat;
            se += evd * evd + evq * evq;
            m += 2;
        }
        out->used_samples = sample_count;
        out->rms_error = (m > 0) ? sqrtf(se / (float)m) : 0.0f;
    }

    return 0;
}

void motor_id_result_to_mpc_params(const MotorIdResult *id,
                                   float Ts,
                                   float Vdc,
                                   MotorParams *out)
{
    /* 当前MPC是单电感简化模型，故取 Ls = (Ld + Lq)/2 */
    out->Rs = id->Rs;
    out->Ls = 0.5f * (id->Ld + id->Lq);
    out->psi_f = id->psi_f;
    out->Ts = Ts;
    out->Vdc = Vdc;
}
