/**
 * identify_dead.c — 死区补偿 LUT 辨识（移植 AC_Control DeadTime_Identify_Step）
 *
 * 子 FSM：D_INIT → D_WAIT(撤压 0.3s) → D_RAMP(升 vd 至 |I|≥目标 或达电压限)
 *         → D_HOLD(0.1s) → D_SAMPLE(0.05s 平均 |I|,vd) → D_NEXT(×20) → D_DONE/D_FAIL
 * 输出：ctx->res.dead_lut_I/V（Params_Update 提交到 DeadComp_Lut_I/V + DeadComp_En=1）
 * R 取 Motor_Params.Phase_R（上次 R 辨识值）；电压限用 VmaxCoeff（模型实际限幅值，
 * 保证 vd_ref = 实际施加，避免 VoltageLimiter 截顶失真）。
 */
#include "identify.h"
#include "identify_dead.h"
#include "CyberDog_Motor_FOC.h"   /* Y.id_meas/iq_meas、DeadComp_*、VmaxCoeff */
#include "foc.h"                  /* Motor_Params / FOC_Frequency */
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ---------------- 参数 ---------------- */
/* 目标电流点（20 点，最大 1.5A：0.05~0.5A 每 0.05 步进——过零区最密；0.5~1.5A 每 0.1 步进） */
static const float DEAD_CURRENT_TARGET[DEAD_LUT_N] = {
    0.05f, 0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.35f, 0.40f, 0.45f, 0.50f,
    0.60f, 0.70f, 0.80f, 0.90f, 1.00f, 1.10f, 1.20f, 1.30f, 1.40f, 1.50f
};

#define D_VD_STEP          (10.0f / FOC_Frequency)        /* d 轴升压步进 ~10V/s */
#define D_VD_LIMIT         (VmaxCoeff)                    /* 模型实际电压限 Vbus/√3·0.95，避免 limiter 截顶 */
#define D_WAIT_CNT         ((int)(FOC_Frequency * 0.3f))  /* 初始撤压稳定 */
#define D_SETTLE_CNT       ((int)(FOC_Frequency * 0.1f))  /* 每点稳态等待 */
#define D_SAMPLE_N         ((int)(FOC_Frequency * 0.05f)) /* 每点均值采样数 */
#define D_RAMP_TIMEOUT_CNT ((int)(FOC_Frequency * 3.0f))  /* 单段升压超时 */
#define D_LPF_A            0.05f
#define D_EPS              1e-6f

/* ---------------- 子 FSM ---------------- */
typedef enum {
    D_INIT = 0,
    D_WAIT,
    D_RAMP,
    D_HOLD,
    D_SAMPLE,
    D_NEXT,
    D_DONE,
    D_FAIL
} dead_state_t;

static struct {
    dead_state_t state;
    int   count;
    float vd_ref;
    float vq_ref;
    float i_mag_filtered;
    int   pt_idx;
    float acc_i;
    float acc_vd;
    int   sample_cnt;
} d_ctx = {0};

#define LPF(x, last, a)  ((a) * (x) + (1.0f - (a)) * (last))

void identify_dead_Init(Identify_Context_t *ctx)
{
    (void)ctx;
    memset(&d_ctx, 0, sizeof(d_ctx));
    d_ctx.state = D_INIT;
    /* 测原始畸变：确保死区补偿关闭（电压模式本身已旁路 DeadTimeComp，双保险） */
    DeadComp_En = 0.0f;
}

void identify_dead_Step(Identify_Context_t *ctx)
{
    float id   = CyberDog_Motor_FOC_Y.id_meas;
    float iq   = CyberDog_Motor_FOC_Y.iq_meas;
    float i_mag = sqrtf(id * id + iq * iq);
    d_ctx.i_mag_filtered = LPF(i_mag, d_ctx.i_mag_filtered, D_LPF_A);

    switch (d_ctx.state) {
    case D_INIT:
        d_ctx.state = D_WAIT;
        break;

    case D_WAIT:                    /* 撤压等稳定 */
        d_ctx.vd_ref = 0.0f;  d_ctx.vq_ref = 0.0f;
        if (++d_ctx.count >= D_WAIT_CNT) { d_ctx.count = 0; d_ctx.state = D_RAMP; }
        break;

    case D_RAMP: {                  /* 升 vd 直到 |I| ≥ 目标电流（或达电压限） */
        float target = DEAD_CURRENT_TARGET[d_ctx.pt_idx];
        if (d_ctx.vd_ref < D_VD_LIMIT) {
            d_ctx.vd_ref += D_VD_STEP;
            if (d_ctx.vd_ref > D_VD_LIMIT) d_ctx.vd_ref = D_VD_LIMIT;
        }
        if (d_ctx.i_mag_filtered >= target || d_ctx.vd_ref >= D_VD_LIMIT) {
            d_ctx.count = 0;
            d_ctx.state = D_HOLD;   /* 达目标或达限：采样当前点（LUT 断点用实测 |I|） */
        } else if (++d_ctx.count >= D_RAMP_TIMEOUT_CNT) {
            d_ctx.count = 0;
            d_ctx.state = D_HOLD;   /* 超时：按当前电流记录 */
        }
        break;
    }

    case D_HOLD:                    /* 保持电压等稳态 */
        if (++d_ctx.count >= D_SETTLE_CNT) {
            d_ctx.count = 0;
            d_ctx.acc_i = 0.0f; d_ctx.acc_vd = 0.0f; d_ctx.sample_cnt = 0;
            d_ctx.state = D_SAMPLE;
        }
        break;

    case D_SAMPLE:                  /* 稳态均值采样 (|I|, vd) */
        d_ctx.acc_i += i_mag;
        d_ctx.acc_vd += d_ctx.vd_ref;
        if (++d_ctx.sample_cnt >= D_SAMPLE_N) {
            float i_avg = d_ctx.acc_i / (float)d_ctx.sample_cnt;
            float vd_avg = d_ctx.acc_vd / (float)d_ctx.sample_cnt;
            float R = Motor_Params.Phase_R;
            float vcomp = vd_avg - R * i_avg;          /* 纯逆变器畸变 = |V| − R·|I| */
            if (vcomp < 0.0f) vcomp = 0.0f;
            ctx->res.dead_lut_I[d_ctx.pt_idx] = i_avg;
            ctx->res.dead_lut_V[d_ctx.pt_idx] = vcomp;
            d_ctx.state = D_NEXT;
        }
        break;

    case D_NEXT:
        d_ctx.pt_idx++;
        if (d_ctx.pt_idx >= DEAD_LUT_N) d_ctx.state = D_DONE;
        else { d_ctx.count = 0; d_ctx.state = D_RAMP; }
        break;

    case D_DONE:
        d_ctx.vd_ref = 0.0f;  d_ctx.vq_ref = 0.0f;
        /* 合理性检查：最大实测电流应达到有意义水平，否则视为失败（防止使能废 LUT） */
        if (ctx->res.dead_lut_I[DEAD_LUT_N - 1] < 0.5f) {
            ctx->task_fail_flag = 1;
            printf("DEAD_IDENT FAIL: max_I=%.3f<0.5A\r\n",
                   (double)ctx->res.dead_lut_I[DEAD_LUT_N - 1]);
        } else {
            ctx->res.Vdead_est = ctx->res.dead_lut_V[DEAD_LUT_N - 1];
            ctx->task_done_flag = 1;
        }
        d_ctx.state = D_INIT;
        break;

    case D_FAIL:
        d_ctx.vd_ref = 0.0f;  d_ctx.vq_ref = 0.0f;
        ctx->task_fail_flag = 1;
        d_ctx.state = D_INIT;
        break;
    }

    /* 应用 dq 电压（复用模型电压模式；q=0 锁转子于 d 轴） */
    identify_apply_voltage(d_ctx.vd_ref, d_ctx.vq_ref);
}
