/**
 * identify_r.c — 相电阻 R 辨识（移植 AC_Control Resistance_Identify_Step_VoltageRamp）
 *
 * 子 FSM：R_INIT → R_WAIT(撤压 0.3s) → R_RAMP(升 vd 至 id≥目标) → R_HOLD(0.1s)
 *         → R_SAMPLE(0.05s 均值) → R_NEXT(×4) → R_FIT(LS) → R_DONE/R_FAIL
 * 输出：ctx->res.Rs（斜率=相电阻）、ctx->res.Vdead_est（截距=死区压降）
 */
#include "identify.h"
#include "identify_r.h"
#include "CyberDog_Motor_FOC.h"   /* Y.id_meas */
#include "FOC_run.h"              /* MotorState / DeadComp_En 使能控制 */
#include "foc.h"                  /* Motor_Params / FOC_Frequency */
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ---------------- 参数 ---------------- */
#define R_FIT_NUM       4
/* 目标电流点：绝对电流 A（本电机额定 1A，辨识到 1.0A=额定电流） */
static const float R_CURRENT_TARGET[R_FIT_NUM] = {0.25f, 0.5f, 0.75f, 1.0f};

#define R_VD_STEP          (10.0f / FOC_Frequency)                          /* d 轴升压步进 ~10V/s */
#define R_VD_LIMIT         (Motor_Params.VBUS * 0.57735027f)                /* 电压上限 VBUS/√3 */
#define R_WAIT_CNT         ((int)(FOC_Frequency * 0.3f))                    /* 初始撤压稳定 0.3s */
#define R_SETTLE_CNT       ((int)(FOC_Frequency * 0.1f))                    /* 每点稳态等待 0.1s */
#define R_SAMPLE_N         ((int)(FOC_Frequency * 0.05f))                   /* 每点均值采样数 0.05s */
#define R_RAMP_TIMEOUT_CNT ((int)(FOC_Frequency * 3.0f))                    /* 单段升压超时 3s */
#define R_LPF_A            0.05f                                            /* 升压判据电流低通系数 */
#define R_EPS              1e-6f

/* ---------------- 子 FSM ---------------- */
typedef enum {
    R_INIT = 0,
    R_WAIT,
    R_RAMP,
    R_HOLD,
    R_SAMPLE,
    R_NEXT,
    R_FIT,
    R_FAIL,
    R_DONE
} r_ident_state_t;

static struct {
    r_ident_state_t state;
    int   count;
    float vd_ref;
    float vq_ref;
    float id_filtered;
    int   pt_idx;
    float id_data[R_FIT_NUM];
    float vd_data[R_FIT_NUM];
    int   valid[R_FIT_NUM];
    float acc_id;
    float acc_vd;
    int   sample_cnt;
} r_ctx = {0};

#define LPF(x, last, a)  ((a) * (x) + (1.0f - (a)) * (last))

void identify_r_Init(Identify_Context_t *ctx)
{
    (void)ctx;
    memset(&r_ctx, 0, sizeof(r_ctx));
    r_ctx.state = R_INIT;
    /* 辨识必须测原始电压畸变：确保死区补偿关闭 */
    DeadComp_En = 0.0f;
}

void identify_r_Step(Identify_Context_t *ctx)
{
    float id = CyberDog_Motor_FOC_Y.id_meas;
    r_ctx.id_filtered = LPF(id, r_ctx.id_filtered, R_LPF_A);

    switch (r_ctx.state) {
    case R_INIT:
        if (R_CURRENT_TARGET[0] <= R_EPS || R_VD_LIMIT <= R_EPS) {
            r_ctx.state = R_FAIL;
        } else {
            r_ctx.state = R_WAIT;
        }
        break;

    case R_WAIT:                    /* 撤压等稳定 */
        r_ctx.vd_ref = 0.0f;  r_ctx.vq_ref = 0.0f;
        if (++r_ctx.count >= R_WAIT_CNT) {
            r_ctx.count = 0;
            r_ctx.state = R_RAMP;
        }
        break;

    case R_RAMP: {                  /* 升 vd 直到 id ≥ 目标电流 */
        float id_target = R_CURRENT_TARGET[r_ctx.pt_idx];
        if (id_target > Motor_Params.Current_Max) id_target = Motor_Params.Current_Max;  /* 安全上界 */
        if (r_ctx.vd_ref < R_VD_LIMIT) {
            r_ctx.vd_ref += R_VD_STEP;
            if (r_ctx.vd_ref > R_VD_LIMIT) r_ctx.vd_ref = R_VD_LIMIT;
        }
        if (r_ctx.id_filtered >= id_target) {
            r_ctx.count = 0;
            r_ctx.state = R_HOLD;
        } else if (r_ctx.vd_ref >= R_VD_LIMIT || ++r_ctx.count >= R_RAMP_TIMEOUT_CNT) {
            r_ctx.state = R_FIT;    /* 达限/超时：用已采到的点拟合 */
        }
        break;
    }

    case R_HOLD:                    /* 保持电压等稳态 */
        if (++r_ctx.count >= R_SETTLE_CNT) {
            r_ctx.count = 0;
            r_ctx.acc_id = 0.0f; r_ctx.acc_vd = 0.0f; r_ctx.sample_cnt = 0;
            r_ctx.state = R_SAMPLE;
        }
        break;

    case R_SAMPLE:                  /* 稳态均值采样 (id, vd) */
        r_ctx.acc_id += id;
        r_ctx.acc_vd += r_ctx.vd_ref;
        if (++r_ctx.sample_cnt >= R_SAMPLE_N) {
            r_ctx.id_data[r_ctx.pt_idx] = r_ctx.acc_id / (float)r_ctx.sample_cnt;
            r_ctx.vd_data[r_ctx.pt_idx] = r_ctx.acc_vd / (float)r_ctx.sample_cnt;
            r_ctx.valid[r_ctx.pt_idx]   = 1;
            r_ctx.state = R_NEXT;
        }
        break;

    case R_NEXT:
        r_ctx.pt_idx++;
        if (r_ctx.pt_idx >= R_FIT_NUM) {
            r_ctx.state = R_FIT;
        } else {
            r_ctx.count = 0;
            r_ctx.state = R_RAMP;
        }
        break;

    case R_FIT: {                   /* 一元线性最小二乘: Vd = R·Id + Vdead */
        int   n   = 0;
        float Sx  = 0.0f, Sy = 0.0f, Sxx = 0.0f, Sxy = 0.0f;
        for (int i = 0; i < R_FIT_NUM; i++) {
            if (!r_ctx.valid[i]) continue;
            float x = r_ctx.id_data[i];
            float y = r_ctx.vd_data[i];
            Sx += x; Sy += y; Sxx += x * x; Sxy += x * y;
            n++;
        }
        float det = (float)n * Sxx - Sx * Sx;
        if (n < 2 || fabsf(det) < R_EPS) {
            r_ctx.state = R_FAIL;
            break;
        }
        float R_slope    = ((float)n * Sxy - Sx * Sy) / det;   /* 斜率 = 相电阻 */
        float Vdead_intc = (Sxx * Sy - Sx * Sxy) / det;        /* 截距 = 死区电压 */
        if (R_slope <= 0.0f) {
            r_ctx.state = R_FAIL;
            break;
        }
        ctx->res.Rs        = R_slope;
        ctx->res.Vdead_est = Vdead_intc;
        r_ctx.state = R_DONE;
        break;
    }

    case R_FAIL:
        r_ctx.vd_ref = 0.0f;  r_ctx.vq_ref = 0.0f;
        {   /* 诊断：打印有效采样点数，便于定位（电流未达目标/拟合退化） */
            int n = 0;
            for (int i = 0; i < R_FIT_NUM; i++) if (r_ctx.valid[i]) n++;
            printf("R_IDENT FAIL: valid=%d/%d id_fb=%.3f vd=%.3f\r\n", n, R_FIT_NUM,
                   (double)r_ctx.id_filtered, (double)r_ctx.vd_ref);
        }
        ctx->task_fail_flag = 1;
        r_ctx.state = R_INIT;
        break;

    case R_DONE:
        r_ctx.vd_ref = 0.0f;  r_ctx.vq_ref = 0.0f;
        r_ctx.state = R_INIT;
        ctx->task_done_flag = 1;
        break;
    }

    /* 应用 dq 电压（复用模型电压模式；q=0 锁转子于 d 轴） */
    identify_apply_voltage(r_ctx.vd_ref, r_ctx.vq_ref);
}
