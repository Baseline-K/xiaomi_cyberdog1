/**
 * identify_j.c — 转动惯量 J 辨识（移植 AC_Control Inertia_Identify_Step，加减速对称法）
 *
 * 子 FSM：INIT → WAIT(等静止) → ACCEL(iq 斜坡至目标) → ACCEL_SAMPLE(采 |α|>20 至 0.50·max)
 *   → DECEL_WAIT_SPEED(保持 iq 至 0.70·max) → DECEL(负 iq 斜坡) → DECEL_SAMPLE(采 α<−20 至 0.30·max)
 *   → [×4 点] → CALC → DONE。
 * 拟合：J_est=(Kt·iq_avg−B·ωm_avg−fc)/α_avg；J=(J_accel+J_decel)/2、J_error=|ΔJ|/max。
 * 驱动复用模型转矩模式（identify_apply_torque，ctrl_mode=0 电流环闭环保持 iq）；
 * 读 Y.iq_meas、Y.speed_meas_rps×2π、α=LPF(dwm/dt)。
 * 电压饱和保护：每状态 8s 超时（防电流不足达不到目标转速卡死）；采样段 |iq_meas−iq_cmd|>20% 判饱和，跳样本并警告。
 */
#include "identify.h"
#include "identify_j.h"
#include "CyberDog_Motor_FOC.h"   /* Y.iq_meas / speed_meas_rps */
#include "foc.h"                  /* Motor_Params / Two_PI / FOC_Frequency */
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ---------------- 参数（参考 Inertia 默认） ---------------- */
#define INERTIA_IQ_NUM           4
static const float INERTIA_IQ_RATIO[INERTIA_IQ_NUM] = {0.4f, 0.6f, 0.8f, 1.0f};  /* ×额定电流（最高 1.0·额定=1A，避开电压饱和） */

#define INERTIA_WAIT_SPEED_TH     ((float)Two_PI)            /* 等待静止速度阈值 1 RPS（rad/s） */
#define INERTIA_WAIT_ACCEL_TH     (100.0f * (float)Two_PI)  /* 等待静止加速度阈值 */
#define INERTIA_WAIT_TIMEOUT_CNT  ((int)(FOC_Frequency * 5.0f))
#define INERTIA_SETTLE_TIME       ((int)(FOC_Frequency * 0.001f))   /* 稳定 1ms */
#define INERTIA_RAMP_STEP         (4000.0f / FOC_Frequency)         /* iq 斜坡步进 */
#define INERTIA_IQ_ERROR          0.05f                              /* iq 误差 5% */
#define INERTIA_MIN_ALPHA         20.0f                              /* 采样最小加速度 rad/s² */
#define INERTIA_ALPHA_LPF_K       0.10f                              /* α 一阶低通 */
#define INERTIA_SPEED_ACCEL_START 0.30f   /* 加速开始/减速结束 速度比例（最高 0.70 max，避电压饱和） */
#define INERTIA_SPEED_ACCEL_END   0.50f   /* 加速结束/减速开始 速度比例 */
#define INERTIA_SPEED_DECEL_WAIT  0.70f   /* 减速等待速度比例 */
#define INERTIA_STATE_TIMEOUT_CNT ((int)(FOC_Frequency * 8.0f))   /* 每状态超时 8s（防电压饱和达不到转速卡死） */
#define INERTIA_IQ_SAT_FRAC       0.20f                            /* 电压饱和判据：实测 iq 偏离指令 >20% */

/* ---------------- 子 FSM ---------------- */
typedef enum {
    INERTIA_INIT = 0,
    INERTIA_WAIT,
    INERTIA_ACCEL,
    INERTIA_ACCEL_SAMPLE,
    INERTIA_DECEL_WAIT_SPEED,
    INERTIA_DECEL,
    INERTIA_DECEL_SAMPLE,
    INERTIA_CALC,
    INERTIA_DONE
} inertia_state_t;

/* 采样统计量（LS/平均拟合用） */
typedef struct {
    int   n;
    float sum_iq, sum_wm, sum_alpha;
    float sum_iq2, sum_wm2, sum_alpha2;
    float sum_iq_wm, sum_iq_alpha, sum_wm_alpha;
    int   valid;
} InertiaData_t;

static struct {
    inertia_state_t state;
    inertia_state_t prev_state;   /* 状态切换检测（每状态超时用） */
    int   state_t;                /* 当前状态停留计数 */
    int   pt_idx;
    int   count;
    float iq_ref;
    uint8_t sat_prev;             /* 上一拍是否电压饱和（上升沿打印警告） */
    float wm_prev, alpha_lpf;
    InertiaData_t accel_data[INERTIA_IQ_NUM];
    InertiaData_t decel_data[INERTIA_IQ_NUM];
    float J_accel, J_decel, J_avg, J_error;
} j_ctx = {0};

void identify_j_Init(Identify_Context_t *ctx)
{
    (void)ctx;
    memset(&j_ctx, 0, sizeof(j_ctx));
    j_ctx.state = INERTIA_INIT;
    j_ctx.wm_prev = CyberDog_Motor_FOC_Y.speed_meas_rps * (float)Two_PI;
    DeadComp_En = 0.0f;   /* 测原始 */
}

void identify_j_Step(Identify_Context_t *ctx)
{
    const float max_wm = (Motor_Params.RPM_Rating / 60.0f) * (float)Two_PI;   /* 机械 rad/s */
    const float wm_accel_start = max_wm * INERTIA_SPEED_ACCEL_START;
    const float wm_accel_end   = max_wm * INERTIA_SPEED_ACCEL_END;
    const float wm_decel_wait  = max_wm * INERTIA_SPEED_DECEL_WAIT;
    const float iq_target      = Motor_Params.Current_Rating * INERTIA_IQ_RATIO[j_ctx.pt_idx];
    float iq_now   = CyberDog_Motor_FOC_Y.iq_meas;
    float wm_now   = CyberDog_Motor_FOC_Y.speed_meas_rps * (float)Two_PI;   /* 机械 rad/s */
    float alpha_raw = (wm_now - j_ctx.wm_prev) * FOC_Frequency;
    float alpha_now;

    j_ctx.wm_prev = wm_now;
    j_ctx.alpha_lpf += INERTIA_ALPHA_LPF_K * (alpha_raw - j_ctx.alpha_lpf);
    alpha_now = j_ctx.alpha_lpf;

    /* 状态停留跟踪 */
    if (j_ctx.state != j_ctx.prev_state) {
        j_ctx.prev_state = j_ctx.state;
        j_ctx.state_t = 0;
    }
    j_ctx.state_t++;

    /* 每状态超时：ACCEL~DECEL_SAMPLE 超过 8s 未推进 → 失败
     * （电压饱和 → 电流不足 → 电机达不到目标转速 → 卡在等转速状态，防卡死） */
    if (j_ctx.state >= INERTIA_ACCEL && j_ctx.state <= INERTIA_DECEL_SAMPLE &&
        j_ctx.state_t >= INERTIA_STATE_TIMEOUT_CNT) {
        printf("INERTIA: TIMEOUT state=%d\r\n", (int)j_ctx.state);
        ctx->task_fail_flag = 1;
        j_ctx.state = INERTIA_DONE;
    }

    switch (j_ctx.state) {
    case INERTIA_INIT:
        j_ctx.pt_idx = 0;
        j_ctx.count = 0;
        j_ctx.iq_ref = 0.0f;
        j_ctx.state = INERTIA_WAIT;
        break;

    case INERTIA_WAIT:            /* 等电机静止 */
        if (wm_now < INERTIA_WAIT_SPEED_TH && fabsf(alpha_now) < INERTIA_WAIT_ACCEL_TH) {
            if (j_ctx.count >= 10 * INERTIA_SETTLE_TIME) {
                j_ctx.count = 0;
                j_ctx.state = INERTIA_ACCEL;
            }
        }
        j_ctx.count++;
        if (j_ctx.count >= INERTIA_WAIT_TIMEOUT_CNT) {
            ctx->task_fail_flag = 1;
            j_ctx.state = INERTIA_DONE;
        }
        break;

    case INERTIA_ACCEL:           /* 逐渐增加 iq 至目标 */
        j_ctx.iq_ref += INERTIA_RAMP_STEP;
        if (j_ctx.iq_ref > iq_target) j_ctx.iq_ref = iq_target;
        if (wm_now >= wm_accel_start) {
            j_ctx.count++;
            float iq_err = fabsf(j_ctx.iq_ref - iq_target) / iq_target;
            if (iq_err < INERTIA_IQ_ERROR && j_ctx.count >= INERTIA_SETTLE_TIME) {
                j_ctx.count = 0;
                j_ctx.state = INERTIA_ACCEL_SAMPLE;
            }
        }
        break;

    case INERTIA_ACCEL_SAMPLE: {  /* 加速采样：累加统计量至 0.50·max */
        InertiaData_t *pd = &j_ctx.accel_data[j_ctx.pt_idx];
        /* 电压饱和检测：实测 iq 偏离指令 >20%（电流环电压饱和 iq 塌陷）→ 该样本不采 */
        int saturated = (fabsf(iq_now - j_ctx.iq_ref) > INERTIA_IQ_SAT_FRAC * iq_target);
        if (saturated && !j_ctx.sat_prev) {
            printf("INERTIA: SAT pt=%d accel\r\n", j_ctx.pt_idx);
        }
        j_ctx.sat_prev = (uint8_t)saturated;
        if (!saturated && fabsf(alpha_now) > INERTIA_MIN_ALPHA) {
            pd->n++;
            pd->sum_iq += iq_now;
            pd->sum_wm += wm_now;
            pd->sum_alpha += alpha_now;
            pd->sum_iq2 += iq_now * iq_now;
            pd->sum_wm2 += wm_now * wm_now;
            pd->sum_alpha2 += alpha_now * alpha_now;
            pd->sum_iq_wm += iq_now * wm_now;
            pd->sum_iq_alpha += iq_now * alpha_now;
            pd->sum_wm_alpha += wm_now * alpha_now;
        }
        if (wm_now >= wm_accel_end && pd->n > 0) {
            j_ctx.count++;
            if (j_ctx.count >= INERTIA_SETTLE_TIME) {
                pd->valid = 1;
                j_ctx.count = 0;
                j_ctx.state = INERTIA_DECEL_WAIT_SPEED;
            }
        }
        break;
    }

    case INERTIA_DECEL_WAIT_SPEED: /* 保持 iq 至 0.8·max */
        j_ctx.iq_ref = iq_target;
        if (wm_now >= wm_decel_wait) {
            j_ctx.count++;
            if (j_ctx.count >= INERTIA_SETTLE_TIME) {
                j_ctx.count = 0;
                j_ctx.state = INERTIA_DECEL;
            }
        }
        break;

    case INERTIA_DECEL:           /* 逐渐给负 iq 至目标 */
        j_ctx.iq_ref -= INERTIA_RAMP_STEP;
        if (j_ctx.iq_ref < -iq_target) j_ctx.iq_ref = -iq_target;
        if (wm_now <= wm_accel_end) {
            j_ctx.count++;
            float iq_err = fabsf(j_ctx.iq_ref + iq_target) / iq_target;
            if (iq_err < INERTIA_IQ_ERROR && j_ctx.count >= INERTIA_SETTLE_TIME) {
                j_ctx.count = 0;
                j_ctx.state = INERTIA_DECEL_SAMPLE;
            }
        }
        break;

    case INERTIA_DECEL_SAMPLE: {  /* 减速采样：累加至 0.30·max */
        InertiaData_t *pd = &j_ctx.decel_data[j_ctx.pt_idx];
        /* 电压饱和检测（iq_cmd = −target；iq 偏离指令 → 不采） */
        int saturated = (fabsf(iq_now - j_ctx.iq_ref) > INERTIA_IQ_SAT_FRAC * iq_target);
        if (saturated && !j_ctx.sat_prev) {
            printf("INERTIA: SAT pt=%d decel\r\n", j_ctx.pt_idx);
        }
        j_ctx.sat_prev = (uint8_t)saturated;
        if (!saturated && alpha_now < -INERTIA_MIN_ALPHA) {
            pd->n++;
            pd->sum_iq += iq_now;
            pd->sum_wm += wm_now;
            pd->sum_alpha += alpha_now;   /* 负值 */
            pd->sum_iq2 += iq_now * iq_now;
            pd->sum_wm2 += wm_now * wm_now;
            pd->sum_alpha2 += alpha_now * alpha_now;
            pd->sum_iq_wm += iq_now * wm_now;
            pd->sum_iq_alpha += iq_now * alpha_now;
            pd->sum_wm_alpha += wm_now * alpha_now;
        }
        if (wm_now <= wm_accel_start && pd->n > 0) {
            pd->valid = 1;
            j_ctx.count++;
            if (j_ctx.count >= INERTIA_SETTLE_TIME) {
                j_ctx.pt_idx++;
                j_ctx.count = 0;
                if (j_ctx.pt_idx >= INERTIA_IQ_NUM) {
                    j_ctx.state = INERTIA_CALC;
                } else {
                    j_ctx.iq_ref = 0.0f;
                    j_ctx.state = INERTIA_WAIT;
                }
            }
        }
        break;
    }

    case INERTIA_CALC: {          /* 拟合 J（加减速对称 + 一致性校验） */
        float Kt = (ctx->res.Kt > 1.0e-6f) ? ctx->res.Kt
                                           : (1.5f * Motor_Params.Pole_Pairs * Motor_Params.Flux);
        float B  = ctx->res.B;
        float fc = ctx->res.friction_Coulomb;
        float J_acc_sum = 0.0f, J_dec_sum = 0.0f;
        int n_acc = 0, n_dec = 0;

        if (Kt <= 1.0e-6f) {
            printf("INERTIA FAIL: Kt=0 (先跑 identify flux)\r\n");
            ctx->task_fail_flag = 1;
            j_ctx.state = INERTIA_DONE;
            break;
        }

        for (int i = 0; i < INERTIA_IQ_NUM; i++) {
            if (!j_ctx.accel_data[i].valid) continue;
            InertiaData_t *pd = &j_ctx.accel_data[i];
            float iq_avg    = pd->sum_iq    / pd->n;
            float wm_avg    = pd->sum_wm    / pd->n;
            float alpha_avg = pd->sum_alpha / pd->n;
            J_acc_sum += (Kt * iq_avg - B * wm_avg - fc) / alpha_avg;
            n_acc++;
        }
        for (int i = 0; i < INERTIA_IQ_NUM; i++) {
            if (!j_ctx.decel_data[i].valid) continue;
            InertiaData_t *pd = &j_ctx.decel_data[i];
            float iq_avg    = pd->sum_iq    / pd->n;
            float wm_avg    = pd->sum_wm    / pd->n;
            float alpha_avg = pd->sum_alpha / pd->n;   /* 负值 */
            J_dec_sum += (Kt * iq_avg - B * wm_avg - fc) / alpha_avg;
            n_dec++;
        }

        if (n_acc > 0 && n_dec > 0) {
            j_ctx.J_accel = J_acc_sum / n_acc;
            j_ctx.J_decel = J_dec_sum / n_dec;
            j_ctx.J_avg   = 0.5f * (j_ctx.J_accel + j_ctx.J_decel);
            float J_diff  = fabsf(j_ctx.J_accel - j_ctx.J_decel);
            float J_max   = fmaxf(j_ctx.J_accel, j_ctx.J_decel);
            j_ctx.J_error = (J_max > 1.0e-6f) ? (J_diff / J_max) : 1.0f;
            ctx->res.J = j_ctx.J_avg;
            ctx->res.J_error = j_ctx.J_error;
            printf("INERTIA: J_accel=%.7f J_decel=%.7f J=%.7f err=%.3f (n=%d/%d)\r\n",
                   (double)j_ctx.J_accel, (double)j_ctx.J_decel, (double)j_ctx.J_avg,
                   (double)j_ctx.J_error, n_acc, n_dec);
        } else {
            ctx->res.J = -1.0f;
            ctx->task_fail_flag = 1;
        }
        j_ctx.state = INERTIA_DONE;
        break;
    }

    case INERTIA_DONE:
        j_ctx.iq_ref = 0.0f;
        j_ctx.state = INERTIA_INIT;   /* 为下次使用重置 */
        ctx->task_done_flag = 1;
        break;
    }

    /* 控制输出：模型转矩模式（iq 指令）；失败时撤压 */
    if (ctx->task_fail_flag) {
        identify_apply_torque(0.0f);
    } else {
        identify_apply_torque(j_ctx.iq_ref);
    }
}
