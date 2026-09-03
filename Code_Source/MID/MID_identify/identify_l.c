/**
 * identify_l.c — 相电感 L（Ld/Lq）辨识（HFI 高频脉冲注入，移植 AC_Control Inductance_Identify_HFI_Step）
 *
 * 原理：沿 32 个方向依次注入 +V/−V 短电压脉冲，把 αβ 电流响应投影到注入方向，
 *       偶奇分解：偶部 → 1/L 导纳（bin0/bin2 定轴线与 Ld/Lq），奇部 → 饱和极性（L_ONLY 不用）。
 * 幅值自搜索：V 从 1.0V 起 ×1.5，直到 32 方向平均 Δi ≥ 0.35·Current_Rating 或达 VBUS/√3·0.5。
 * L_ONLY：凸极 Lmax/Lmin<1.05 → Ld=Lq=Lavg（本表贴电机适用）。
 *
 * 复用模型 αβ 电压模式：identify_apply_alpha_beta(vα,vβ) → ctrl_mode=4 直通 SVPWM；
 * 电流响应读 Y.i_alpha_meas/i_beta_meas（模型 Clarke 输出，1 拍延迟，脉冲时序已留余量）。
 */
#include "identify.h"
#include "identify_l.h"
#include "CyberDog_Motor_FOC.h"   /* Y.i_alpha_meas/i_beta_meas */
#include "foc.h"                  /* Motor_Params / timFactor / Two_PI */
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ---------------- 参数（对齐参考） ---------------- */
#define HFI_DIR_NUM              32
#define HFI_RESULT_ROUND_NUM     12
#define HFI_RESULT_SCALE         0.9f      /* 保守缩放，降低采样延迟/死区导致电感偏大 */
#define HFI_PULSE_CNT            3         /* +V/−V 脉冲拍数 */
#define HFI_ZERO_CNT             2         /* 方向切换后零电压等待拍数 */
#define HFI_EFFECTIVE_PULSE_CNT  2         /* 电感估算有效脉冲拍数（含 1 拍延迟近似） */
#define HFI_PULSE_TIME           ((float)HFI_EFFECTIVE_PULSE_CNT * timFactor)
#define HFI_WAIT_SETTLE_CNT      ((int)(FOC_Frequency * 0.5f))   /* 进入前静止等待 0.5s */
#define HFI_WAIT_TIMEOUT_CNT     ((int)(FOC_Frequency * 5.0f))
#define HFI_V_AMP_START          1.0f
#define HFI_V_AMP_GAIN           1.5f
#define HFI_V_AMP_LIMIT          (Motor_Params.VBUS * 0.57735027f)
#define HFI_I_DELTA_TARGET       (Motor_Params.Current_Rating * 0.35f)
#define HFI_AXIS_SALIENCY_MIN_RATIO 1.05f
#define HFI_RESPONSE_SIGN        1.0f
#define HFI_TWO_BY_DIR_NUM       (2.0f / (float)HFI_DIR_NUM)
#define HFI_EPS                  1.0e-6f

/* ---------------- 子 FSM ---------------- */
typedef enum {
    HFI_INIT = 0,
    HFI_WAIT,
    HFI_ZERO,
    HFI_POS_SAMPLE,
    HFI_NEG_SAMPLE,
    HFI_NEXT_DIR,
    HFI_AMP_CHECK,
    HFI_CALC,
    HFI_DONE,
    HFI_FAIL
} hfi_state_t;

typedef struct {
    hfi_state_t state;
    int   count;
    int   wait_count;
    float v_amp;
    int   dir_idx;
    float theta, cos_theta, sin_theta;
    int   is_amp_search;
    /* 每方向采样（αβ 电流） */
    float i_alpha_base, i_beta_base, i_alpha_pos, i_beta_pos, i_alpha_neg, i_beta_neg;
    float base_sample, pos_sample, neg_sample;
    /* 幅值搜索统计 */
    float delta_i_sum;
    int   delta_i_count;
    float avg_delta_i;
    /* 32 方向响应 */
    float response[HFI_DIR_NUM];
    float inv_l_response[HFI_DIR_NUM];
    /* DFT bins */
    float bin1_re, bin1_im, bin1_amp;
    float inv_l_bin0, inv_l_bin2_re, inv_l_bin2_im, inv_l_bin2_amp;
    /* 多轮累加 */
    float inv_l_bin0_sum, inv_l_bin2_re_sum, inv_l_bin2_im_sum;
    int   valid_round_count;
    int   result_round_idx;
    int   d_axis_is_max_l;
    /* 结果 */
    float Lavg, Ld, Lq;
} hfi_ctx_t;

static hfi_ctx_t h = {0};

/* ---------------- 辅助 ---------------- */
static void hfi_update_dir_vector(void)
{
    h.theta = Two_PI * (float)h.dir_idx / (float)HFI_DIR_NUM;
    h.cos_theta = cosf(h.theta);
    h.sin_theta = sinf(h.theta);
}

static float hfi_project_current(float i_alpha, float i_beta)
{
    return i_alpha * h.cos_theta + i_beta * h.sin_theta;
}

static void hfi_reset_scan_data(void)
{
    memset(h.response, 0, sizeof(h.response));
    memset(h.inv_l_response, 0, sizeof(h.inv_l_response));
}

static void hfi_start_amp_sweep(void)
{
    h.dir_idx = 0;
    h.count = 0;
    h.is_amp_search = 1;
    h.avg_delta_i = 0.0f;
    h.delta_i_sum = 0.0f;
    h.delta_i_count = 0;
    hfi_update_dir_vector();
    h.state = HFI_ZERO;
}

static void hfi_start_scan_sweep(void)
{
    h.dir_idx = 0;
    h.count = 0;
    h.is_amp_search = 0;
    hfi_reset_scan_data();
    hfi_update_dir_vector();
    h.state = HFI_ZERO;
}

static void hfi_start_result_rounds(void)
{
    h.result_round_idx = 0;
    h.Ld = h.Lq = h.Lavg = 0.0f;
    h.inv_l_bin0_sum = h.inv_l_bin2_re_sum = h.inv_l_bin2_im_sum = 0.0f;
    h.valid_round_count = 0;
    hfi_start_scan_sweep();
}

static void hfi_fail(Identify_Context_t *ctx)
{
    identify_apply_alpha_beta(0.0f, 0.0f);
    printf("L_IDENT FAIL: Lavg=%.6f valid=%d\r\n", (double)h.Lavg, h.valid_round_count);
    ctx->task_fail_flag = 1;
    h.state = HFI_INIT;
}

static float hfi_finish_pulse_pair(Identify_Context_t *ctx, float *signed_response, float *inv_l_response)
{
    float di_pos, di_neg, di_even, di_odd, delta_mag, scale;

    h.base_sample = hfi_project_current(h.i_alpha_base, h.i_beta_base);
    h.pos_sample  = hfi_project_current(h.i_alpha_pos,  h.i_beta_pos);
    h.neg_sample  = hfi_project_current(h.i_alpha_neg,  h.i_beta_neg);

    di_pos  = h.pos_sample - h.base_sample;
    di_neg  = h.neg_sample - h.pos_sample;
    di_even = 0.5f * (di_pos - di_neg);   /* 偶部 → 电感（bin0/bin2） */
    di_odd  = 0.5f * (di_pos + di_neg);   /* 奇部 → 极性（L_ONLY 不用） */
    delta_mag = fabsf(di_even);

    scale = (h.v_amp > HFI_EPS && HFI_PULSE_TIME > HFI_EPS) ?
            (1.0f / (h.v_amp * HFI_PULSE_TIME)) : 0.0f;

    *signed_response = HFI_RESPONSE_SIGN * di_odd * scale;
    *inv_l_response  = delta_mag * scale;
    (void)ctx;
    return delta_mag;
}

static void hfi_calc_dft(void)
{
    float sum1_re = 0.0f, sum1_im = 0.0f, inv_sum0 = 0.0f, inv_sum2_re = 0.0f, inv_sum2_im = 0.0f;

    for (int k = 0; k < HFI_DIR_NUM; k++) {
        float theta = Two_PI * (float)k / (float)HFI_DIR_NUM;
        float y     = h.response[k];
        float inv_l = h.inv_l_response[k];
        sum1_re    += y * cosf(theta);
        sum1_im    -= y * sinf(theta);
        inv_sum0   += inv_l;
        inv_sum2_re += inv_l * cosf(2.0f * theta);
        inv_sum2_im -= inv_l * sinf(2.0f * theta);
    }
    h.bin1_re = HFI_TWO_BY_DIR_NUM * sum1_re;
    h.bin1_im = HFI_TWO_BY_DIR_NUM * sum1_im;
    h.inv_l_bin0   = inv_sum0 / (float)HFI_DIR_NUM;
    h.inv_l_bin2_re = HFI_TWO_BY_DIR_NUM * inv_sum2_re;
    h.inv_l_bin2_im = HFI_TWO_BY_DIR_NUM * inv_sum2_im;
    h.bin1_amp       = sqrtf(h.bin1_re * h.bin1_re + h.bin1_im * h.bin1_im);
    h.inv_l_bin2_amp = sqrtf(h.inv_l_bin2_re * h.inv_l_bin2_re + h.inv_l_bin2_im * h.inv_l_bin2_im);
}

/* ---------------- 框架接口 ---------------- */
void identify_l_Init(Identify_Context_t *ctx)
{
    (void)ctx;
    memset(&h, 0, sizeof(h));
    h.state = HFI_INIT;
    DeadComp_En = 0.0f;   /* 测原始畸变（αβ 电压模式本就旁路死区，双保险） */
}

void identify_l_Step(Identify_Context_t *ctx)
{
    float delta_i = 0.0f, project_response = 0.0f, inv_l_response = 0.0f;

    switch (h.state) {
    case HFI_INIT:
        h.v_amp = HFI_V_AMP_START;
        h.count = 0;
        h.d_axis_is_max_l = 0;   /* L_ONLY 表贴默认 */
        hfi_reset_scan_data();
        if (HFI_V_AMP_LIMIT <= HFI_EPS || HFI_I_DELTA_TARGET <= HFI_EPS ||
            HFI_PULSE_TIME <= HFI_EPS) {
            hfi_fail(ctx);
            break;
        }
        identify_apply_alpha_beta(0.0f, 0.0f);
        h.wait_count = 0;
        h.state = HFI_WAIT;
        break;

    case HFI_WAIT:              /* 静止等待（短脉冲 HFI 不会明显转动转子） */
        identify_apply_alpha_beta(0.0f, 0.0f);
        if (++h.wait_count >= HFI_WAIT_SETTLE_CNT) {
            h.wait_count = 0;
            hfi_start_amp_sweep();      /* 先幅值搜索 */
        } else if (h.wait_count >= HFI_WAIT_TIMEOUT_CNT) {
            hfi_fail(ctx);
            break;
        }
        break;

    case HFI_ZERO:              /* 零电压等待，采样基准电流 */
        identify_apply_alpha_beta(0.0f, 0.0f);
        if (++h.count >= HFI_ZERO_CNT) {
            h.count = 0;
            h.i_alpha_base = CyberDog_Motor_FOC_Y.i_alpha_meas;
            h.i_beta_base  = CyberDog_Motor_FOC_Y.i_beta_meas;
            identify_apply_alpha_beta(h.v_amp * h.cos_theta, h.v_amp * h.sin_theta);
            h.state = HFI_POS_SAMPLE;
        }
        break;

    case HFI_POS_SAMPLE:        /* +V 脉冲，采样正向电流 */
        if (++h.count >= HFI_PULSE_CNT) {
            h.count = 0;
            h.i_alpha_pos = CyberDog_Motor_FOC_Y.i_alpha_meas;
            h.i_beta_pos  = CyberDog_Motor_FOC_Y.i_beta_meas;
            identify_apply_alpha_beta(-h.v_amp * h.cos_theta, -h.v_amp * h.sin_theta);
            h.state = HFI_NEG_SAMPLE;
        }
        break;

    case HFI_NEG_SAMPLE:        /* −V 脉冲，采样反向电流并计算差分响应 */
        if (++h.count >= HFI_PULSE_CNT) {
            h.count = 0;
            h.i_alpha_neg = CyberDog_Motor_FOC_Y.i_alpha_meas;
            h.i_beta_neg  = CyberDog_Motor_FOC_Y.i_beta_meas;
            delta_i = hfi_finish_pulse_pair(ctx, &project_response, &inv_l_response);
            if (h.is_amp_search) {
                h.delta_i_sum += delta_i;
                h.delta_i_count++;
            } else {
                h.response[h.dir_idx]       = project_response;
                h.inv_l_response[h.dir_idx] = inv_l_response;
            }
            identify_apply_alpha_beta(0.0f, 0.0f);
            h.state = HFI_NEXT_DIR;
        }
        break;

    case HFI_NEXT_DIR:
        h.dir_idx++;
        if (h.dir_idx >= HFI_DIR_NUM) {
            h.dir_idx = 0;
            h.state = h.is_amp_search ? HFI_AMP_CHECK : HFI_CALC;
        } else {
            hfi_update_dir_vector();
            h.state = HFI_ZERO;
        }
        break;

    case HFI_AMP_CHECK:         /* 幅值搜索：Δi 达标则进入正式轮，否则放大 V */
        h.avg_delta_i = (h.delta_i_count > 0) ? (h.delta_i_sum / (float)h.delta_i_count) : 0.0f;
        if (h.avg_delta_i >= HFI_I_DELTA_TARGET) {
            hfi_start_result_rounds();
        } else {
            h.v_amp *= HFI_V_AMP_GAIN;
            if (h.v_amp > HFI_V_AMP_LIMIT) {
                h.v_amp = HFI_V_AMP_LIMIT;
                hfi_start_result_rounds();
            } else {
                hfi_start_amp_sweep();
            }
        }
        break;

    case HFI_CALC:
        hfi_calc_dft();
        if (h.inv_l_bin0 > HFI_EPS) {
            float g_max_now = h.inv_l_bin0 + h.inv_l_bin2_amp;
            float g_min_now = h.inv_l_bin0 - h.inv_l_bin2_amp;
            if (g_min_now > HFI_EPS && g_max_now > g_min_now) {   /* L_ONLY 阈值放松到全部计入 */
                h.inv_l_bin0_sum    += h.inv_l_bin0;
                h.inv_l_bin2_re_sum += h.inv_l_bin2_re;
                h.inv_l_bin2_im_sum += h.inv_l_bin2_im;
                h.valid_round_count++;
            }
        }
        h.result_round_idx++;
        if (h.result_round_idx < HFI_RESULT_ROUND_NUM) {
            hfi_start_scan_sweep();
            break;
        }
        /* ---- 末轮：相量平均 → Ld/Lq ---- */
        if (h.valid_round_count < 1) {
            hfi_fail(ctx);
            break;
        }
        {
            float inv_bin0_avg = h.inv_l_bin0_sum / (float)h.valid_round_count;
            float bin2_re_avg  = h.inv_l_bin2_re_sum / (float)h.valid_round_count;
            float bin2_im_avg  = h.inv_l_bin2_im_sum / (float)h.valid_round_count;
            if (inv_bin0_avg <= HFI_EPS) {
                hfi_fail(ctx);
                break;
            }
            h.Lavg = (1.0f / inv_bin0_avg) * HFI_RESULT_SCALE;
            {
                float bin2_amp = sqrtf(bin2_re_avg * bin2_re_avg + bin2_im_avg * bin2_im_avg);
                float g_max = inv_bin0_avg + bin2_amp;
                float g_min = inv_bin0_avg - bin2_amp;
                if (g_min > HFI_EPS && g_max > g_min &&
                    (g_max / g_min) >= HFI_AXIS_SALIENCY_MIN_RATIO) {
                    h.Ld = (1.0f / g_max) * HFI_RESULT_SCALE;   /* d_axis_is_max_l=0 → Ld=Lmin */
                    h.Lq = (1.0f / g_min) * HFI_RESULT_SCALE;
                } else {
                    h.Ld = h.Lavg;   /* 凸极不足 → 退化平均 */
                    h.Lq = h.Lavg;
                }
            }
        }
        ctx->res.Ld = h.Ld;
        ctx->res.Lq = h.Lq;
        identify_apply_alpha_beta(0.0f, 0.0f);
        h.state = HFI_DONE;
        break;

    case HFI_DONE:
        identify_apply_alpha_beta(0.0f, 0.0f);
        h.state = HFI_INIT;
        ctx->task_done_flag = 1;
        break;

    default:
        hfi_fail(ctx);
        break;
    }
}
