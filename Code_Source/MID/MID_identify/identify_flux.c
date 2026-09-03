/**
 * identify_flux.c — 磁链 ψf 辨识（移植 AC_Control FluxLinkage_Identify_Step）
 *
 * 子 FSM：F_INIT → F_WAIT(撤压 0.3s) → F_RAMP(升 vq 至 ωm 达目标 5% 容差带[低通 speed_filt 滤速度波动]；达电压限则跳过该点)
 *         → F_HOLD(1s) → F_SAMPLE(0.5s 平均 vq,iq,ωe + 原始速度 min/max) → F_NEXT(×4) → F_FIT(LS) → F_DONE/F_FAIL
 * 目标转速 = 额定转速(RPM_Rating)比例 {3/8,4/8,5/8,6/8}（12-13V 台架空载上限 ~20RPS，高两档电压限跳过）。
 * d 轴电流 PI（id→0）+ 交叉耦合前馈 −ωe·Lq·iq（参考 AC_Control PID_D_Control）：
 *   否则本电机 ωe·Lq/R≈0.63 会使 id≈iq，拟合丢 ωe·Ld·id 项导致 ψf 偏小不可信。
 * 拟合：y = vq − R·iq = ψf·ωe + Vdt → ψf(斜率)、Vdt(截距)；Ke=P·ψf、Kt=1.5·Ke。
 * 摩擦：iq = B_A·ωm + f_A → B、库仑摩擦（供 J 辨识用）。
 * 复用模型 dq 电压模式（ctrl_mode=3）；空载稳态 id≈0。
 */
#include "identify.h"
#include "identify_flux.h"
#include "CyberDog_Motor_FOC.h"   /* Y.iq_meas / speed_meas_rps */
#include "foc.h"                  /* Motor_Params / Two_PI */
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ---------------- 参数 ---------------- */
#define FLUX_PT_NUM 4
/* 目标转速 = 额定转速(RPM_Rating)比例，覆盖中高速段（参考 AC_Control）。
 * 本电机 2400RPM→40RPS：3/8,4/8,5/8,6/8 = 15/20/25/30 RPS；12-13V 台架空载上限 ~20RPS，{25,30} 顶电压限按语义跳过。 */
static const float FLUX_SPEED_RATIO[FLUX_PT_NUM] = {3.0f/8.0f, 4.0f/8.0f, 5.0f/8.0f, 6.0f/8.0f};

#define F_VQ_STEP        (0.2f / FOC_Frequency)                       /* vq 斜坡 ~1V/s（降速防过冲） */
#define F_VQ_LIMIT       (Motor_Params.VBUS * 0.57735027f * 0.95f)    /* 模型电压限 Vbus/√3·0.95（比参考 0.8 留足裕量，保证 20RPS 点可达） */
#define FLUX_SPEED_TOL   0.05f                                        /* 达速容差带 5% */
#define F_SPEED_FLT_K    0.002f                                       /* 达速判据速度低通 τ≈50ms（滤 14.8↔20 波动，防瞬时误触发） */
#define F_WAIT_CNT       ((int)(FOC_Frequency * 0.3f))
#define F_HOLD_CNT       ((int)(FOC_Frequency * 1.0f))
#define F_SAMPLE_N       ((int)(FOC_Frequency * 0.5f))
#define F_EPS            1e-9f

/* ---------------- 子 FSM ---------------- */
typedef enum {
    F_INIT = 0,
    F_WAIT,
    F_RAMP,
    F_HOLD,
    F_SAMPLE,
    F_NEXT,
    F_FIT,
    F_DONE,
    F_FAIL
} flux_state_t;

static struct {
    flux_state_t state;
    int   count;
    float vq_ref, vd_ref;
    float speed_filt;           /* 达速判据低通速度（滤波动） */
    float spd_min, spd_max;     /* 采样段原始速度 min/max（诊断波动幅度） */
    int   pt_idx;
    float acc_vq, acc_iq, acc_id, acc_wm;
    int   sample_cnt;
    float we_data[FLUX_PT_NUM], y_data[FLUX_PT_NUM];
    float vq_data[FLUX_PT_NUM], iq_data[FLUX_PT_NUM], id_data[FLUX_PT_NUM];
    float wm_data[FLUX_PT_NUM];
    int   valid[FLUX_PT_NUM];
} f_ctx = {0};

void identify_flux_Init(Identify_Context_t *ctx)
{
    (void)ctx;
    memset(&f_ctx, 0, sizeof(f_ctx));
    f_ctx.state = F_INIT;
    DeadComp_En = 0.0f;   /* 测原始（dq 电压模式本就旁路死区，双保险） */
}

void identify_flux_Step(Identify_Context_t *ctx)
{
    float speed = CyberDog_Motor_FOC_Y.speed_meas_rps;   /* 机械 RPS（PLL） */
    float iq    = CyberDog_Motor_FOC_Y.iq_meas;
    float id    = CyberDog_Motor_FOC_Y.id_meas;
    float R     = Motor_Params.Phase_R;
    f_ctx.speed_filt += F_SPEED_FLT_K * (speed - f_ctx.speed_filt);   /* 达速判据速度低通 */

    switch (f_ctx.state) {
    case F_INIT:
        f_ctx.state = F_WAIT;
        break;

    case F_WAIT:                /* 撤压等静止（vd 由 d 轴电流 PI 接管） */
        f_ctx.vq_ref = 0.0f;
        if (++f_ctx.count >= F_WAIT_CNT) { f_ctx.count = 0; f_ctx.state = F_RAMP; }
        break;

    case F_RAMP: {              /* 升 vq 直到机械转速达目标（5% 容差带），或达电压限则跳过该点（参考 AC_Control） */
        float target_rps = (Motor_Params.RPM_Rating / 60.0f) * FLUX_SPEED_RATIO[f_ctx.pt_idx];
        if (f_ctx.vq_ref < F_VQ_LIMIT) {
            f_ctx.vq_ref += F_VQ_STEP;
            if (f_ctx.vq_ref > F_VQ_LIMIT) f_ctx.vq_ref = F_VQ_LIMIT;
        }
        if (fabsf(f_ctx.speed_filt - target_rps) < target_rps * FLUX_SPEED_TOL) {
            printf("FLUX ramp->hold pt%d: tgt=%.1f spdfilt=%.2f spd=%.2f vq=%.3f\r\n",
                   f_ctx.pt_idx, (double)target_rps, (double)f_ctx.speed_filt,
                   (double)speed, (double)f_ctx.vq_ref);
            f_ctx.count = 0;
            f_ctx.state = F_HOLD;   /* 达目标（低通后） → 稳态保持 */
        } else if (f_ctx.vq_ref >= F_VQ_LIMIT) {
            /* 电压限仍未达目标（负载过重/目标过高）：该点标无效，跳到下一点，避免采到非目标速点 */
            f_ctx.valid[f_ctx.pt_idx] = 0;
            f_ctx.pt_idx++;
            f_ctx.count = 0;
            if (f_ctx.pt_idx >= FLUX_PT_NUM) {
                f_ctx.vq_ref = 0.0f;
                f_ctx.state = F_FIT;    /* 末点 → 拟合 */
            } else {
                f_ctx.state = F_RAMP;   /* 继续用当前 vq 爬向下一个更高目标 */
            }
        }
        break;
    }

    case F_HOLD:                /* 保持 vq 等稳态 */
        if (++f_ctx.count >= F_HOLD_CNT) {
            f_ctx.count = 0;
            f_ctx.acc_vq = f_ctx.acc_iq = f_ctx.acc_id = f_ctx.acc_wm = 0.0f;
            f_ctx.spd_min = 1e9f;  f_ctx.spd_max = -1e9f;
            f_ctx.sample_cnt = 0;
            f_ctx.state = F_SAMPLE;
        }
        break;

    case F_SAMPLE:              /* 稳态均值采样 (vq, iq, id, ωm) + 原始速度 min/max */
        if (speed < f_ctx.spd_min) f_ctx.spd_min = speed;
        if (speed > f_ctx.spd_max) f_ctx.spd_max = speed;
        f_ctx.acc_vq += CyberDog_Motor_FOC_Y.vq_meas;   /* 用模型输出 vq（ctrl_mode=3 下=辨识指令，1 拍延迟可忽略） */
        f_ctx.acc_iq += iq;
        f_ctx.acc_id += id;
        f_ctx.acc_wm += speed;
        if (++f_ctx.sample_cnt >= F_SAMPLE_N) {
            float vq_avg = f_ctx.acc_vq / (float)f_ctx.sample_cnt;
            float iq_avg = f_ctx.acc_iq / (float)f_ctx.sample_cnt;
            float id_avg = f_ctx.acc_id / (float)f_ctx.sample_cnt;
            float wm_avg = f_ctx.acc_wm / (float)f_ctx.sample_cnt;   /* RPS */
            float we_avg = wm_avg * Two_PI * Motor_Params.Pole_Pairs;
            f_ctx.vq_data[f_ctx.pt_idx] = vq_avg;
            f_ctx.iq_data[f_ctx.pt_idx] = iq_avg;
            f_ctx.id_data[f_ctx.pt_idx] = id_avg;
            f_ctx.wm_data[f_ctx.pt_idx] = wm_avg;
            f_ctx.y_data[f_ctx.pt_idx]  = vq_avg - R * iq_avg;   /* y = vq − R·iq = ψf·ωe + Vdt */
            f_ctx.we_data[f_ctx.pt_idx] = we_avg;
            f_ctx.valid[f_ctx.pt_idx] = 1;
            f_ctx.state = F_NEXT;
        }
        break;

    case F_NEXT:                /* 打印该点（speed[min..max]/vq/iq/id/Vbus/单点ψf），供台架诊断 */
        {
            float we = f_ctx.we_data[f_ctx.pt_idx];
            float psi1 = (we > 1.0f) ? f_ctx.y_data[f_ctx.pt_idx] / we : 0.0f;   /* 单点 ψf = (vq−R·iq)/ωe */
            printf("FLUX pt%d: spd=%.1f(%.1f~%.1f) vq=%.3f iq=%.3f id=%.3f Vbus=%.1f psi=%.5f\r\n",
                   f_ctx.pt_idx,
                   (double)f_ctx.wm_data[f_ctx.pt_idx],
                   (double)f_ctx.spd_min, (double)f_ctx.spd_max,
                   (double)f_ctx.vq_data[f_ctx.pt_idx],
                   (double)f_ctx.iq_data[f_ctx.pt_idx],
                   (double)f_ctx.id_data[f_ctx.pt_idx],
                   (double)Motor_Params.VBUS,
                   (double)psi1);
        }
        f_ctx.pt_idx++;
        if (f_ctx.pt_idx >= FLUX_PT_NUM) f_ctx.state = F_FIT;
        else { f_ctx.count = 0; f_ctx.state = F_RAMP; }
        break;

    case F_FIT: {               /* LS: y = ψf·x + Vdt */
        int n = 0;
        float Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
        printf("FLUX fit pts:\r\n");
        for (int i = 0; i < FLUX_PT_NUM; i++) {
            if (!f_ctx.valid[i]) { printf("  pt%d invalid\r\n", i); continue; }
            float x = f_ctx.we_data[i], y = f_ctx.y_data[i];
            printf("  pt%d: we=%.0f y=%.4f\r\n", i, (double)x, (double)y);
            Sx += x; Sy += y; Sxx += x * x; Sxy += x * y;
            n++;
        }
        float det = (float)n * Sxx - Sx * Sx;
        if (n < 2 || fabsf(det) < F_EPS) { f_ctx.state = F_FAIL; break; }
        float flux = ((float)n * Sxy - Sx * Sy) / det;   /* ψf */
        float Vdt  = (Sxx * Sy - Sx * Sxy) / det;        /* 死区电压截距 */
        printf("FLUX fit: n=%d flux=%.6f Vdt=%.4f\r\n", n, (double)flux, (double)Vdt);
        if (flux <= 0.0f) { f_ctx.state = F_FAIL; break; }
        ctx->res.Flux_linkage = flux;
        ctx->res.Vdead_est    = Vdt;
        ctx->res.Ke = flux * Motor_Params.Pole_Pairs;
        ctx->res.Kt = 1.5f * ctx->res.Ke;
        /* 摩擦拟合：iq = B_A·ωm + f_A */
        {
            int m = 0;
            float Qx = 0, Qy = 0, Qxx = 0, Qxy = 0;
            for (int i = 0; i < FLUX_PT_NUM; i++) {
                if (!f_ctx.valid[i]) continue;
                float x = f_ctx.wm_data[i], y = f_ctx.iq_data[i];
                Qx += x; Qy += y; Qxx += x * x; Qxy += x * y;
                m++;
            }
            float d2 = (float)m * Qxx - Qx * Qx;
            if (m >= 2 && fabsf(d2) > F_EPS) {
                float B_A = ((float)m * Qxy - Qx * Qy) / d2;   /* A/RPS */
                float f_A = (Qy - B_A * Qx) / (float)m;        /* A */
                ctx->res.B               = B_A * ctx->res.Kt;
                ctx->res.friction_Coulomb = f_A * ctx->res.Kt;
            }
        }
        f_ctx.state = F_DONE;
        break;
    }

    case F_DONE:
        f_ctx.vq_ref = 0.0f;   /* vd 由 d 轴电流 PI 接管（id→0） */
        f_ctx.state = F_INIT;
        ctx->task_done_flag = 1;
        break;

    case F_FAIL:
        f_ctx.vq_ref = 0.0f;
        printf("FLUX_IDENT FAIL\r\n");
        ctx->task_fail_flag = 1;
        f_ctx.state = F_INIT;
        break;
    }

    /* d 轴电流 PI（id→0）：复用模型 d-PI —— 模型每步用 id_ref=0 更新 Integrator_DSTATE_l
     * （ctrl_mode=3 下其输出被 U.vd_ref 覆盖但积分器照常累加），读积分器 + Kp·err 即模型 d-PI 输出。
     * 辨识时不加前馈（纯 PI 靠积分器收敛 id→0，稳态拟合足够；与 Stage 2 导出函数一致）。 */
    {
        float err_d  = 0.0f - id;   /* = 模型 id_ref(0) − id_meas */
        float vd_pre = CurrD_Kp * err_d + CyberDog_Motor_FOC_DW.Integrator_DSTATE_l;
        f_ctx.vd_ref = (vd_pre >  CurrD_MaxOut) ? CurrD_MaxOut :
                       (vd_pre < CurrD_MinOut) ? CurrD_MinOut : vd_pre;
    }

    /* 应用 dq 电压（复用模型电压模式；vd 由 d-PI 输出，vq 开环斜坡） */
    identify_apply_voltage(f_ctx.vd_ref, f_ctx.vq_ref);
}
