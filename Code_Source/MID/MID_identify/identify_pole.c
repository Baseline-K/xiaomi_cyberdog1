/**
 * identify_pole.c — 极对数/方向辨识（移植 AC_Control Para_identify_PoleDirection）
 *
 * 子 FSM：INIT → WAIT_STOP(等静止) → VOLT_RAMP(升电压至电流≥0.2·额定吸合)
 *   → ALIGN_HOLD(锁角等稳定) → FORWARD_SCAN(受控电角正扫至机械角一整圈)
 *   → TURN_HOLD → REVERSE_SCAN(反扫相同电角行程) → VOLT_RAMP_DOWN(撤压)
 *   → CALC(p=round(½(p_f+p_r))、dir=sign(Δθe·Δθm)、质量门限) → DONE；
 *   采集阶段失败 → FAIL_RAMP_DOWN → FAIL。
 * 复用模型 dq 电压模式（ctrl_mode=3）+ 开环电角度覆盖（反 Park 用受控角，非编码器角）。
 * 读编码器机械角（Encoder_AS5600.angle 解包）、机械速度（Speed_RPM/60）、电流幅值（Y.i_alpha/beta_meas）。
 * 结果存 ctx->res.pole_pairs_identified/control_to_encoder_dir，不自动写回 Motor_Params。
 */
#include "identify.h"
#include "identify_pole.h"
#include "CyberDog_Motor_FOC.h"   /* Y.i_alpha_meas / i_beta_meas */
#include "AS5600.h"               /* Encoder_AS5600.angle / Speed_RPM */
#include "foc.h"                  /* Motor_Params / Two_PI / FOC_Frequency */
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ---------------- 参数（参考 PoleDirection 默认） ---------------- */
#define POLE_DIR_P_TOL              0.25f
#define POLE_DIR_EPS                1.0e-6f
#define POLE_DIR_CURRENT_TARGET_FRAC 0.2f   /* 目标电流 = 0.2·额定（吸合判据） */
#define POLE_DIR_VOLT_LIMIT_FRAC    0.15f   /* 电压搜索上限 = 0.15·VBUS */
#define POLE_DIR_VOLT_RAMP_TIME_S   2.0f
#define POLE_DIR_CURRENT_LPF_ALPHA  0.02f
#define POLE_DIR_SPEED_E_REV_S      0.8f    /* 电角扫速 0.8 电圈/s */
#define POLE_DIR_OMEGA_E_CMD        (POLE_DIR_SPEED_E_REV_S * Two_PI)
#define POLE_DIR_STOP_SPEED_RPS     1.0f
#define POLE_DIR_STOP_STABLE_CNT    ((int)(FOC_Frequency * 0.10f))
#define POLE_DIR_CURRENT_STABLE_CNT ((int)(FOC_Frequency * 0.05f))
#define POLE_DIR_ALIGN_HOLD_CNT     ((int)(FOC_Frequency * 0.30f))
#define POLE_DIR_WAIT_TIMEOUT_CNT   ((int)(FOC_Frequency * 5.0f))
#define POLE_DIR_VOLT_TIMEOUT_CNT   ((int)(FOC_Frequency * 3.0f))
#define POLE_DIR_ALIGN_TIMEOUT_CNT  ((int)(FOC_Frequency * 1.0f))
#define POLE_DIR_SCAN_TIMEOUT_CNT   ((int)(FOC_Frequency * 30.0f))
#define POLE_PI                     3.14159265f

/* ---------------- 子 FSM ---------------- */
typedef enum {
    POLE_DIR_INIT = 0,
    POLE_DIR_WAIT_STOP,
    POLE_DIR_VOLT_RAMP,
    POLE_DIR_ALIGN_HOLD,
    POLE_DIR_FORWARD_SCAN,
    POLE_DIR_TURN_HOLD,
    POLE_DIR_REVERSE_SCAN,
    POLE_DIR_VOLT_RAMP_DOWN,
    POLE_DIR_CALC,
    POLE_DIR_DONE,
    POLE_DIR_FAIL_RAMP_DOWN,
    POLE_DIR_FAIL
} pole_dir_state_t;

static struct {
    pole_dir_state_t state;
    int   state_cnt;
    int   stable_cnt;
    float voltage_amp;
    float current_mag_lpf;
    float theta_cmd_unwrapped;      /* 连续累计受控电角（不取模，角度差用） */
    float mec_last;
    float mec_unwrapped;            /* 编码器机械角解包 */
    float forward_start_ele, forward_start_mec;
    float forward_delta_ele, forward_delta_mec;
    float reverse_start_ele, reverse_start_mec;
    float reverse_delta_ele, reverse_delta_mec;
    uint8_t forward_full_turn;
} p_ctx = {0};

/* ---------------- 辅助 ---------------- */
static inline float pole_dir_step_towards(float x, float target, float step)
{
    if (x < target) { x += step; if (x > target) x = target; }
    else if (x > target) { x -= step; if (x < target) x = target; }
    return x;
}

static float pole_dir_wrap_0_2pi(float angle)
{
    angle = fmodf(angle, (float)Two_PI);
    if (angle < 0.0f) angle += (float)Two_PI;
    return angle;
}

/* 应用电压 + 开环电角度（反 Park 用受控角；vd=幅值、vq=0 强拖沿受控角 d 轴） */
static void pole_dir_apply_voltage(void)
{
    identify_set_eleangle_override(pole_dir_wrap_0_2pi(p_ctx.theta_cmd_unwrapped));
    identify_apply_voltage(p_ctx.voltage_amp, 0.0f);
}

/* 编码器机械角解包（±π 判环绕） */
static void pole_dir_update_mec_unwrapped(void)
{
    float mec_now = Encoder_AS5600.angle;
    float mec_delta = mec_now - p_ctx.mec_last;
    if (mec_delta > POLE_PI) mec_delta -= (float)Two_PI;
    else if (mec_delta < -POLE_PI) mec_delta += (float)Two_PI;
    p_ctx.mec_unwrapped += mec_delta;
    p_ctx.mec_last = mec_now;
}

static int8_t pole_dir_sign(float value)
{
    if (value > 0.0f) return 1;
    if (value < 0.0f) return -1;
    return 0;
}

static void pole_dir_start_fail_ramp(void)
{
    if (p_ctx.state != POLE_DIR_FAIL_RAMP_DOWN && p_ctx.state != POLE_DIR_FAIL &&
        p_ctx.state != POLE_DIR_DONE) {
        p_ctx.state = POLE_DIR_FAIL_RAMP_DOWN;
        p_ctx.state_cnt = 0; p_ctx.stable_cnt = 0;
    }
}

/* 运行时有效性：仅校验辨识所需输入（VBUS/额定电流）；Safety 由外层负责 */
static int pole_dir_runtime_invalid(void)
{
    if (Motor_Params.VBUS <= POLE_DIR_EPS || Motor_Params.Current_Rating <= POLE_DIR_EPS) return 1;
    return 0;
}

/* ---------------- 任务入口 ---------------- */
void identify_pole_Init(Identify_Context_t *ctx)
{
    memset(&p_ctx, 0, sizeof(p_ctx));
    p_ctx.state = POLE_DIR_INIT;
    p_ctx.mec_last = Encoder_AS5600.angle;
    /* 结果清零（不写回 Motor_Params） */
    ctx->res.pole_pairs_identified = 0U;
    ctx->res.pole_pairs_valid      = 0U;
    ctx->res.control_to_encoder_dir = 0;
    ctx->res.direction_valid       = 0U;
    DeadComp_En = 0.0f;   /* 测原始（电压模式本就旁路死区） */
}

void identify_pole_Step(Identify_Context_t *ctx)
{
    const float voltage_limit  = Motor_Params.VBUS * POLE_DIR_VOLT_LIMIT_FRAC;
    const float voltage_step   = voltage_limit / (FOC_Frequency * POLE_DIR_VOLT_RAMP_TIME_S);
    const float current_target = Motor_Params.Current_Rating * POLE_DIR_CURRENT_TARGET_FRAC;
    const float ele_step       = POLE_DIR_OMEGA_E_CMD / FOC_Frequency;   /* 每拍电角增量 */
    float speed_rps = Encoder_AS5600.Speed_RPM / 60.0f;   /* 编码器机械 RPS */
    float current_mag;

    if (p_ctx.state != POLE_DIR_FAIL_RAMP_DOWN && p_ctx.state != POLE_DIR_FAIL &&
        p_ctx.state != POLE_DIR_DONE && pole_dir_runtime_invalid()) {
        pole_dir_start_fail_ramp();
    }

    pole_dir_update_mec_unwrapped();

    switch (p_ctx.state) {
    case POLE_DIR_INIT:
        p_ctx.voltage_amp = 0.0f;
        p_ctx.theta_cmd_unwrapped = 0.0f;
        p_ctx.mec_last = Encoder_AS5600.angle;
        p_ctx.mec_unwrapped = 0.0f;
        p_ctx.state_cnt = 0; p_ctx.stable_cnt = 0;
        pole_dir_apply_voltage();
        p_ctx.state = POLE_DIR_WAIT_STOP;
        break;

    case POLE_DIR_WAIT_STOP:      /* 等机械速度低于静止阈值 */
        p_ctx.state_cnt++;
        pole_dir_apply_voltage();
        if (fabsf(speed_rps) <= POLE_DIR_STOP_SPEED_RPS) {
            p_ctx.stable_cnt++;
            if (p_ctx.stable_cnt >= POLE_DIR_STOP_STABLE_CNT) {
                p_ctx.state = POLE_DIR_VOLT_RAMP; p_ctx.state_cnt = 0; p_ctx.stable_cnt = 0;
                p_ctx.current_mag_lpf = 0.0f;
            }
        } else { p_ctx.stable_cnt = 0; }
        if (p_ctx.state_cnt >= POLE_DIR_WAIT_TIMEOUT_CNT) pole_dir_start_fail_ramp();
        break;

    case POLE_DIR_VOLT_RAMP:      /* 升电压至电流幅值达吸合目标 */
        p_ctx.state_cnt++;
        p_ctx.voltage_amp = pole_dir_step_towards(p_ctx.voltage_amp, voltage_limit, voltage_step);
        pole_dir_apply_voltage();
        current_mag = sqrtf(CyberDog_Motor_FOC_Y.i_alpha_meas * CyberDog_Motor_FOC_Y.i_alpha_meas +
                            CyberDog_Motor_FOC_Y.i_beta_meas  * CyberDog_Motor_FOC_Y.i_beta_meas);
        p_ctx.current_mag_lpf += POLE_DIR_CURRENT_LPF_ALPHA * (current_mag - p_ctx.current_mag_lpf);
        if (p_ctx.current_mag_lpf >= current_target) {
            p_ctx.stable_cnt++;
            if (p_ctx.stable_cnt >= POLE_DIR_CURRENT_STABLE_CNT) {
                p_ctx.state = POLE_DIR_ALIGN_HOLD; p_ctx.state_cnt = 0; p_ctx.stable_cnt = 0;
            }
        } else {
            p_ctx.stable_cnt = 0;
            if (p_ctx.voltage_amp >= voltage_limit) pole_dir_start_fail_ramp();   /* 电压达上限仍无吸合电流 */
        }
        if (p_ctx.state_cnt >= POLE_DIR_VOLT_TIMEOUT_CNT) pole_dir_start_fail_ramp();
        break;

    case POLE_DIR_ALIGN_HOLD:     /* 锁角等转子稳定 */
        p_ctx.state_cnt++;
        pole_dir_apply_voltage();
        if (fabsf(speed_rps) <= POLE_DIR_STOP_SPEED_RPS) p_ctx.stable_cnt++;
        else p_ctx.stable_cnt = 0;
        if (p_ctx.state_cnt >= POLE_DIR_ALIGN_HOLD_CNT && p_ctx.stable_cnt >= POLE_DIR_STOP_STABLE_CNT) {
            p_ctx.forward_start_ele = p_ctx.theta_cmd_unwrapped;
            p_ctx.forward_start_mec = p_ctx.mec_unwrapped;
            p_ctx.forward_delta_ele = 0.0f; p_ctx.forward_delta_mec = 0.0f;
            p_ctx.forward_full_turn = 0U;
            p_ctx.state_cnt = 0; p_ctx.stable_cnt = 0;
            p_ctx.state = POLE_DIR_FORWARD_SCAN;
        } else if (p_ctx.state_cnt >= POLE_DIR_ALIGN_TIMEOUT_CNT) {
            pole_dir_start_fail_ramp();
        }
        break;

    case POLE_DIR_FORWARD_SCAN:   /* 正扫：受控电角递增，机械角解包累计到一整圈 */
        p_ctx.state_cnt++;
        p_ctx.theta_cmd_unwrapped += ele_step;
        pole_dir_apply_voltage();
        p_ctx.forward_delta_ele = p_ctx.theta_cmd_unwrapped - p_ctx.forward_start_ele;
        p_ctx.forward_delta_mec = p_ctx.mec_unwrapped - p_ctx.forward_start_mec;
        if (fabsf(p_ctx.forward_delta_mec) >= (float)Two_PI) {
            p_ctx.forward_full_turn = 1U;
            p_ctx.state = POLE_DIR_TURN_HOLD; p_ctx.state_cnt = 0; p_ctx.stable_cnt = 0;
        } else if (p_ctx.state_cnt >= POLE_DIR_SCAN_TIMEOUT_CNT) {
            pole_dir_start_fail_ramp();
        }
        break;

    case POLE_DIR_TURN_HOLD:      /* 保持终点电角等机械停止 */
        p_ctx.state_cnt++;
        pole_dir_apply_voltage();
        if (fabsf(speed_rps) <= POLE_DIR_STOP_SPEED_RPS) {
            p_ctx.stable_cnt++;
            if (p_ctx.stable_cnt >= POLE_DIR_STOP_STABLE_CNT) {
                p_ctx.forward_delta_mec = p_ctx.mec_unwrapped - p_ctx.forward_start_mec;
                p_ctx.reverse_start_ele = p_ctx.theta_cmd_unwrapped;
                p_ctx.reverse_start_mec = p_ctx.mec_unwrapped;
                p_ctx.state = POLE_DIR_REVERSE_SCAN; p_ctx.state_cnt = 0; p_ctx.stable_cnt = 0;
            }
        } else { p_ctx.stable_cnt = 0; }
        if (p_ctx.state_cnt >= POLE_DIR_ALIGN_TIMEOUT_CNT) pole_dir_start_fail_ramp();
        break;

    case POLE_DIR_REVERSE_SCAN:   /* 反扫：受控电角递减相同行程 */
        p_ctx.state_cnt++;
        p_ctx.theta_cmd_unwrapped -= ele_step;
        pole_dir_apply_voltage();
        p_ctx.reverse_delta_ele = p_ctx.theta_cmd_unwrapped - p_ctx.reverse_start_ele;
        p_ctx.reverse_delta_mec = p_ctx.mec_unwrapped - p_ctx.reverse_start_mec;
        if (fabsf(p_ctx.reverse_delta_ele) >= fabsf(p_ctx.forward_delta_ele)) {
            p_ctx.state = POLE_DIR_VOLT_RAMP_DOWN; p_ctx.state_cnt = 0;
        } else if (p_ctx.state_cnt >= POLE_DIR_SCAN_TIMEOUT_CNT) {
            pole_dir_start_fail_ramp();
        }
        break;

    case POLE_DIR_VOLT_RAMP_DOWN: /* 受控降压至零 */
        p_ctx.voltage_amp = pole_dir_step_towards(p_ctx.voltage_amp, 0.0f, voltage_step);
        pole_dir_apply_voltage();
        if (p_ctx.voltage_amp <= POLE_DIR_EPS) {
            p_ctx.voltage_amp = 0.0f; pole_dir_apply_voltage();
            p_ctx.state = POLE_DIR_CALC;
        }
        break;

    case POLE_DIR_CALC: {         /* 极对数 + 方向 + 质量门限 */
        float pole_forward = 0.0f, pole_reverse = 0.0f, pole_average, pole_rounded;
        int8_t dir_f, dir_r;
        int quality_ok = 1;
        if (fabsf(p_ctx.forward_delta_mec) > POLE_DIR_EPS)
            pole_forward  = fabsf(p_ctx.forward_delta_ele / p_ctx.forward_delta_mec);
        if (fabsf(p_ctx.reverse_delta_mec) > POLE_DIR_EPS)
            pole_reverse  = fabsf(p_ctx.reverse_delta_ele / p_ctx.reverse_delta_mec);
        pole_average = 0.5f * (pole_forward + pole_reverse);
        pole_rounded = floorf(pole_average + 0.5f);
        dir_f = pole_dir_sign(p_ctx.forward_delta_ele  * p_ctx.forward_delta_mec);
        dir_r = pole_dir_sign(p_ctx.reverse_delta_ele * p_ctx.reverse_delta_mec);

        ctx->res.pole_pairs_identified = (pole_rounded > 65535.0f) ? 65535U : (uint16_t)pole_rounded;
        ctx->res.control_to_encoder_dir = dir_f;
        printf("POLE: pf=%.3f pr=%.3f P=%u dir_f=%d dir_r=%d\r\n",
               (double)pole_forward, (double)pole_reverse,
               (unsigned)ctx->res.pole_pairs_identified, (int)dir_f, (int)dir_r);

        if (ctx->res.pole_pairs_identified < 1U ||
            p_ctx.forward_full_turn == 0U ||
            fabsf(p_ctx.forward_delta_mec) <= POLE_DIR_EPS ||
            fabsf(p_ctx.reverse_delta_mec) <= POLE_DIR_EPS ||
            dir_f == 0 || dir_f != dir_r ||
            fabsf(pole_forward  - pole_rounded) > POLE_DIR_P_TOL ||
            fabsf(pole_reverse - pole_rounded) > POLE_DIR_P_TOL) {
            quality_ok = 0;
        }
        if (quality_ok) {
            ctx->res.pole_pairs_valid = 1U; ctx->res.direction_valid = 1U;
            ctx->task_done_flag = 1U; p_ctx.state = POLE_DIR_DONE;
        } else {
            ctx->task_fail_flag = 1U; p_ctx.state = POLE_DIR_FAIL;
        }
        break;
    }

    case POLE_DIR_DONE:
        p_ctx.voltage_amp = 0.0f;
        identify_apply_voltage(0.0f, 0.0f);
        identify_clear_eleangle_override();
        break;

    case POLE_DIR_FAIL_RAMP_DOWN:
        p_ctx.voltage_amp = pole_dir_step_towards(p_ctx.voltage_amp, 0.0f, voltage_step);
        pole_dir_apply_voltage();
        if (p_ctx.voltage_amp <= POLE_DIR_EPS) {
            p_ctx.voltage_amp = 0.0f; pole_dir_apply_voltage();
            identify_clear_eleangle_override();
            p_ctx.state = POLE_DIR_FAIL; ctx->task_fail_flag = 1U;
        }
        break;

    case POLE_DIR_FAIL:
    default:
        p_ctx.voltage_amp = 0.0f;
        identify_apply_voltage(0.0f, 0.0f);
        identify_clear_eleangle_override();
        break;
    }
}
