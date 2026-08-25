/**
 * Safety_Module.c
 *
 * 快/慢两级安全保护实现（纯计算，不碰 HAL/RTOS/printf）。
 * NaN/Inf 输入视为故障；毫秒累计用饱和加法防长时间运行溢出。
 *
 * 对应《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§6。
 */
#include "Safety_Module.h"
#include "foc.h"          /* Motor_Params */
#include <math.h>

SafetyConfig_t Safety_Config;

/* 测试强制位（仅调试） */
static volatile uint32_t g_safety_test_force = 0U;

void Safety_TestForce(uint32_t mask)  { g_safety_test_force |= mask; }
void Safety_TestClear(uint32_t mask)  { g_safety_test_force &= ~mask; }

/* 饱和加法：a+b 溢出时返回最大值 */
static uint32_t satadd(uint32_t a, uint32_t b)
{
    uint32_t sum = a + b;
    return (sum < a) ? 0xFFFFFFFFUL : sum;
}

void Safety_Config_Init(SafetyConfig_t *cfg)
{
    (void)0;
    *cfg = (SafetyConfig_t){
        .v_bus_over_V          = 14.5f,                       /* 12V 母线过压 */
        .v_bus_under_V         = 9.0f,
        .vbus_under_hyst_V     = 1.0f,
        .vbus_under_thr_ms     = 500U,
        .current_over_A        = 3.0f,   /* 高于典型启动电流，低于堵转(12V/3.2Ω≈3.75A) */
        .shortcircuit_A        = 5.0f,   /* 高于堵转，异常大电流视为短路 */
        .locked_current_thr_A  = Motor_Params.Current_Rating * 0.8f,
        .locked_speed_thr_rps  = 0.5f,
        .locked_thr_ms         = 2000U,
        .speederror_thr_rps    = 2.0f,
        .speederror_thr_ms     = 2000U,
        .comm_thr_ms           = 1000U,
        .enable_overcurrent    = true,
        .enable_overvolt       = true,
        .enable_undervolt      = true,
        .enable_stall          = true,
        .enable_speed_err      = true,
        .enable_comm           = true,
    };
}

void Safety_FastStep(const SafetyFastInput_t *input,
                     const SafetyConfig_t *cfg,
                     fault_mask_t *new_faults)
{
    fault_mask_t f = 0;
    float ia = input->phase_current_A[0];
    float ib = input->phase_current_A[1];
    float ic = input->phase_current_A[2];
    float vbus = input->vbus_V;

    /* NaN/Inf：非有限值按紧急处理 */
    if (!isfinite(ia) || !isfinite(ib) || !isfinite(ic) || !isfinite(vbus)) {
        f |= FAULT_OVERCURRENT_MASK;
    } else {
        if (cfg->enable_overcurrent) {
            float imax = fmaxf(fmaxf(fabsf(ia), fabsf(ib)), fabsf(ic));
            if (imax > cfg->shortcircuit_A) {
                f |= FAULT_SHORTCIRCUIT_MASK;
            } else if (imax > cfg->current_over_A) {
                f |= FAULT_OVERCURRENT_MASK;
            }
        }
        if (cfg->enable_overvolt && (vbus > cfg->v_bus_over_V)) {
            f |= FAULT_OVERVOLT_MASK;
        }
    }

    /* 测试强制（调试用，正常为 0） */
    if (g_safety_test_force & SAFETY_TEST_FORCE_OVERCURRENT) f |= FAULT_OVERCURRENT_MASK;
    if (g_safety_test_force & SAFETY_TEST_FORCE_OVERVOLT)    f |= FAULT_OVERVOLT_MASK;

    *new_faults = f;
}

void Safety_SlowStep(const SafetySlowInput_t *in,
                     const SafetyConfig_t *cfg,
                     SafetyRuntime_t *rt,
                     fault_mask_t *new_faults)
{
    fault_mask_t f = 0;
    uint32_t dt = in->elapsed_ms;
    float ia = in->measurement.phase_current_A[0];
    float ib = in->measurement.phase_current_A[1];
    float ic = in->measurement.phase_current_A[2];
    float vbus = in->measurement.vbus_V;
    float speed = fabsf(in->measurement.motor_speed_rps);
    float target = in->measurement.target_speed_rps;

    /* NaN/Inf 防护 */
    if (!isfinite(ia) || !isfinite(ib) || !isfinite(ic) || !isfinite(vbus) ||
        !isfinite(speed) || !isfinite(target)) {
        f |= FAULT_UNDERVOLT_MASK;   /* 数值异常按常规故障处理 */
    }

    /* 通信超时：仅在运行态 AND 已建立过 CAN 通信（seq>0）时检测；
     * RTT 控制 / 上电 IDLE / 从未收到 CAN 命令 → 不检测，累计清零。 */
    if (cfg->enable_comm && in->is_run && (in->measurement.can_rx_valid_seq > 0U)) {
        if (in->measurement.can_rx_valid_seq == rt->last_can_rx_seq) {
            rt->comm_accum_ms = satadd(rt->comm_accum_ms, dt);
        } else {
            rt->comm_accum_ms = 0U;
            rt->last_can_rx_seq = in->measurement.can_rx_valid_seq;
        }
        if (rt->comm_accum_ms >= cfg->comm_thr_ms) f |= FAULT_COMM_TIMEOUT_MASK;
    } else {
        rt->comm_accum_ms = 0U;   /* 非运行/无 CAN：清零，避免跨次运行残留 */
        rt->last_can_rx_seq = in->measurement.can_rx_valid_seq;
    }

    if (in->is_run) {
        /* 欠压（带滞回） */
        if (cfg->enable_undervolt) {
            if (vbus < cfg->v_bus_under_V) {
                rt->vbus_under_accum_ms = satadd(rt->vbus_under_accum_ms, dt);
            } else if (vbus > (cfg->v_bus_under_V + cfg->vbus_under_hyst_V)) {
                rt->vbus_under_accum_ms = 0U;   /* 回到阈值+回差以上才清零 */
            }
            if (rt->vbus_under_accum_ms >= cfg->vbus_under_thr_ms) f |= FAULT_UNDERVOLT_MASK;
        }

        /* 堵转：电流大 + 转速低 + 持续 */
        if (cfg->enable_stall) {
            float imax = fmaxf(fmaxf(fabsf(ia), fabsf(ib)), fabsf(ic));
            if ((imax > cfg->locked_current_thr_A) && (speed < cfg->locked_speed_thr_rps)) {
                rt->stall_accum_ms = satadd(rt->stall_accum_ms, dt);
            } else {
                rt->stall_accum_ms = 0U;
            }
            if (rt->stall_accum_ms >= cfg->locked_thr_ms) f |= FAULT_STALL_MASK;
        }

        /* 转速异常：目标/实际偏差 + 持续 */
        if (cfg->enable_speed_err) {
            if (fabsf(speed - target) > cfg->speederror_thr_rps) {
                rt->speed_err_accum_ms = satadd(rt->speed_err_accum_ms, dt);
            } else {
                rt->speed_err_accum_ms = 0U;
            }
            if (rt->speed_err_accum_ms >= cfg->speederror_thr_ms) f |= FAULT_SPEED_ERR_MASK;
        }
    } else {
        /* 非运行态：常规累计清零（欠压/堵转/转速异常只在运行态判定） */
        rt->vbus_under_accum_ms = 0U;
        rt->stall_accum_ms = 0U;
        rt->speed_err_accum_ms = 0U;
    }

    /* 测试强制 */
    if (g_safety_test_force & SAFETY_TEST_FORCE_UNDERVOLT) f |= FAULT_UNDERVOLT_MASK;

    *new_faults = f;
}
