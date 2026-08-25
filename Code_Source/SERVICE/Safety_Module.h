/**
 * Safety_Module.h
 *
 * 快/慢两级安全保护（参照参考工程 Safety_Module，仅保留本项目硬件支持的检测器）。
 * - Safety_FastStep():  FOC ISR 内，10kHz，本周期 vbus+三相电流（紧急：过流/过压）
 * - Safety_SlowStep():  SafetyTask 内，3ms 绝对周期（常规：欠压/堵转/转速异常/通信超时）
 * - 核心函数纯计算，不碰 HAL/RTOS/printf；NaN/Inf 视为故障。
 *
 * 对应《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§6。
 */
#ifndef SAFETY_MODULE_H
#define SAFETY_MODULE_H

#include <stdint.h>
#include <stdbool.h>

/*-----------------------------------------------------------
 * 故障掩码（参考工程，紧急/常规两组）
 *----------------------------------------------------------*/
#define FAULT_OVERCURRENT_MASK   (1u << 0)   /* 过流 */
#define FAULT_SHORTCIRCUIT_MASK  (1u << 1)   /* 短路 */
#define FAULT_OVERVOLT_MASK      (1u << 2)   /* 过压 */
#define FAULT_UNDERVOLT_MASK     (1u << 8)   /* 欠压 */
#define FAULT_COMM_TIMEOUT_MASK  (1u << 13)  /* 通信超时 */
#define FAULT_ENCODER_MASK       (1u << 14)  /* 编码器 */
#define FAULT_STALL_MASK         (1u << 18)  /* 堵转 */
#define FAULT_SPEED_ERR_MASK     (1u << 19)  /* 转速异常 */

#define FAULT_EMERGENCY_MASK (FAULT_OVERCURRENT_MASK | FAULT_SHORTCIRCUIT_MASK | FAULT_OVERVOLT_MASK)

typedef uint32_t fault_mask_t;

/*-----------------------------------------------------------
 * 输入结构
 *----------------------------------------------------------*/
typedef struct {
    float vbus_V;
    float phase_current_A[3];
} SafetyFastInput_t;

typedef struct {
    float vbus_V;
    float phase_current_A[3];
    float motor_speed_rps;
    float target_speed_rps;
    float target_iq_A;
    uint32_t can_rx_valid_seq;
} SafetyMeasurement_t;

typedef struct {
    uint32_t elapsed_ms;
    uint8_t  is_run;
    SafetyMeasurement_t measurement;
} SafetySlowInput_t;

/*-----------------------------------------------------------
 * 配置（运行期只读）与运行期累计（Safety 独占）
 *----------------------------------------------------------*/
typedef struct {
    float    v_bus_over_V;         /* 过压阈值 V */
    float    v_bus_under_V;        /* 欠压阈值 V */
    float    vbus_under_hyst_V;    /* 欠压回差 V（高于 阈值+回差 才清累计） */
    uint32_t vbus_under_thr_ms;    /* 欠压持续时间 ms */
    float    current_over_A;       /* 过流阈值 A */
    float    shortcircuit_A;       /* 短路阈值 A */
    float    locked_current_thr_A; /* 堵转电流阈值 A */
    float    locked_speed_thr_rps; /* 堵转速度阈值 RPS */
    uint32_t locked_thr_ms;        /* 堵转持续时间 ms */
    float    speederror_thr_rps;   /* 转速偏差阈值 RPS */
    uint32_t speederror_thr_ms;    /* 转速异常持续 ms */
    uint32_t comm_thr_ms;          /* 通信超时阈值 ms */
    bool enable_overcurrent;
    bool enable_overvolt;
    bool enable_undervolt;
    bool enable_stall;
    bool enable_speed_err;
    bool enable_comm;
} SafetyConfig_t;

typedef struct {
    uint32_t vbus_under_accum_ms;
    uint32_t stall_accum_ms;
    uint32_t speed_err_accum_ms;
    uint32_t comm_accum_ms;
    uint32_t last_can_rx_seq;
} SafetyRuntime_t;

/*-----------------------------------------------------------
 * API
 *----------------------------------------------------------*/
void Safety_Config_Init(SafetyConfig_t *cfg);   /* 阈值从 Motor_Params 推算，运行期只读 */

/* 快检：返回本周期新增紧急故障位（纯计算） */
void Safety_FastStep(const SafetyFastInput_t *input,
                     const SafetyConfig_t *cfg,
                     fault_mask_t *new_faults);

/* 慢检：返回本周期新增常规故障位（纯计算；rt 为运行期累计，Safety 独占） */
void Safety_SlowStep(const SafetySlowInput_t *input,
                     const SafetyConfig_t *cfg,
                     SafetyRuntime_t *rt,
                     fault_mask_t *new_faults);

/* 测试强制位（仅调试：让指定检测器强制报故障，便于验证紧急关断路径；发布移除 RTT 命令） */
#define SAFETY_TEST_FORCE_OVERCURRENT  (1u << 0)
#define SAFETY_TEST_FORCE_OVERVOLT     (1u << 1)
#define SAFETY_TEST_FORCE_UNDERVOLT    (1u << 2)
void Safety_TestForce(uint32_t mask);
void Safety_TestClear(uint32_t mask);

extern SafetyConfig_t Safety_Config;

#endif /* SAFETY_MODULE_H */
