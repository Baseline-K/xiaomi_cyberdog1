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
/* 非"检测器算出来"的故障：由软件/硬件状态直接上报（不在 Safety_FastStep/SlowStep 里产生）
 *   1u << 3 : 栅极驱动 nFAULT（DRV8323 FSR1/FSR2）—— 计划 Batch 3
 *   1u << 4 : 任务看门狗（Task_Watchdog.c 判某任务卡死后 PostFault）
 * 这两个位都**不进** g_recovery_mask：它们不需要定向硬件重初始化，
 * 由状态机 state_FAULT_do 走"无需恢复"分支，直接 FAULT_NOW → FAULT_OVER → IDLE。 */
#define FAULT_GATE_DRIVER_MASK   (1u << 3)   /* 栅极驱动故障（DRV8323 FSR1/FSR2） */
#define FAULT_TASK_WATCHDOG_MASK (1u << 4)   /* 任务看门狗：某任务卡死 */
#define FAULT_CURRENT_SENSE_MASK (1u << 5)   /* 电流采样链故障（零偏校准未完成等） */

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
/* 仿 AC_Control：过压/欠压/转速异常用「超量积分」——acc += (实测 − 阈值)×dt，饱和到
 * [0, trip]，acc≥trip 触发；回到安全侧 delta<0 自然漏泄回 0（无独立回差）。
 * 过压另留瞬时硬闸 v_bus_over_V 在 FastStep 兜底（本工程无 ADC AWD）。 */
typedef struct {
    float    v_bus_over_V;         /* 过压瞬时硬闸 V（FastStep 立即，极端档） */
    float    vbus_over_soft_V;     /* 过压软积分起点 V（>它开始积 V·ms） */
    float    vbus_over_trip_Vms;   /* 过压积分门限 V·ms */
    float    v_bus_under_V;        /* 欠压积分起点 V（<它开始积 V·ms） */
    float    vbus_under_trip_Vms;  /* 欠压积分门限 V·ms */
    float    current_over_A;       /* 过流阈值 A */
    float    shortcircuit_A;       /* 短路阈值 A */
    float    locked_current_thr_A; /* 堵转电流阈值 A */
    float    locked_speed_thr_rps; /* 堵转速度阈值 RPS（幅值） */
    uint32_t locked_thr_ms;        /* 堵转持续时间 ms */
    float    speederror_db_rps;     /* 转速误差死区 RPS：|err|<db 时漏泄回 0 */
    float    speederror_trip_rps_ms;/* 转速误差积分门限 RPS·ms */
    uint32_t comm_thr_ms;          /* 通信超时阈值 ms */
    bool enable_overcurrent;
    bool enable_overvolt;
    bool enable_undervolt;
    bool enable_stall;
    bool enable_speed_err;
    bool enable_comm;
} SafetyConfig_t;

typedef struct {
    float    vbus_over_integral_Vms;     /* 过压软积分 V·ms（Safety 独占） */
    float    vbus_under_integral_Vms;    /* 欠压积分 V·ms */
    float    speed_err_integral_rps_ms;  /* 转速误差积分 RPS·ms */
    uint32_t stall_accum_ms;
    uint32_t comm_accum_ms;
    uint32_t last_can_rx_seq;
} SafetyRuntime_t;

/*-----------------------------------------------------------
 * API
 *----------------------------------------------------------*/
/* 阈值：母线电压类用板级常量（见 Safety_Module.c 文件头的依据），
 *       电流类从 Motor_Params 推算（Current_Max×1.2 / ×1.5）。
 * 运行期只读。注意调用点必须在 Motor_Params 赋值之后。 */
void Safety_Config_Init(SafetyConfig_t *cfg);

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
