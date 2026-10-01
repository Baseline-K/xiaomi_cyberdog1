/**
 * Task_Watchdog.h
 *
 * 任务看门狗（TWDT）+ 系统看门狗（IWDG）复位原因诊断。
 *
 * 分层（三层，各管各的失效模式，缺一层就有盲区）：
 *   1. TWDT  ：某个任务卡死     → SafetyTask 巡检发现 → 记录诊断 + 关功率 + 上报；
 *              非关键任务自动重建，关键任务交人工处理（见 Task_Watchdog.c 策略表）。
 *   2. IWDG  ：SafetyTask 自己卡死 / 全程关中断 → 硬件复位（40ms）。
 *   3. 复位原因：下一次启动时读 RCC 复位标志，打印 + 记录，供 RTT `get` 查询。
 *
 * 与 IWDG 的分工一定要分清：
 *   - IWDG 的喂狗点**只有** SafetyTask 一处（运行期），初始化期间只有 TIM4 中断一处。
 *     任何第三处喂狗都会让这套机制失效。
 *   - SafetyTask 不能监视自己，那是 IWDG 的职责，所以 Safety 在策略表里 monitored=0。
 */
#ifndef TASK_WATCHDOG_H
#define TASK_WATCHDOG_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/*-----------------------------------------------------------
 * 被监视的任务
 * 新增任务时：枚举末尾追加一项 → 同步 Task_Watchdog.c 的 s_policy 表 → 在任务
 * 循环里加一行 Twdt_Feed() → 在 AppTasks_Init() 里 Twdt_Register()。
 *----------------------------------------------------------*/
typedef enum {
    TWDT_ID_BRINGUP = 0,
    TWDT_ID_DIAG,
    TWDT_ID_COMM,
    TWDT_ID_MOTORSTATE,
    TWDT_ID_SAFETY,
    TWDT_ID_COUNT
} Twdt_Id_t;

/* 快照结构（供 RTT `get` / `diag` 打印） */
typedef struct {
    const char *name;
    uint32_t    feeds;          /* 累计喂狗次数（= 任务推进了多少圈） */
    uint32_t    stalls;         /* 累计判定卡死次数 */
    uint32_t    last_stall_ms;  /* 最近一次判定的卡死时长 */
    uint32_t    rebuilt;        /* 累计重建次数 */
    uint8_t     monitored;      /* 1 = 当前正在监视 */
    uint8_t     critical;       /* 1 = 关键任务（卡死不自动重建） */
} Twdt_Status_t;

/*-----------------------------------------------------------
 * 复位原因（启动早期调用一次）
 *----------------------------------------------------------*/
typedef enum {
    TWDT_RST_UNKNOWN = 0,
    TWDT_RST_POWER_ON,      /* 上电复位 */
    TWDT_RST_PIN,           /* NRST 引脚复位 */
    TWDT_RST_SOFTWARE,      /* 软件复位 */
    TWDT_RST_IWDG,          /* 独立看门狗超时 ← 我们要抓的就是这个 */
    TWDT_RST_WWDG,          /* 窗口看门狗超时 */
    TWDT_RST_LOW_POWER,     /* 低功耗复位 */
    TWDT_RST_OTHER
} Twdt_ResetCause_t;

/*-----------------------------------------------------------
 * API
 *----------------------------------------------------------*/

/* 清空监视表并记录启动时刻（在创建任务之前调用，AppTasks_Init() 内部会调） */
void Twdt_Init(void);

/* 任务创建后登记：重建时需要的入口/静态栈/静态 TCB/优先级全部存下来 */
void Twdt_Register(Twdt_Id_t id,
                   TaskHandle_t handle,
                   TaskFunction_t entry,
                   StackType_t *stack,
                   StaticTask_t *tcb,
                   uint16_t stack_words,
                   UBaseType_t prio);

/* 任务循环里调用；纯自增，任务上下文（也可在 ISR 里调，32 位对齐单写） */
void Twdt_Feed(Twdt_Id_t id);

/* 巡检：只允许在 SafetyTask 内调用（内部按 tick 差分频到 10ms 一次） */
void Twdt_Check(void);

/* 运行期开关某个任务的监视（例如一次性任务跑完就停掉） */
void Twdt_SetMonitored(Twdt_Id_t id, uint8_t on);

/* 汇总计数（诊断用） */
uint32_t Twdt_GetStallCount(void);
uint32_t Twdt_GetRebuildCount(void);

/* 导出快照，返回实际写入条数 */
uint32_t Twdt_GetStatus(Twdt_Status_t *out, uint32_t max_out);

/*-----------------------------------------------------------
 * 复位原因
 *----------------------------------------------------------*/
/* 读 RCC 复位标志 → 记录 → 清标志。返回"上一次是什么原因复位的"。
 * 必须在 RTT 初始化之后调用（会打印）。 */
Twdt_ResetCause_t Twdt_ResetCauseInit(void);
Twdt_ResetCause_t Twdt_GetResetCause(void);
const char *Twdt_ResetCauseName(Twdt_ResetCause_t c);
/* 1 = 上次复位是看门狗引起的（IWDG 或 WWDG） */
uint8_t Twdt_IsWatchdogReset(void);

#endif /* TASK_WATCHDOG_H */
