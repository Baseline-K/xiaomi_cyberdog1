/**
 * Task_Watchdog.c
 *
 * 任务看门狗实现，见 Task_Watchdog.h 的分层说明。
 *
 * 巡检流程（Twdt_Check，只在 SafetyTask 里跑）：
 *   每个监视项维护一个"喂狗计数 feeds"和"上次看到的计数 last_seen"。
 *   计数变了  → 任务在推进，清零 stall_ms。
 *   计数没变  → stall_ms += dt；累计到 timeout_ms 判定卡死。
 *
 * 判定卡死后的动作（顺序不能反）：
 *   1) 打印诊断：任务名 / 卡死时长 / 任务状态 / 栈高水位。
 *   2) MotorStateMachine_PostFault(FAULT_TASK_WATCHDOG_MASK) ← 先锁存故障位，
 *      这样并发的 PowerStage_Enable() 能被故障位闸门拦住。
 *   3) 立即硬件级关断：PowerStage_MoeOff()（无条件清 TIM1 的 MOE），
 *      一次寄存器写，不碰 SPI/RTOS。注意**不是** __HAL_TIM_MOE_DISABLE()：
 *      F1 HAL 那个宏只在 CCER 通道使能位全为 0 时才写 BDTR，PWM 运行时是空操作
 *      （详见 MotorCtrl.h 的说明）。
 *      不能只靠 PowerStage_Disable()：它第一件事是阻塞 SPI 写 DRV8323 的 COAST 寄存器，
 *      而本任务优先级最高，可能抢占正在用 SPI2 的任务（HAL 的 __HAL_LOCK 只会让那次写
 *      返回 HAL_BUSY），"关断是否真的发生"不能建立在这一步上。
 *      所以必须直接做，不能指望状态机 —— MotorStateTask 卡死时它根本不跑，
 *      PostFault 只锁存一个位，没人处理。
 *   4) PowerStage_Disable() 走标准关断路径，尽力把 DRV8323 的 COAST 位也置上。
 *   5) 恢复：非关键任务 vTaskDelete + 延时后 xTaskCreateStatic 重建；
 *            关键任务只停监视，等人工介入。
 *
 * 为什么重建必须"删除 → 等 ≥20ms → 重建"两步：
 *   vTaskDelete() 只是把任务摘出就绪/阻塞链表并把 TCB 挂到终止链表，
 *   真正从终止链表摘除是**空闲任务**做的。若立刻用同一块静态 TCB/栈重建，
 *   空闲任务随后会往新任务的链表节点里写，把新任务写坏。
 *   SafetyTask 每 3ms 让出一次 CPU，空闲任务（优先级 0）有机会运行，
 *   20ms 足够它跑好几遍。
 */
#include <stdio.h>
#include "Task_Watchdog.h"
#include "stm32f1xx_hal.h"
#include "tim.h"                  /* htim1（PowerStage_MoeOff 经 MotorCtrl.h） */
#include "MotorCtrl.h"            /* PowerStage_Disable */
#include "Safety_Module.h"        /* FAULT_TASK_WATCHDOG_MASK */
#include "motor_state_machine.h"  /* MotorStateMachine_PostFault */

/* 巡检节拍：SafetyTask 3ms 调用一次，这里按 tick 差分频到 10ms */
#define TWDT_CHECK_PERIOD_MS   10U
/* vTaskDelete 之后等空闲任务回收 TCB 的时间 */
#define TWDT_REBUILD_DELAY_MS  20U

/*-----------------------------------------------------------
 * 监视策略表（按 TWDT_Id_t 顺序，一一对应）
 *   timeout_ms : 允许"毫无推进"的最长时间，需 >= 任务正常节拍的 5~10 倍
 *   critical   : 1 = 卡死只停功率+上报+记录，不自动重建
 *   monitored  : 上电默认是否监视
 *----------------------------------------------------------*/
typedef struct {
    const char *name;
    uint32_t    timeout_ms;
    uint8_t     critical;
    uint8_t     monitored;
} Twdt_Policy_t;

static const Twdt_Policy_t s_policy[TWDT_ID_COUNT] = {
    /* BringUp    */ { "BringUp",     1000U, 0U, 1U },  /* 500ms 节拍，仅翻 LED */
    /* Diag       */ { "Diag",         100U, 0U, 1U },  /*  10ms 节拍，RTT 命令解析 */
    /* Comm       */ { "Comm",         100U, 1U, 1U },  /*  10ms 节拍，CAN 收发+上报 */
    /* MotorState */ { "MotorState",    50U, 1U, 1U },  /*  ~1ms 节拍，状态机唯一写入者 */
    /* Safety     */ { "Safety",        30U, 1U, 0U },  /* 检查者自身不监视，靠 IWDG 兜底 */
};

typedef enum {
    TWDT_ST_RUN = 0,
    TWDT_ST_WAIT_REBUILD = 1
} twdt_state_t;

typedef struct {
    volatile uint32_t feeds;        /* 任务每次循环 +1 */
    uint32_t    last_seen;          /* 上次巡检看到的 feeds */
    uint32_t    stall_ms;           /* 累计"无推进"时间 */
    uint32_t    stalls;             /* 累计判定卡死次数 */
    uint32_t    last_stall_ms;      /* 最近一次判定的卡死时长 */
    uint32_t    rebuilt;            /* 累计重建次数 */
    uint32_t    killed_tick;        /* 执行 vTaskDelete 的时刻 */
    TaskHandle_t    handle;
    TaskFunction_t  entry;
    StackType_t    *stack;
    StaticTask_t   *tcb;
    uint16_t        stack_words;
    UBaseType_t     prio;
    uint8_t         monitored;
    uint8_t         registered;
    uint8_t         state;
} Twdt_Entry_t;

static Twdt_Entry_t s_twdt[TWDT_ID_COUNT];
static uint32_t s_last_check_tick;
static uint32_t s_stall_total;
static uint32_t s_rebuild_total;

/*-----------------------------------------------------------
 * 初始化 / 登记
 *----------------------------------------------------------*/
void Twdt_Init(void)
{
    for (uint32_t i = 0U; i < (uint32_t)TWDT_ID_COUNT; i++) {
        Twdt_Entry_t *e = &s_twdt[i];

        e->feeds         = 0U;
        e->last_seen     = 0U;
        e->stall_ms      = 0U;
        e->stalls        = 0U;
        e->last_stall_ms = 0U;
        e->rebuilt       = 0U;
        e->killed_tick   = 0U;
        e->handle        = NULL;
        e->entry         = NULL;
        e->stack         = NULL;
        e->tcb           = NULL;
        e->stack_words   = 0U;
        e->prio          = 0U;
        e->monitored     = s_policy[i].monitored;
        e->registered    = 0U;
        e->state         = (uint8_t)TWDT_ST_RUN;
    }

    s_last_check_tick = xTaskGetTickCount();
    s_stall_total     = 0U;
    s_rebuild_total   = 0U;
}

void Twdt_Register(Twdt_Id_t id,
                   TaskHandle_t handle,
                   TaskFunction_t entry,
                   StackType_t *stack,
                   StaticTask_t *tcb,
                   uint16_t stack_words,
                   UBaseType_t prio)
{
    if ((uint32_t)id >= (uint32_t)TWDT_ID_COUNT) {
        return;
    }

    Twdt_Entry_t *e = &s_twdt[id];

    e->handle      = handle;
    e->entry       = entry;
    e->stack       = stack;
    e->tcb         = tcb;
    e->stack_words = stack_words;
    e->prio        = prio;
    e->last_seen   = e->feeds;
    e->stall_ms    = 0U;
    e->state       = (uint8_t)TWDT_ST_RUN;
    e->registered  = 1U;
}

void Twdt_Feed(Twdt_Id_t id)
{
    if ((uint32_t)id < (uint32_t)TWDT_ID_COUNT) {
        s_twdt[id].feeds++;      /* 32 位对齐单写，任务/ISR 皆安全，无需临界区 */
    }
}

void Twdt_SetMonitored(Twdt_Id_t id, uint8_t on)
{
    if ((uint32_t)id < (uint32_t)TWDT_ID_COUNT) {
        s_twdt[id].monitored = (on != 0U) ? 1U : 0U;
        if (on != 0U) {
            s_twdt[id].last_seen = s_twdt[id].feeds;
            s_twdt[id].stall_ms  = 0U;
        }
    }
}

uint32_t Twdt_GetStallCount(void)   { return s_stall_total; }
uint32_t Twdt_GetRebuildCount(void) { return s_rebuild_total; }

uint32_t Twdt_GetStatus(Twdt_Status_t *out, uint32_t max_out)
{
    uint32_t n = 0U;

    if (out == NULL) {
        return 0U;
    }

    for (uint32_t i = 0U; (i < (uint32_t)TWDT_ID_COUNT) && (n < max_out); i++) {
        const Twdt_Entry_t *e = &s_twdt[i];

        if (e->registered == 0U) {
            continue;
        }

        out[n].name          = s_policy[i].name;
        out[n].feeds         = e->feeds;
        out[n].stalls        = e->stalls;
        out[n].last_stall_ms = e->last_stall_ms;
        out[n].rebuilt       = e->rebuilt;
        out[n].monitored     = e->monitored;
        out[n].critical      = s_policy[i].critical;
        n++;
    }

    return n;
}

/*-----------------------------------------------------------
 * 卡死诊断打印
 *----------------------------------------------------------*/
static void twdt_report_stall(Twdt_Id_t id, const Twdt_Entry_t *e)
{
    printf("\r\n[TWDT] STALL task=%s stalled=%lums feeds=%lu stall_#%lu\r\n",
           s_policy[id].name,
           (unsigned long)e->stall_ms,
           (unsigned long)e->feeds,
           (unsigned long)(e->stalls + 1U));

    if (e->handle != NULL) {
        /* state: 0=eRunning 1=eReady 2=eBlocked 3=eSuspended 4=eDeleted
         * 卡死通常是 eReady（被更高优先级任务饿死）或 eBlocked（等一个永不到来的事件）。 */
        printf("[TWDT]   state=%d stack_hw=%lu/%u words\r\n",
               (int)eTaskGetState(e->handle),
               (unsigned long)uxTaskGetStackHighWaterMark(e->handle),
               (unsigned)e->stack_words);
    }

    if (s_policy[id].critical != 0U) {
        printf("[TWDT]   CRITICAL: power stage OFF + fault posted; no auto rebuild\r\n");
    } else {
        printf("[TWDT]   non-critical: rebuild in %ums\r\n", (unsigned)TWDT_REBUILD_DELAY_MS);
    }
}

/*-----------------------------------------------------------
 * 巡检（SafetyTask 上下文，10ms 分频）
 *----------------------------------------------------------*/
void Twdt_Check(void)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t dt  = (uint32_t)(now - s_last_check_tick);

    if (dt < (uint32_t)TWDT_CHECK_PERIOD_MS) {
        return;
    }
    s_last_check_tick = now;

    for (uint32_t i = 0U; i < (uint32_t)TWDT_ID_COUNT; i++) {
        Twdt_Entry_t *e = &s_twdt[i];

        if (e->registered == 0U) {
            continue;
        }

        /* ---- 阶段 2：已删除，等空闲任务把 TCB 从终止链表摘掉后再重建 ---- */
        if (e->state == (uint8_t)TWDT_ST_WAIT_REBUILD) {
            if ((uint32_t)(now - e->killed_tick) < (uint32_t)TWDT_REBUILD_DELAY_MS) {
                continue;
            }

            e->handle = xTaskCreateStatic(e->entry,
                                          s_policy[i].name,
                                          e->stack_words,
                                          (void *)NULL,
                                          e->prio,
                                          e->stack,
                                          e->tcb);
            if (e->handle != NULL) {
                e->feeds     = 0U;
                e->last_seen = 0U;
                e->stall_ms  = 0U;
                e->state     = (uint8_t)TWDT_ST_RUN;
                e->rebuilt++;
                s_rebuild_total++;
                printf("[TWDT] rebuilt task=%s\r\n", s_policy[i].name);
            } else {
                printf("[TWDT] REBUILD FAILED task=%s\r\n", s_policy[i].name);
            }
            continue;
        }

        /* ---- 未监视：只同步计数，不判卡死 ---- */
        if (e->monitored == 0U) {
            e->last_seen = e->feeds;
            e->stall_ms  = 0U;
            continue;
        }

        /* ---- 有推进 → 健康 ---- */
        if (e->feeds != e->last_seen) {
            e->last_seen = e->feeds;
            e->stall_ms  = 0U;
            continue;
        }

        /* ---- 无推进 → 累计，未到门限就返回 ---- */
        e->stall_ms += dt;
        if (e->stall_ms < s_policy[i].timeout_ms) {
            continue;
        }

        /* ================= 判定卡死 ================= */
        e->stalls++;
        e->last_stall_ms = e->stall_ms;
        s_stall_total++;
        twdt_report_stall((Twdt_Id_t)i, e);

        /* 1) 先锁存故障位，再断功率。顺序不能反：
         *    这样并发的 PowerStage_Enable()（如 state_RUN_entry 执行到一半被本任务抢占）
         *    能看到故障位而被拦住，否则会出现"看门狗刚关断、被抢占者又把功率打开"。
         *    （PowerStage_Enable() 里的故障位闸门是 Batch 3 项，届时这里就闭合了。） */
        MotorStateMachine_PostFault(FAULT_TASK_WATCHDOG_MASK);

        /* 2) 立即硬件级关断：只清 TIM1 的 MOE，一次寄存器写，不碰 SPI、不碰 RTOS，
         *    是这里最确定、最快的动作。不能只依赖 PowerStage_Disable()：
         *    它第一件事是阻塞 SPI 写 DRV8323 的 COAST 寄存器，而本任务优先级最高，
         *    可能抢占正在使用 SPI2 的 MotorStateTask/CommTask —— 虽然 HAL 的
         *    __HAL_LOCK 保证不会损坏总线（只会返回 HAL_BUSY 丢掉那次写），
         *    但"关断是否真的发生"不能建立在一次可能失败的 SPI 上。
         *    注意用 PowerStage_MoeOff() 而不是 __HAL_TIM_MOE_DISABLE()（后者是空操作，
         *    原因见 MotorCtrl.h 的说明）。 */
        PowerStage_MoeOff();

        /* 3) 再走标准关断路径，尽力把 DRV8323 的 COAST 位也置上 */
        PowerStage_Disable();

        /* 4) 恢复动作 */
        if ((s_policy[i].critical == 0U) && (e->handle != NULL)) {
            /* SafetyTask 优先级最高，此刻目标任务必然不在运行，删除安全 */
            vTaskDelete(e->handle);
            e->handle      = NULL;
            e->killed_tick = now;
            e->state       = (uint8_t)TWDT_ST_WAIT_REBUILD;
        } else {
            /* 关键任务：不重建 → 停监视，避免每 10ms 刷屏 */
            e->monitored = 0U;
            printf("[TWDT] %s monitoring disabled, host intervention required\r\n",
                   s_policy[i].name);
        }

        e->stall_ms = 0U;
    }
}

/*-----------------------------------------------------------
 * 复位原因诊断
 *----------------------------------------------------------*/
static Twdt_ResetCause_t s_reset_cause = TWDT_RST_UNKNOWN;

Twdt_ResetCause_t Twdt_ResetCauseInit(void)
{
    Twdt_ResetCause_t cause = TWDT_RST_OTHER;

    /* 顺序有讲究：上电瞬间 PORRST 和 PINRST 会同时置位，所以先判"更有信息量"的。
     * STM32F1 的 HAL 里独立看门狗标志名是 RCC_FLAG_IWDGRST（不是 IWDG1RST）。 */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
        cause = TWDT_RST_IWDG;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) {
        cause = TWDT_RST_WWDG;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
        cause = TWDT_RST_SOFTWARE;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) {
        cause = TWDT_RST_LOW_POWER;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) {
        cause = TWDT_RST_POWER_ON;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
        cause = TWDT_RST_PIN;
    }

    __HAL_RCC_CLEAR_RESET_FLAGS();   /* 清掉，让"下一次"复位能被正确识别 */
    s_reset_cause = cause;

    /* 注意：这里**不**上报故障位，只打印 + 记录。
     * 上电时状态机还在 S_INIT，而 eventMap 里没有 {EVENT_toFAULT, S_INIT}
     * （之前日志里 "illegal event [toFAULT] in S_INIT" 就是这个洞）。
     * 现在 motor_state_machine.c 的 state_IDLE_do 会兜底检查 g_fault_bits，
     * 所以只要在这里加一行 MotorStateMachine_PostFault()，故障就能一路
     * 走到关功率 + EMCY 上报；不加则仅作诊断记录（当前选择）。 */
    printf("[WDG] RCC reset cause = %s\r\n", Twdt_ResetCauseName(cause));
    if ((cause == TWDT_RST_IWDG) || (cause == TWDT_RST_WWDG)) {
        printf("[WDG]   !! previous run was killed by the watchdog (feed point missed) !!\r\n");
    }

    return cause;
}

Twdt_ResetCause_t Twdt_GetResetCause(void)
{
    return s_reset_cause;
}

const char *Twdt_ResetCauseName(Twdt_ResetCause_t c)
{
    switch (c) {
    case TWDT_RST_POWER_ON:  return "POWER_ON";
    case TWDT_RST_PIN:       return "NRST_PIN";
    case TWDT_RST_SOFTWARE:  return "SOFTWARE";
    case TWDT_RST_IWDG:      return "IWDG_TIMEOUT";
    case TWDT_RST_WWDG:      return "WWDG_TIMEOUT";
    case TWDT_RST_LOW_POWER: return "LOW_POWER";
    case TWDT_RST_OTHER:     return "OTHER";
    case TWDT_RST_UNKNOWN:
    default:                 return "UNKNOWN";
    }
}

uint8_t Twdt_IsWatchdogReset(void)
{
    return ((s_reset_cause == TWDT_RST_IWDG) || (s_reset_cause == TWDT_RST_WWDG)) ? 1U : 0U;
}
