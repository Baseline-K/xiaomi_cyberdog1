/**
 * app_tasks.c
 *
 * FreeRTOS 应用任务与钩子实现（Phase 1：内核接入阶段）。
 *
 * Phase 1 只接入内核，不改 FOC/CAN/状态机路径：
 * - 提供内核必需的静态内存回调与钩子。
 * - BringUpTask / LegacyMainTask 在调度器启动后运行。
 *
 * 对应《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§3.2 / §14 首个工作包。
 */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "app_tasks.h"
#include "SEGGER_RTT.h"
#include "CANopen_OD.h"
#include "CAN_bsp.h"
#include "RTT_Cmd.h"
#include "motor_state_machine.h"
#include "motor_command_snapshot.h"
#include "Safety_Module.h"
#include "FOC_generated.h"
#include "foc.h"   /* foc_abc_current_i */
#include "iwdg.h"            /* hiwdg：运行期唯一的 IWDG 喂养点在 SafetyTask */
#include "Task_Watchdog.h"   /* Twdt_*：任务看门狗 + 复位原因诊断 */
#include "FOC_run.h"         /* V_SCALE */
#include "main_user.h"       /* Vbus_ADC_DMA：慢安全专用的母线电压 DMA 缓冲 */
#include "MotorCtrl.h"       /* g_nfault_*：栅极驱动 nFAULT 诊断计数 */

extern float Vbus;                 /* FOC_run.c */
extern volatile uint32_t g_foc_wcet_us;   /* FOC_run.c */
extern volatile uint32_t g_foc_isr_us;    /* FOC_run.c：FOC ISR 最近一次耗时 */
extern volatile uint32_t g_foc_isr_cnt;   /* FOC_run.c：FOC ISR 进入次数（判采样链路是否活着） */
extern volatile uint32_t g_enc_read_us;      /* FOC_run.c：编码器服务层一拍耗时（µs） */
extern volatile uint32_t g_enc_read_wcet_us; /* FOC_run.c：上电以来最大编码器一拍耗时（µs） */

/* IWDG 因"FOC 中断未推进"而跳过的喂狗次数（>0 说明采样链路曾经停过）。
 * `diag` 可读；一旦非 0，IWDG 会在 40.8ms 内复位，下次上电看复位原因即可确认。 */
volatile uint32_t g_wdg_skip = 0U;

/*-----------------------------------------------------------
 * 任务栈/TCB 静态内存与句柄
 *----------------------------------------------------------*/
static StackType_t   BringUpTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t  BringUpTaskTCB;
static TaskHandle_t  BringUpTaskHandle;

static StackType_t   DiagTaskStack[configMINIMAL_STACK_SIZE * 4];   /* 2 KB，RTT 命令+printf 调用链 */
static StaticTask_t  DiagTaskTCB;
static TaskHandle_t  DiagTaskHandle;

static StackType_t   CommTaskStack[configMINIMAL_STACK_SIZE * 3];   /* 1.5 KB，CAN 解析+应答调用链 */
static StaticTask_t  CommTaskTCB;
TaskHandle_t         xCommTaskHandle;   /* 非 static：供 CAN RX ISR 通知 */

static StackType_t   MotorStateTaskStack[configMINIMAL_STACK_SIZE * 2];  /* 1 KB */
static StaticTask_t  MotorStateTaskTCB;
static TaskHandle_t  MotorStateTaskHandle;

/*-----------------------------------------------------------
 * MotorStateTask：状态机唯一写入者（计划 §3.2 优先级 4）
 * 队列 + 1ms 巡检 fault_pending；事件驱动 + 定期 Do。
 *----------------------------------------------------------*/
static void MotorStateTask_Entry(void *pvParam)
{
    (void) pvParam;

    for (;;)
    {
        MotorStateMachine_Step();
        Twdt_Feed(TWDT_ID_MOTORSTATE);   /* 每圈约 1ms（内部 xQueueReceive 超时 1ms） */
    }
}

/*-----------------------------------------------------------
 * SafetyTask：慢安全检测（计划 §3.2 优先级 5，3ms 绝对周期）
 * 组装一致快照 → Safety_SlowStep → 新故障位 → PostFault。
 *----------------------------------------------------------*/
static StackType_t  SafetyTaskStack[configMINIMAL_STACK_SIZE * 3];  /* 1.5 KB */
static StaticTask_t SafetyTaskTCB;
static TaskHandle_t SafetyTaskHandle;

static void SafetyTask_Entry(void *pvParam)
{
    SafetyRuntime_t rt = { 0 };
    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t uPrevTick = xLastWake;
    uint32_t uPrevFocCnt = g_foc_isr_cnt;   /* 上次巡检看到的 FOC ISR 计数（喂狗条件） */

    (void) pvParam;

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(3));

        Twdt_Feed(TWDT_ID_SAFETY);   /* 自身记账；策略表里 Safety 不监视自己，靠 IWDG 兜底 */

        uint32_t uNow = xTaskGetTickCount();

        SafetySlowInput_t in = { 0 };
        in.elapsed_ms = (uint32_t)(uNow - uPrevTick);   /* 实际经过时间（计划 §6.2） */
        uPrevTick = uNow;

        in.is_run = (MotorState_GetCurrent() == S_RUN);
        /* 母线电压：直接用 FOC ISR 发布的值（ADC3 注入 rank2，10kHz、PWM 同步）。
         * 不再单独开一条 DMA 通道：ADC3 若同时跑规则组，那 16.8us（239.5 周期采样）
         * 的单次转换会把注入序列（电流采样）推迟最多 16.8us，得不偿失。
         * 而且经过 2026-09-27 的结构性修复（采样触发改 TRGO/OC4REF、
         * PowerStage_Disable 保住 CEN、ADC 三路彻底分开）之后，
         * "FOC ISR 被意外掐断"这个风险已经从机理上消除了。 */
        in.measurement.vbus_V            = Vbus;
        in.measurement.phase_current_A[0] = foc_abc_current_i.ia;
        in.measurement.phase_current_A[1] = foc_abc_current_i.ib;
        in.measurement.phase_current_A[2] = foc_abc_current_i.ic;
        in.measurement.motor_speed_rps   = FOC_Generated_GetSpeedRps();
        {
            const MotorCommand_t *cmd = MotorCommand_Get();
            in.measurement.target_speed_rps = cmd->speed_rps;
            in.measurement.target_iq_A      = cmd->iq_ref_A;
        }
        in.measurement.can_rx_valid_seq   = CANopen_OD_GetValidSeq();

        fault_mask_t new_faults = 0;
        Safety_SlowStep(&in, &Safety_Config, &rt, &new_faults);
        if (new_faults != 0U) {
            MotorStateMachine_PostFault(new_faults);
        }

        /* ===== 任务看门狗巡检（内部按 tick 差分频到 10ms）=====
         * 判某任务卡死 → 记录诊断 + 直接关功率 + PostFault(FAULT_TASK_WATCHDOG_MASK)
         *                + 非关键任务延时重建；关键任务只停功率等人工。 */
        Twdt_Check();

        /* ===== IWDG 喂狗：运行期全系统唯一一处，且**必须证明采样链路还活着** =====
         * 放在 SafetyTask 末尾，含义是"慢安全检测完整跑完一轮才算活着"。
         * 初始化期间由 TIM4 中断喂（main_user.c 里已在启动调度器前关掉），
         * 任何第三处喂狗都会让 IWDG 失去意义。
         *
         * 为什么还要加"FOC 中断必须推进"这一条：
         *   只看任务健康是不够的 —— FOC 中断一旦停掉（CEN 丢了 / JEOCIE 被 HAL 关掉 /
         *   触发源接错），任务照跑，但 Vbus、三相电流、转速会**全部冻结**，
         *   而本任务却还在盲目喂狗：保护逻辑看着在工作，实际已经在拿陈旧数据做判断。
         *   所以要求 g_foc_isr_cnt 在本拍（3ms，正常应推进约 30 次）内有增长；
         *   没增长就不喂 → 40.8ms 后 IWDG 复位，把"采样链路死了"变成可见的复位原因
         *   （下次上电 `[WDG] RCC reset cause = IWDG_TIMEOUT`）。
         *   FOC_Interrupt_Stop() 全工程从未被调用，所以不存在"合法停中断"被误判的情况。 */
        {
            uint32_t foc_now = g_foc_isr_cnt;

            if (foc_now != uPrevFocCnt) {
                uPrevFocCnt = foc_now;
                HAL_IWDG_Refresh(&hiwdg);
            } else {
                g_wdg_skip++;
                if (g_wdg_skip == 1U) {   /* 只报第一次，避免 3ms 刷屏 */
                    printf("[WDG] FOC ISR stalled (cnt=%lu) -> withholding IWDG feed\r\n",
                           (unsigned long) foc_now);
                }
            }
        }
    }
}

/*-----------------------------------------------------------
 * DiagTask：RTT 命令解释（计划 §3.2 优先级 1）。
 * RTT_Cmd_Process 从 LegacyMainTask 移入；诊断输出改为交互 `diag` 命令触发，
 * 不再周期刷屏（系统存活由 BringUpTask 心跳确认）。
 *----------------------------------------------------------*/
static void DiagTask_Entry(void *pvParam)
{
    TickType_t xLastWake = xTaskGetTickCount();

    (void) pvParam;

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));

        Twdt_Feed(TWDT_ID_DIAG);
        RTT_Cmd_Process();   /* 非阻塞逐字符解析 RTT 下行命令 */
    }
}

/* 诊断输出：各任务 CPU%/栈高水位 / FOC WCET / CAN 统计 / 状态机统计（供 RTT `diag` 命令调用） */
void AppTasks_Diag(void)
{
    uint32_t rx_total, rx_overrun, rx_drop;
    uint32_t posted, dropped, illegal, fault_forced;

    CAN_bsp_GetStats(&rx_total, &rx_overrun, &rx_drop);
    MotorState_GetStats(&posted, &dropped, &illegal, &fault_forced);

    printf("\r\n==== Diag ====\r\n");
    printf("  FOC ISR WCET: %lu us (预算100us)\r\n", (unsigned long) g_foc_wcet_us);
    /* FOC ISR 进入次数：连着两次 `diag` 对比，涨 = 采样链路活着；
     * 不涨 = TIM1_TRGO(OC4REF) → ADC 注入 → JEOC 这条链断了（查 TIM1->CR1.CEN / CCER /
     * ADC1->CR2.JEXTTRIG / CRT1.JEOCIE）。 */
    printf("  FOC ISR count: %lu (last %lu us)\r\n",
           (unsigned long) g_foc_isr_cnt, (unsigned long) g_foc_isr_us);
    /* 编码器一拍在 FOC 中断里的实测耗时。AS5047 阻塞读（2 帧 @7.5MHz 共 4.27us
     * + 4 次 t_CSn/t_L 间隔）预期约 6us；DMA 流水线模式预期 <1us（只取缓冲+发下一帧）。
     * 若 >>6us：先查 as5047_cs_gap 与 SPI 分频，不要先去改协议。 */
    printf("  ENC read: %lu us (max %lu us)\r\n",
           (unsigned long) g_enc_read_us, (unsigned long) g_enc_read_wcet_us);
    /* IWDG 跳过喂狗次数：非 0 = 曾经出现"FOC 中断没推进"，40.8ms 内会复位 */
    printf("  WDG skip (FOC stalled): %lu\r\n", (unsigned long) g_wdg_skip);
    {
        /* 各任务 CPU%（uxTaskGetSystemState，纯静态可用） */
        TaskStatus_t tstat[10];
        configRUN_TIME_COUNTER_TYPE ulTotalRun = 0;
        UBaseType_t n = uxTaskGetSystemState(tstat, 10U, &ulTotalRun);
        printf("  Task CPU%% (run=%lu):\r\n", (unsigned long) ulTotalRun);
        for (UBaseType_t i = 0; i < n; i++) {
            unsigned long pct = (ulTotalRun > 0U) ?
                (unsigned long)((tstat[i].ulRunTimeCounter * 100UL) / (unsigned long)ulTotalRun) : 0UL;
            printf("    %-10s %lu%%\r\n", tstat[i].pcTaskName, pct);
        }
    }
    printf("  nFAULT: real=%lu spurious=%lu last_tick=%lu\r\n",
           (unsigned long) g_nfault_count,
           (unsigned long) g_nfault_spurious,
           (unsigned long) g_nfault_tick);
    printf("  CAN: rx=%lu overrun=%lu drop=%lu\r\n",
           (unsigned long) rx_total, (unsigned long) rx_overrun, (unsigned long) rx_drop);
    printf("  FSM: posted=%lu dropped=%lu illegal=%lu fault=%lu\r\n",
           (unsigned long) posted, (unsigned long) dropped,
           (unsigned long) illegal, (unsigned long) fault_forced);

    /* 看门狗：上次复位原因 + 任务看门狗各项计数 */
    {
        Twdt_Status_t ts[TWDT_ID_COUNT];
        uint32_t n = Twdt_GetStatus(ts, (uint32_t)TWDT_ID_COUNT);

        printf("  WDG: reset=%s was_iwdg=%u stall_total=%lu rebuild_total=%lu\r\n",
               Twdt_ResetCauseName(Twdt_GetResetCause()),
               (unsigned) Twdt_IsWatchdogReset(),
               (unsigned long) Twdt_GetStallCount(),
               (unsigned long) Twdt_GetRebuildCount());
        for (uint32_t i = 0U; i < n; i++) {
            printf("    %-10s feeds=%-8lu stalls=%lu last=%lums rebuilt=%lu mon=%u%s\r\n",
                   ts[i].name,
                   (unsigned long) ts[i].feeds,
                   (unsigned long) ts[i].stalls,
                   (unsigned long) ts[i].last_stall_ms,
                   (unsigned long) ts[i].rebuilt,
                   (unsigned) ts[i].monitored,
                   (ts[i].critical != 0U) ? " CRIT" : "");
        }
    }
    printf("  Stacks HW/alloc:\r\n");
    printf("    BringUp %lu/%lu\r\n",
           (unsigned long) uxTaskGetStackHighWaterMark(BringUpTaskHandle),
           (unsigned long) (sizeof(BringUpTaskStack) / sizeof(StackType_t)));
    printf("    Comm    %lu/%lu\r\n",
           (unsigned long) uxTaskGetStackHighWaterMark(xCommTaskHandle),
           (unsigned long) (sizeof(CommTaskStack) / sizeof(StackType_t)));
    printf("    MotorSt %lu/%lu\r\n",
           (unsigned long) uxTaskGetStackHighWaterMark(MotorStateTaskHandle),
           (unsigned long) (sizeof(MotorStateTaskStack) / sizeof(StackType_t)));
    printf("    Safety  %lu/%lu\r\n",
           (unsigned long) uxTaskGetStackHighWaterMark(SafetyTaskHandle),
           (unsigned long) (sizeof(SafetyTaskStack) / sizeof(StackType_t)));
    printf("    Diag    %lu/%lu\r\n",
           (unsigned long) uxTaskGetStackHighWaterMark(DiagTaskHandle),
           (unsigned long) (sizeof(DiagTaskStack) / sizeof(StackType_t)));
}

/*-----------------------------------------------------------
 * CommTask：CAN 协议解析任务（计划 §3.2 优先级 3）
 * CAN RX ISR 收帧入环形缓冲 → vTaskNotifyGiveFromISR 唤醒本任务 → 批量弹帧处理。
 * 同时周期性发送心跳测试帧，便于用 CAN 分析仪验证 TX。
 *----------------------------------------------------------*/
#define CAN_COMM_PERIOD_MS  10U    /* CommTask 节拍：故障上报重发周期 */
#define CAN_HB_EVERY_TICKS  10U    /* 每 10 拍(10ms)=100ms 发一次心跳 */
#define CAN_HB_MARKER       0xFEU  /* 心跳标记 data[0]，与分析仪区分 */
#define CAN_EMCY_CLASS      0x01U  /* 故障帧 data[0]：设备故障类 */

/* 心跳帧：ID=0x581，data[0]=0xFE 标记，data[1]=节点号，data[2..4]=时间戳，data[6..7]=递增计数 */
static void CAN_SendHeartbeat(void)
{
    uint8_t d[8] = {0};
    static uint16_t seq = 0U;
    uint32_t tick = HAL_GetTick();

    d[0] = CAN_HB_MARKER;
    d[1] = (uint8_t)CAN_NODE_ID;
    d[2] = (uint8_t)(tick & 0xFFU);
    d[3] = (uint8_t)((tick >> 8) & 0xFFU);
    d[4] = (uint8_t)((tick >> 16) & 0xFFU);
    d[6] = (uint8_t)(seq & 0xFFU);
    d[7] = (uint8_t)(seq >> 8);
    seq++;
    CAN_bsp_Send(CAN_ID_SDO_TX, d, 8);
}

/* 故障上报帧：ID=EMCY 0x081
 * data[0]=故障类(0x01 设备) data[1..4]=32 位故障掩码 LE data[5]=FSM 状态 data[7]=递增计数 */
static void CAN_SendFaultReport(void)
{
    uint8_t d[8] = {0};
    static uint8_t seq = 0U;
    uint32_t w = g_fault_report_word;

    d[0] = CAN_EMCY_CLASS;
    d[1] = (uint8_t)(w & 0xFFU);
    d[2] = (uint8_t)((w >> 8) & 0xFFU);
    d[3] = (uint8_t)((w >> 16) & 0xFFU);
    d[4] = (uint8_t)((w >> 24) & 0xFFU);
    d[5] = (uint8_t)MotorState_GetCurrent();
    d[7] = seq++;
    if (CAN_bsp_Send(CAN_ID_EMCY, d, 8U)) {
        g_fault_report_dirty = 0U;   /* 至少发出一次后清 dirty */
    }
}

static void CommTask_Entry(void *pvParam)
{
    (void) pvParam;
    const TickType_t xPeriod = pdMS_TO_TICKS(CAN_COMM_PERIOD_MS);
    uint8_t hb_div = 0U;

    for (;;)
    {
        /* 等待 CAN 帧通知；带 10ms 超时以便周期处理心跳/故障上报 */
        xTaskNotifyWait(0UL, 0xFFFFFFFFUL, NULL, xPeriod);

        Twdt_Feed(TWDT_ID_COMM);

        /* 一次性批处理缓冲内全部帧（内部 while PopFrame） */
        CANopen_OD_Process();

        /* 故障态：每 10ms 重发 EMCY（新故障立即发；发送成功清 dirty） */
        if (g_fault_report_active != 0U) {
            CAN_SendFaultReport();
        }

        /* 心跳 100ms（10×10ms）；故障帧优先占用邮箱，心跳可容忍偶发丢帧 */
        if (++hb_div >= CAN_HB_EVERY_TICKS) {
            hb_div = 0U;
            CAN_SendHeartbeat();
        }
    }
}

/*-----------------------------------------------------------
 * BringUpTask：低优先级验证任务，翻转板载 LED(PC13) 并计数
 *----------------------------------------------------------*/
static void BringUpTask_Entry(void *pvParam)
{
    TickType_t xLastWake = xTaskGetTickCount();

    (void) pvParam;

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(500));
        Twdt_Feed(TWDT_ID_BRINGUP);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   /* 板载 LED 500ms 翻转，验证调度器存活 */
        /* RTT 心跳已关闭（保留 CAN 心跳，见 CommTask_Entry CAN_SendHeartbeat） */
    }
}

/*-----------------------------------------------------------
 * 创建所有 Phase 1 静态任务
 *----------------------------------------------------------*/
void AppTasks_Init(void)
{
    /* 任务看门狗：清空监视表并记录启动时刻（策略表见 Task_Watchdog.c） */
    Twdt_Init();

    /* 状态机初始化：创建静态事件队列 + 投递 EVENT_INIT */
    MotorStateMachine_Init();

    /* BringUpTask：优先级 1，验证内核运行 */
    BringUpTaskHandle = xTaskCreateStatic(BringUpTask_Entry,
                                          "BringUp",
                                          sizeof(BringUpTaskStack) / sizeof(StackType_t),
                                          (void *) NULL,
                                          1,
                                          BringUpTaskStack,
                                          &BringUpTaskTCB);
    configASSERT(BringUpTaskHandle != NULL);

    /* DiagTask：优先级 1，RTT 命令 + 周期诊断 */
    DiagTaskHandle = xTaskCreateStatic(DiagTask_Entry,
                                       "Diag",
                                       sizeof(DiagTaskStack) / sizeof(StackType_t),
                                       (void *) NULL,
                                       1,
                                       DiagTaskStack,
                                       &DiagTaskTCB);
    configASSERT(DiagTaskHandle != NULL);

    /* CommTask：优先级 3，CAN 协议解析（任务通知驱动） */
    xCommTaskHandle = xTaskCreateStatic(CommTask_Entry,
                                        "Comm",
                                        sizeof(CommTaskStack) / sizeof(StackType_t),
                                        (void *) NULL,
                                        3,
                                        CommTaskStack,
                                        &CommTaskTCB);
    configASSERT(xCommTaskHandle != NULL);

    /* MotorStateTask：优先级 4，状态机唯一写入者 */
    MotorStateTaskHandle = xTaskCreateStatic(MotorStateTask_Entry,
                                             "MotorState",
                                             sizeof(MotorStateTaskStack) / sizeof(StackType_t),
                                             (void *) NULL,
                                             4,
                                             MotorStateTaskStack,
                                             &MotorStateTaskTCB);
    configASSERT(MotorStateTaskHandle != NULL);

    /* SafetyTask：优先级 5，慢安全检测（3ms 绝对周期） */
    SafetyTaskHandle = xTaskCreateStatic(SafetyTask_Entry,
                                         "Safety",
                                         sizeof(SafetyTaskStack) / sizeof(StackType_t),
                                         (void *) NULL,
                                         5,
                                         SafetyTaskStack,
                                         &SafetyTaskTCB);
    configASSERT(SafetyTaskHandle != NULL);

    /* ===== 任务看门狗登记 =====
     * 把"重建一个任务"所需的东西全部交给 TWDT：入口函数、静态栈、静态 TCB、优先级。
     * 静态分配下 vTaskDelete 不释放内存，重建就是复用同一块内存，所以必须存下这些指针。
     * 监视策略（超时时间 / 是否关键任务 / 是否默认监视）在 Task_Watchdog.c 的 s_policy 表。 */
    Twdt_Register(TWDT_ID_BRINGUP, BringUpTaskHandle, BringUpTask_Entry,
                  BringUpTaskStack, &BringUpTaskTCB,
                  (uint16_t)(sizeof(BringUpTaskStack) / sizeof(StackType_t)), 1U);
    Twdt_Register(TWDT_ID_DIAG, DiagTaskHandle, DiagTask_Entry,
                  DiagTaskStack, &DiagTaskTCB,
                  (uint16_t)(sizeof(DiagTaskStack) / sizeof(StackType_t)), 1U);
    Twdt_Register(TWDT_ID_COMM, xCommTaskHandle, CommTask_Entry,
                  CommTaskStack, &CommTaskTCB,
                  (uint16_t)(sizeof(CommTaskStack) / sizeof(StackType_t)), 3U);
    Twdt_Register(TWDT_ID_MOTORSTATE, MotorStateTaskHandle, MotorStateTask_Entry,
                  MotorStateTaskStack, &MotorStateTaskTCB,
                  (uint16_t)(sizeof(MotorStateTaskStack) / sizeof(StackType_t)), 4U);
    Twdt_Register(TWDT_ID_SAFETY, SafetyTaskHandle, SafetyTask_Entry,
                  SafetyTaskStack, &SafetyTaskTCB,
                  (uint16_t)(sizeof(SafetyTaskStack) / sizeof(StackType_t)), 5U);
}

/*-----------------------------------------------------------
 * configSUPPORT_STATIC_ALLOCATION 必需回调
 *----------------------------------------------------------*/
static StackType_t   IdleTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t  IdleTaskTCB;

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &IdleTaskTCB;
    *ppxIdleTaskStackBuffer = IdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

/*-----------------------------------------------------------
 * configCHECK_FOR_STACK_OVERFLOW=2 栈溢出钩子
 *----------------------------------------------------------*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* 开发阶段：停在此处便于调试。发布版可改走 cm_backtrace。 */
    (void) xTask;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        (void) pcTaskName;
    }
}

/*-----------------------------------------------------------
 * configUSE_MALLOC_FAILED_HOOK=1 钩子（运行期零动态分配，理论不会触发）
 *----------------------------------------------------------*/
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

/*-----------------------------------------------------------
 * configUSE_IDLE_HOOK=1 钩子
 *----------------------------------------------------------*/
void vApplicationIdleHook(void)
{
    /* 第一版保持为空；后续可在此做低功耗或喂狗兜底。 */
}

/*-----------------------------------------------------------
 * configASSERT 断言回调
 *----------------------------------------------------------*/
void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
    /* 记录到 RTT 便于现场定位；随后进入死循环。 */
    (void) pcFile;
    (void) ulLine;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
