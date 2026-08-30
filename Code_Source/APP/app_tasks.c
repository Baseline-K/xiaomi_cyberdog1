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

extern float Vbus;                 /* FOC_run.c */
extern volatile uint32_t g_foc_wcet_us;   /* FOC_run.c */

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

    (void) pvParam;

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(3));

        uint32_t uNow = xTaskGetTickCount();

        SafetySlowInput_t in = { 0 };
        in.elapsed_ms = (uint32_t)(uNow - uPrevTick);   /* 实际经过时间（计划 §6.2） */
        uPrevTick = uNow;

        in.is_run = (MotorState_GetCurrent() == S_RUN);
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
    printf("  CAN: rx=%lu overrun=%lu drop=%lu\r\n",
           (unsigned long) rx_total, (unsigned long) rx_overrun, (unsigned long) rx_drop);
    printf("  FSM: posted=%lu dropped=%lu illegal=%lu fault=%lu\r\n",
           (unsigned long) posted, (unsigned long) dropped,
           (unsigned long) illegal, (unsigned long) fault_forced);
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
#define CAN_HB_PERIOD_MS   100U   /* 心跳周期，改这里即可调整发送频率 */
#define CAN_HB_MARKER      0xFEU  /* 心跳标记 data[0]，与分析仪区分 */

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

static void CommTask_Entry(void *pvParam)
{
    (void) pvParam;
    const TickType_t xHbPeriod = pdMS_TO_TICKS(CAN_HB_PERIOD_MS);
    TickType_t xLastHb = xTaskGetTickCount();

    for (;;)
    {
        /* 等待 CAN 帧通知（通知值计数累加，忽略具体值）；带超时以便周期发心跳 */
        xTaskNotifyWait(0UL, 0xFFFFFFFFUL, NULL, xHbPeriod);

        /* 一次性批处理缓冲内全部帧（内部 while PopFrame） */
        CANopen_OD_Process();

        /* 周期发送心跳测试帧（验证 TX，分析仪可见） */
        if ((xTaskGetTickCount() - xLastHb) >= xHbPeriod)
        {
            xLastHb = xTaskGetTickCount();
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
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   /* 板载 LED 500ms 翻转，验证调度器存活 */
        /* RTT 心跳已关闭（保留 CAN 心跳，见 CommTask_Entry CAN_SendHeartbeat） */
    }
}

/*-----------------------------------------------------------
 * 创建所有 Phase 1 静态任务
 *----------------------------------------------------------*/
void AppTasks_Init(void)
{
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
