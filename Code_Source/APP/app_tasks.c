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
#include "motor_state_machine.h"

/*-----------------------------------------------------------
 * 任务栈/TCB 静态内存与句柄
 *----------------------------------------------------------*/
static StackType_t   BringUpTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t  BringUpTaskTCB;
static TaskHandle_t  BringUpTaskHandle;

static StackType_t   LegacyMainTaskStack[configMINIMAL_STACK_SIZE * 4];  /* 2 KB，含 printf/RTT 调用链 */
static StaticTask_t  LegacyMainTaskTCB;
static TaskHandle_t  LegacyMainTaskHandle;

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
    uint32_t   ulCount = 0U;

    (void) pvParam;

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(500));

        ulCount++;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   /* 板载 LED 500ms 翻转，验证调度器存活 */

        if ((ulCount & 1U) == 0U)   /* 每 1s 输出一次心跳 */
        {
            SEGGER_RTT_printf(0, "BringUp alive, cnt=%lu, tick=%lu\r\n",
                              (unsigned long) ulCount,
                              (unsigned long) xTaskGetTickCount());
        }
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

    /* LegacyMainTask：优先级 2，承载原 while(1) 轮询（MotorCtrl/RTT） */
    LegacyMainTaskHandle = xTaskCreateStatic(LegacyMainTask,
                                             "LegacyMain",
                                             sizeof(LegacyMainTaskStack) / sizeof(StackType_t),
                                             (void *) NULL,
                                             2,
                                             LegacyMainTaskStack,
                                             &LegacyMainTaskTCB);
    configASSERT(LegacyMainTaskHandle != NULL);

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
