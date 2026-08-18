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

/*-----------------------------------------------------------
 * 任务栈/TCB 静态内存与句柄
 *----------------------------------------------------------*/
static StackType_t   BringUpTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t  BringUpTaskTCB;
static TaskHandle_t  BringUpTaskHandle;

static StackType_t   LegacyMainTaskStack[configMINIMAL_STACK_SIZE * 4];  /* 2 KB，含 printf/RTT 调用链 */
static StaticTask_t  LegacyMainTaskTCB;
static TaskHandle_t  LegacyMainTaskHandle;

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
    /* BringUpTask：优先级 1，验证内核运行 */
    BringUpTaskHandle = xTaskCreateStatic(BringUpTask_Entry,
                                          "BringUp",
                                          sizeof(BringUpTaskStack) / sizeof(StackType_t),
                                          (void *) NULL,
                                          1,
                                          BringUpTaskStack,
                                          &BringUpTaskTCB);
    configASSERT(BringUpTaskHandle != NULL);

    /* LegacyMainTask：优先级 2，承载原 while(1) 轮询（CAN/MotorCtrl/RTT） */
    LegacyMainTaskHandle = xTaskCreateStatic(LegacyMainTask,
                                             "LegacyMain",
                                             sizeof(LegacyMainTaskStack) / sizeof(StackType_t),
                                             (void *) NULL,
                                             2,
                                             LegacyMainTaskStack,
                                             &LegacyMainTaskTCB);
    configASSERT(LegacyMainTaskHandle != NULL);
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
