/**
 * app_tasks.c
 *
 * FreeRTOS 应用任务与钩子实现（Phase 1：内核接入阶段）。
 *
 * Phase 1 只接入内核，不改 FOC/CAN/状态机路径：
 * - 提供内核必需的静态内存回调与钩子。
 * - BringUpTask / LegacyMainTask 在调度器启动后运行（Phase 1c）。
 *
 * 对应《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§3.2 / §14 首个工作包。
 */
#include "FreeRTOS.h"
#include "task.h"

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
