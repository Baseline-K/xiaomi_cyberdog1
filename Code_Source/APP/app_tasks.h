/**
 * app_tasks.h
 * FreeRTOS 应用任务创建接口。
 * 对应《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§3.2 静态任务创建。
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "task.h"   /* TaskHandle_t */

/* CommTask 句柄：CAN RX ISR（stm32f1xx_it.c）用 vTaskNotifyGiveFromISR 通知。
 * 调度器启动前为 NULL，ISR 需判空保护。 */
extern TaskHandle_t xCommTaskHandle;

/* 创建 Phase 1 的静态任务（BringUpTask + LegacyMainTask + CommTask）。
 * 在 vTaskStartScheduler() 之前调用；所有任务均为 xTaskCreateStatic。 */
void AppTasks_Init(void);

/* LegacyMainTask：Phase 1 过渡任务，承载原 main_user() 的 while(1) 轮询循环。
 * 实现在 main_user.c；此处仅声明供 app_tasks.c 创建任务使用。 */
void LegacyMainTask(void *pvParam);

#endif /* APP_TASKS_H */
