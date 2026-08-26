/**
 * app_tasks.h
 * FreeRTOS 应用任务创建接口。
 * 对应《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§3.2 静态任务创建。
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "FreeRTOS.h"
#include "task.h"   /* TaskHandle_t */

/* CommTask 句柄：CAN RX ISR（stm32f1xx_it.c）用 vTaskNotifyGiveFromISR 通知。
 * 调度器启动前为 NULL，ISR 需判空保护。 */
extern TaskHandle_t xCommTaskHandle;

/* 创建全部静态任务（BringUpTask + CommTask + MotorStateTask + SafetyTask + DiagTask）。
 * 在 vTaskStartScheduler() 之前调用；所有任务均为 xTaskCreateStatic。 */
void AppTasks_Init(void);

/* 打印系统诊断（栈高水位 / FOC WCET / CAN 统计 / 状态机统计），供 RTT `diag` 命令调用 */
void AppTasks_Diag(void);

#endif /* APP_TASKS_H */
