/*
 * FreeRTOSConfig.h
 *
 * 工程：Xiaomi CyberDog 电机驱动器
 * 平台：GD32F303RET6（Cortex-M4F，120 MHz），STM32F1 HAL 兼容层
 * 对应《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§8 / §4
 *
 * 说明：
 * - 第一版关闭动态分配（configSUPPORT_DYNAMIC_ALLOCATION=0），运行期零动态内存。
 * - 第一版不启用软件定时器（configUSE_TIMERS=0）。
 * - 开发阶段打开 configASSERT 与栈溢出钩子。
 * - 中断优先级：NVIC 全抢占分组（4 bits，无子优先级），
 *   优先级 0~4 的 ISR 不得调用 FreeRTOS API（见计划 §4 表）。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * 相关外设头文件（提供 SystemCoreClock / CMSIS 内核定义）
 *----------------------------------------------------------*/
#include "stm32f1xx.h"

/*-----------------------------------------------------------
 * 内核基本配置（计划 §8）
 *----------------------------------------------------------*/
#define configUSE_PREEMPTION                      1
#define configUSE_TIME_SLICING                    1
#define configUSE_IDLE_HOOK                       1
#define configUSE_TICK_HOOK                       0
#define configCPU_CLOCK_HZ                        ( SystemCoreClock )   /* 120 MHz，运行时读取 */
#define configTICK_RATE_HZ                        ( 1000 )              /* 1 kHz tick */
#define configMAX_PRIORITIES                      ( 6 )
#define configMINIMAL_STACK_SIZE                  ( 128 )               /* Idle 任务，单位 word */
#define configMAX_TASK_NAME_LEN                   ( 16 )
#define configUSE_16_BIT_TICKS                    0
#define configIDLE_SHOULD_YIELD                   1
#define configUSE_MUTEXES                         1
#define configUSE_RECURSIVE_MUTEXES               0
#define configUSE_COUNTING_SEMAPHORES             0
#define configUSE_TASK_NOTIFICATIONS              1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION   1
#define configUSE_TICKLESS_IDLE                   0
#define configUSE_TRACE_HOOKS                     1   /* SystemView 任务切换跟踪 */

/*-----------------------------------------------------------
 * 内存管理（计划 §8：静态分配，运行期零动态分配）
 *----------------------------------------------------------*/
#define configSUPPORT_STATIC_ALLOCATION           1
#define configSUPPORT_DYNAMIC_ALLOCATION          0
#define configTOTAL_HEAP_SIZE                     ( 2048 )   /* heap_1，纯静态下会被 --gc-sections 回收 */

/*-----------------------------------------------------------
 * 软件定时器（第一版不使用，计划 §8）
 *----------------------------------------------------------*/
#define configUSE_TIMERS                          0
#define configTIMER_TASK_PRIORITY                 ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                  10
#define configTIMER_TASK_STACK_DEPTH              ( configMINIMAL_STACK_SIZE )

/*-----------------------------------------------------------
 * 调试与错误检测（开发阶段开启，计划 §4.2）
 *----------------------------------------------------------*/
#define configCHECK_FOR_STACK_OVERFLOW            2
#define configUSE_MALLOC_FAILED_HOOK              1
#define configQUEUE_REGISTRY_SIZE                 8

/* 断言：失败时调用 vAssertCalled（由应用实现）并停在此处 */
extern void vAssertCalled(const char *pcFile, unsigned long ulLine);
#define configASSERT( x ) if( ( x ) == 0 ) { vAssertCalled( __FILE__, __LINE__ ); taskDISABLE_INTERRUPTS(); for( ;; ); }

/*-----------------------------------------------------------
 * 运行时统计（uxTaskGetSystemState 输出各任务 CPU%）
 * 计时源：DWT 周期计数器（120 MHz，已由 SEGGER_RTT_Port 的 DWT_Init 使能）
 * 注意：vTaskGetRunTimeStats（格式化函数）需要动态分配，本项目纯静态，
 *       改用 uxTaskGetSystemState 自行格式化。
 *----------------------------------------------------------*/
#define configUSE_TRACE_FACILITY           1
#define configGENERATE_RUN_TIME_STATS      1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()   /* DWT 已使能，无需额外配置 */
#define portGET_RUN_TIME_COUNTER_VALUE()           ( DWT->CYCCNT )

/*-----------------------------------------------------------
 * 中断优先级（计划 §4，GD32F303 __NVIC_PRIO_BITS = 4）
 *----------------------------------------------------------*/
#define configPRIO_BITS                            4

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY    15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* 新版端口别名 */
#define configMAX_API_CALL_INTERRUPT_PRIORITY      configMAX_SYSCALL_INTERRUPT_PRIORITY

/*-----------------------------------------------------------
 * 兼容旧宏名称（V11 仍支持）
 *----------------------------------------------------------*/
#define configENABLE_BACKWARD_COMPATIBILITY        1

/*-----------------------------------------------------------
 * Direct Routing（V11.3.0 端口要求，configCHECK_HANDLER_INSTALLATION=1）
 * 把端口处理器函数重命名为向量表名，使向量表直接指向
 * vPortSVCHandler / xPortPendSVHandler / xPortSysTickHandler，
 * 满足 xPortStartScheduler 里的向量安装断言。
 * 注意：stm32f1xx_it.c 中不得再定义 SVC_Handler / PendSV_Handler / SysTick_Handler。
 *----------------------------------------------------------*/
#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler

/*-----------------------------------------------------------
 * 说明：vApplicationGetIdleTaskMemory / 各钩子回调由应用实现（app_tasks.c）。
 * 静态内存回调的前置声明由内核 tasks.c 在包含 task.h 后自行提供，
 * 不在本配置文件中声明（此处类型尚未定义）。
 *----------------------------------------------------------*/

/* 供 configASSERT 与钩子引用的宏（FreeRTOS.h 引入后可展开） */
#define INCLUDE_vTaskPrioritySet           1
#define INCLUDE_uxTaskPriorityGet          1
#define INCLUDE_vTaskDelete                0
#define INCLUDE_vTaskSuspend               1
#define INCLUDE_vTaskDelayUntil            1
#define INCLUDE_vTaskDelay                 1
#define INCLUDE_xTaskGetSchedulerState     1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/*-----------------------------------------------------------
 * SystemView 集成：在文件末尾引入 FreeRTOS 桥（桥头定义 trace 钩子映射）
 *----------------------------------------------------------*/
#include "SEGGER_SYSVIEW_FreeRTOS.h"

#endif /* FREERTOS_CONFIG_H */
