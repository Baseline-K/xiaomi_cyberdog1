/**
 * Board_Config.h
 *
 * 板级常量集中定义（只放"硬件决定、与具体外设句柄无关"的常数）。
 *
 * 为什么需要这个文件：
 *   CubeMX 在 User Constants 里写的是**别名**，真值必须由工程自己给。
 *   例如 Xiaomi_CyberDog1.ioc 里 `TIM4.Period=wwdg_feed_interval_100us - 1`，
 *   生成的 Core/Inc/main.h 只有
 *       #define wwdg_feed_interval_100us WWDG_FEED_INTERVAL_100us
 *   而 WWDG_FEED_INTERVAL_100us 本身没有任何地方定义 —— 必须在 CubeMX 之外补上，
 *   否则 Core/Src/tim.c 编译不过（undeclared identifier）。
 *   放在这里而不是 main.h，是为了让 CubeMX 再生成时不覆盖、且板级常量集中可查。
 *
 * 引用方式：Core/Inc/main.h 的 `USER CODE BEGIN Includes` 里 include 本文件。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

/*-----------------------------------------------------------
 * 看门狗喂狗间隔
 *
 * TIM4 时钟 = APB1 定时器时钟 = 120MHz；PSC = 12000-1 → 计数频率 10kHz
 *          → 一个 tick = 100us，与宏名后缀 "_100us" 一致。
 * CubeMX 把 (本值 - 1) 写进 TIM4 ARR，即本值 = "多少个 100us"。
 *   本值 50 → ARR = 49 → 每 5ms 产生一次更新中断。
 *
 * 用途：调度器启动前的一段阻塞初始化（DRV83XX_Init / Offset_Current_Start 等，
 *       合计约 670ms，最长单段 300ms）期间，由 TIM4 中断喂 IWDG；
 *       调度器启动后 main_user() 关掉 TIM4 更新中断，改由 SafetyTask(3ms) 喂狗。
 *       绝不能两处同时喂 —— 否则"SafetyTask 卡死"永远查不出来。
 *----------------------------------------------------------*/
#define WWDG_FEED_INTERVAL_100us   ((uint16_t)50U)   /* 50 x 100us = 5ms */

/*-----------------------------------------------------------
 * IWDG 超时
 * 超时 = (Reload + 1) x Prescaler / LSI(40000)
 *      = (50 + 1) x 32 / 40000 = 40.8ms      （.ioc: IWDG.Prescaler=32, IWDG.Reload=50）
 * 余量：初始化期间 40.8ms / 5ms ≈ 8 倍；运行期 40.8ms / 3ms ≈ 13 倍。
 * 注意：改这里必须同步改 Xiaomi_CyberDog1.ioc 的 IWDG.Reload，否则再生成会被覆盖。
 *----------------------------------------------------------*/
#define IWDG_TIMEOUT_MS            (40U)

#endif /* BOARD_CONFIG_H */
