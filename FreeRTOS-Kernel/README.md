# FreeRTOS-Kernel (vendored)

本目录为 FreeRTOS Kernel 的固定版本快照，按《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§11 要求剪枝后入库，仅保留本工程所需文件。

> 存放位置说明：`FreeRTOS-Kernel/` 独立于项目根目录，不使用 `Middlewares/` 名称。
> 原因是 CubeMX 再生成时会清理它认识的 `Middlewares/` 目录（曾导致本目录被删）。

## 版本信息

| 项 | 值 |
|---|---|
| 版本标签 | `V11.3.0` |
| 上游提交 | `9b777ae5c5b8e9e456065a00294d1e5f5f9facf5`（tag `V11.3.0` peeled） |
| 来源 | <https://github.com/FreeRTOS/FreeRTOS-Kernel> |
| 许可证 | MIT，见本目录 `LICENSE.md` |
| 获取时间 | 2026-08-18 |

## 剪枝内容

相比上游 `V11.3.0`，移除了与 GD32F303/STM32F1 工程无关的内容：

- `portable/` 仅保留 `GCC/ARM_CM4F`（Cortex-M4F、GCC 工具链）。`MemMang/` 整个移除：`configSUPPORT_DYNAMIC_ALLOCATION=0` 时内核不引用 `pvPortMalloc`，`heap_*.c` 均会 `#error` 拒绝编译，故不链接任何 heap 文件。
- 移除 `croutine.c`（已废弃的协程实现，本工程不使用）。
- 移除 examples、MISRA、SPDX、CI 等文档与构建文件。
- 内核 `.c` 源文件保留：`tasks.c`、`queue.c`、`list.c`、`timers.c`、`event_groups.c`、`stream_buffer.c`。

## 与上游差异

无源码修改。若需升级版本，重新获取上游该 tag 并按上述剪枝清单处理。

## 构建接入

由顶层 `CMakeLists.txt` 列出以下文件加入编译：

- `tasks.c` / `queue.c` / `list.c` / `timers.c` / `event_groups.c` / `stream_buffer.c`
- `portable/GCC/ARM_CM4F/port.c`
- 不链接任何 `MemMang/heap_*.c`（见上）

`FreeRTOSConfig.h` 位于 `Core/Inc/`（CubeMX 惯例位置），include 路径已含 `Core/Inc`。

## 配置要点（对应计划 §8）

- tick 1 kHz，`configCPU_CLOCK_HZ = SystemCoreClock`（120 MHz）
- `configSUPPORT_STATIC_ALLOCATION = 1`、`configSUPPORT_DYNAMIC_ALLOCATION = 0`（运行期零动态分配）
- `configMAX_PRIORITIES = 6`
- `configUSE_TIMERS = 0`（第一版不用软件定时器）
- SysTick 交给 FreeRTOS，HAL 时基已由 CubeMX 迁移到 TIM6（`Core/Src/stm32f1xx_hal_timebase_tim.c`）
