> 2026-09-27 审查说明：后续章节保留历史实施记录与当时勾选状态；涉及采样已可靠、FOC 故障已消除的结论，须以 §18.7 未完成项和重新验收结果为准。

# FreeRTOS 混合式电机控制架构迁移计划

> 项目：Xiaomi CyberDog 电机驱动器  
> 目标平台：GD32F303RET6（Cortex-M4F，120 MHz；工程保留 STM32F1 HAL 兼容层）  
> 目标架构：PWM/ADC 同步中断执行硬实时 FOC与快速安全检测，FreeRTOS 管理状态机、慢速安全检测、通信和诊断  
> RTOS 选择：FreeRTOS，1 kHz 系统 tick  
> 文档日期：2026-08-16  
> 状态：迁移已实施多阶段，尚未完成验收；2026-09-27 静态复核发现 ADC3 中断保持与电流校准迁移缺口，见 §18.7。

---

## 1. 结论与迁移原则

本次迁移保留现有 FOC 算法和业务状态机思路，不把 10 kHz 电流环改成 RTOS 任务。FreeRTOS 只负责对实时性要求较低、适合阻塞等待和事件驱动的工作。

核心原则：

1. **先修复控制问题，再迁移架构。** RTOS 不能修复相序、电角度、电流采样、PI 参数或模型接口错误。
2. **10 kHz FOC 保留在 ADC 注入转换完成中断。** 周期由 TIM1/PWM 和 ADC 硬件触发，不依赖 RTOS tick。
3. **FreeRTOS tick 保持 1 kHz。** 不为 10 kHz FOC 将 tick 提高到 10 kHz。
4. **状态机只有一个写入者。** 所有普通状态转换由 `MotorStateTask` 完成。
5. **故障关断不等待任务。** 硬件 Break 或高优先级 ISR 先关闭 PWM并锁存故障，任务只做后续状态转换、记录和恢复判断。
6. **分阶段替换，每阶段均可回退。** 不在一次提交中同时更换调度、通信、状态机和控制算法。
7. **优先使用静态分配。** 电机控制固件不在正常运行期动态创建任务或申请堆内存。
8. **参考项目按职责移植，不逐行复制。** 保留状态/事件/Enter-Do-Exit及快慢安全分层，重写不适合RTOS的调度与并发部分。

### 1.1 不属于本次迁移的内容

- 不重新设计 FOC 算法、Clarke/Park、PI、SVPWM 或死区补偿。
- 不通过 RTOS 解决当前“速度为 0、电流很大”的问题。
- 第一阶段不把速度环、位置环从生成代码中拆出。
- 不直接移植 VESC 的 ChibiOS HAL、线程或分散状态变量。
- 不让 ADC/FOC ISR 调用 FreeRTOS API。
- 不原样复制参考项目的单 pending事件槽、全局协程宏和长时间全局关中断代码。

---

## 2. 迁移前历史基线（当前外设状态见 §18.7）

当前工程结构如下：

- `HAL_ADCEx_InjectedConvCpltCallback()` 执行电角度读取、电流采样和 `FOC_Generated_Step()`。
- FOC 由 PWM/ADC 同步触发，目标频率为 10 kHz，即每周期 100 us。
- `main_user.c` 的 `while (1)` 使用 `HAL_GetTick()` 每 1 ms 调用：
  - `CANopen_OD_Process()`
  - `MotorCtrl_Process()`
  - `RTT_Cmd_Process()`
- CAN RX ISR 将帧放入 16 项环形缓冲，主循环弹出并解析。
- `MotorCtrl` 目前包含命令影子值、运行命令、`STOPPED/RUNNING` 状态和故障字。
- `StartPWM()`/`StopPWM()` 负责 PWM、ADC中断和模型状态的启动/停止。

状态机与安全模块的移植参考源：

- `E:\BaiduSyncdisk\AND_Engineering_Project\AC_Control\AC_Control_400W_New\AC_Control_NewPCB\AND_Source\APP\StateMachine.c/.h`
- `E:\BaiduSyncdisk\AND_Engineering_Project\AC_Control\AC_Control_400W_New\AC_Control_NewPCB\AND_Source\APP\main_user.c`
- `E:\BaiduSyncdisk\AND_Engineering_Project\AC_Control\AC_Control_400W_New\AC_Control_NewPCB\AND_Source\APP\fsm.c`
- `E:\BaiduSyncdisk\AND_Engineering_Project\AC_Control\AC_Control_400W_New\AC_Control_NewPCB\AND_Source\SERVICE\Safety_Module.c/.h`
- `E:\BaiduSyncdisk\AND_Engineering_Project\AC_Control\AC_Control_400W_New\AC_Control_NewPCB\AND_Source\SERVICE\Logger_FaultLog.c/.h`

参考项目提供了以下可复用设计：

- 表驱动的状态、事件与 `Enter/Do/Exit` 动作映射。
- `INIT/IDLE/CALIB/RUN/STOP/FAULT_NOW/FAULT_OVER/STANDBY` 业务状态。
- FOC快环中的 `Safety_FastStep()`。
- 主循环约每3 ms执行的 `Safety_SlowStep()`。
- 紧急故障/常规故障掩码、故障快照和故障日志思路。

这些代码作为功能需求和行为参考，不作为可直接复制的RTOS实现。

相关文件：

- `Code_Source/APP/FOC_run.c`
- `Code_Source/APP/main_user.c`
- `Code_Source/SERVICE/MotorCtrl.c`
- `Code_Source/SERVICE/MotorCtrl.h`
- `Code_Source/BSP/CAN_bsp.c`
- `Code_Source/APP/CANopen_OD.c`
- `Core/Src/stm32f1xx_it.c`

当前 Debug 构建约占用：

- Flash：约 64 KiB（不含未链接段）
- RAM 静态数据：约 10 KiB
- 链接区域：512 KiB Flash、64 KiB RAM

RAM 足以支持一个精简 FreeRTOS 配置，但最终任务栈必须通过高水位实测确定，不能只按估算值发布。

### 2.1 Phase 0 实测基线（2026-08-17，提交 3ce8d53）

速度环恢复后实测（Debug 构建 `-Og -g3`，工具链 gnu-tools-for-stm32 14.3.1）：

| 项目 | 实测值 | 说明 |
|---|---|---|
| FOC 10 kHz ADC ISR | ~60 us / 周期（典型值） | 预算 100 us；avg/P99/max 的正式统计待 Phase 1 用 RTT 采集补充 |
| `bsp_as5600GetAngle()` 软件 I2C | ~37 us | 占 ISR 60%，是 WCET 大头；Phase 1 后单独评估硬件 SPI/DMA 或降采样+角度预测 |
| Flash 占用 | 65044 B ≈ 63.5 KiB | text 64100 + data 944；512 KiB 的 12.4% |
| RAM 静态占用 | 9904 B ≈ 9.7 KiB | data 944 + bss 8960；64 KiB 的 15.2% |

Phase 0 控制修复项已通过验收（速度环正反转受控、Start/Stop 无残留积分）。FOC ISR 最坏时间在 100 us 预算的 60%，满足验收门，但余量仅约 40 us，Phase 4 加入 `Safety_FastStep()` 前需复核。

### 2.2 CubeMX 再生成后需重做事项（2026-08-18）

CubeMX 时基改为 TIM6 后实测再生成会带来以下改动，每次再生成后需复查：

- **历史项已更新：当前保留 `MX_IWDG_Init()`。** PSC32/Reload50 按 40 kHz LSI 估算超时 40.8 ms；初始化期由 TIM4 每 5 ms 喂狗，启动调度器前关闭 TIM4 更新中断，运行期由 SafetyTask 喂狗。再生成后核查交接，不应再按早期步骤注释 IWDG；实际超时需考虑 LSI 容差。
- **`Core/Src/stm32f1xx_it.c`：SVC/PendSV/SysTick 空壳处理器会被重新生成**。当前采用 **Direct Routing**（`FreeRTOSConfig.h` 中 `#define vPortSVCHandler SVC_Handler` 等，端口函数直接占用向量名），因此这三个空壳必须删除，否则与端口重命名函数重复符号；再生成后需再次删除。HAL timebase 已迁 TIM6，`SysTick_Handler` 删除无副作用。
- **`Core/Src/tim.c`：`TIM1_BRK_IRQHandler` 及其 NVIC 使能会被移除**。当前 `BreakState=TIM_BREAK_DISABLE`，硬件 Break 未用，无影响；Phase 4 若启用 Break 需重建该中断。
- **`cmake/stm32cubemx/CMakeLists.txt`** 会加回 `stm32f1xx_hal_can.c`，由顶层 CMake 的 GD32 补丁自动替换，无需手工处理。
- **`Middlewares/` 目录会被清理**——FreeRTOS-Kernel 现独立放根目录 `FreeRTOS-Kernel/`，不受影响。

---

## 3. 目标架构

```text
                        ┌──────────────────────────────┐
TIM1 PWM ──触发 ADC ──>│ ADC ISR，10 kHz             │
                        │ 采样/角度/FastSafety/FOC     │
                        │ SVPWM/写 CCR/紧急关断        │
                        │ 不阻塞、不分配内存、不调 RTOS │
                        └──────────────┬───────────────┘
                                       │ 命令快照/遥测快照
                                       ▼
┌─────────────┐   event queue   ┌──────────────────────┐
│ CommTask    │ ───────────────>│ MotorStateTask       │
│ CAN协议解析 │                  │ 唯一状态机所有者     │
└─────────────┘                  │ Start/Stop/Fault     │
                                 └──────────┬───────────┘
┌─────────────┐                            │ 发布控制命令
│ SafetyTask  │ ── fault/timeout event ───┤
│ 3 ms慢安全  │                            ▼
└─────────────┘                  ┌──────────────────────┐
                                 │ MotorCommand 快照    │
┌─────────────┐                  │ ISR只读，任务单写    │
│ DiagTask    │ <── telemetry ──└──────────────────────┘
│ RTT/统计    │
└─────────────┘

硬件 Break/高优先级故障 ISR：立即关闭 PWM → 锁存 fault → 延后通知状态机
```

### 3.1 执行上下文与职责

| 上下文 | 建议频率/触发方式 | 职责 | 禁止事项 |
|---|---:|---|---|
| TIM1 Break / 紧急故障 ISR | 异步 | 硬件关断、PWM失能、故障锁存 | 日志、协议、状态机回调、RTOS API |
| ADC/FOC ISR | 10 kHz硬件触发 | 采样、角度、`Safety_FastStep`、FOC、SVPWM、写 CCR、快照遥测 | 阻塞、动态内存、队列、printf、普通 HAL 延时 |
| `SafetyTask` | 3 ms绝对周期 | `Safety_SlowStep`、故障锁存、向状态机报告、健康汇总 | 直接恢复故障、直接重新开PWM |
| `MotorStateTask` | 事件驱动，最长 1 ms 巡检 | 状态迁移、Start/Stop、故障恢复策略、命令发布 | 直接执行高速控制 |
| `CommTask` | CAN通知驱动 | CAN帧解析、SDO应答、生成状态机事件 | 在 CAN ISR 做协议解析 |
| `DiagTask` | 10~100 Hz或事件驱动 | RTT命令、日志、运行统计、栈水位 | 高优先级持续运行 |
| 可选 `ControlTask` | 500 Hz~1 kHz | 后续拆出的速度/位置外环 | 电流环和PWM更新 |

### 3.2 初始任务配置

第一版仅创建必要任务，避免无意义地“一功能一任务”。以下为初始预算，单位统一按**字节**记录，代码中创建任务时再换算成 `StackType_t` 数量。

| 任务 | FreeRTOS优先级 | 初始栈预算 | 唤醒方式 |
|---|---:|---:|---|
| `SafetyTask` | 5 | 1280 B | `vTaskDelayUntil()`，3 ms |
| `MotorStateTask` | 4 | 1024 B | 状态事件队列 + 1 ms超时 |
| `CommTask` | 3 | 1280 B | CAN任务通知 |
| `DiagTask` | 1 | 1280 B | RTT通知或 10 ms超时 |
| Idle | 0 | 按端口最小值 | 内核管理 |

要求：

- `configMAX_PRIORITIES` 初始设为 6。
- 所有应用任务使用 `xTaskCreateStatic()`。
- 队列、事件组、软件定时器如需使用，也采用静态创建接口。
- 发布前启用并记录 `uxTaskGetStackHighWaterMark()`，将每个栈保留至少 30%余量。
- 第一版不启用 FreeRTOS 软件定时器；周期任务使用 `vTaskDelayUntil()`。

### 3.3 与《电机驱动器RTOS架构建议》文档的映射

`Docs/电机驱动器RTOS架构建议.md` 作为长期总体框架，与本计划的关系如下（本计划是它的分阶段落地路径）：

| 架构建议文档 | 本计划 | 说明 |
|---|---|---|
| MotorManagerTask | `MotorStateTask` | 状态机/启停/故障处理唯一所有者，概念一致 |
| CommunicationTask | `CommTask` | 一致 |
| DiagnosticTask | `DiagTask` | 一致 |
| WatchDogTask | 计划 §6.2 的"独立健康汇总喂狗" | 架构文档建议独立任务；计划允许先并入 DiagTask，稳定后再拆 |
| ScopeTask | 现有 SEGGER RTT PLUS/J-Scope | 由 DiagTask 承担，控制 ISR 只写快照 |
| ParameterTask | Phase 6 之后 | Flash 参数持久化，与计划"参数更新仅在PWM关闭"一致 |
| LogTask | `Logger_FaultLog` + DiagTask | 一致 |
| 控制环分频 | 计划 §Phase 6 候选方案 A | 架构文档给出 40k/4k/2k/1k 示例；本工程按 10k/1k 分频 |

需要纳入本计划的具体落点（已在对应章节体现或待实现时遵守）：

- **命令快照结构**：架构文档 §13.3 的 `MotorReference_t`（position/speed/torque/iq_ref + control_mode + enable）作为 `MotorCommand` 快照的基准结构；ISR 每周期读快照，绝不等 Mutex。
- **遥测快照结构**：架构文档 §13.4 的 `MotorRealtimeState_t`（position/speed/iq/id/torque/vbus）作为遥测快照基准结构。
- **事件通知优先 Task Notification**：架构文档 §13.1 建议单事件优先用 Task Notification（开销小）；本计划 §4.1/§6.6 的故障通知即采用任务通知位兜底。
- **ISR 绝不等 Task**：架构文档 §14 与本计划 §7 完全一致。
- **优先级次序**：控制实时性 > 指令/状态管理 > 通信 > 诊断 > 调试/日志/Flash，与本计划 §3.1 任务表一致。

---

## 4. 中断与 FreeRTOS 优先级设计

Cortex-M NVIC 中数值越小，硬件中断优先级越高；FreeRTOS任务优先级则数值越大越高，两者不可混淆。

建议初始配置（最终以 GD32F303 实际 `__NVIC_PRIO_BITS` 验证结果为准）：

```c
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
```

| 中断 | 建议 NVIC优先级 | 可否调用 `...FromISR()` | 说明 |
|---|---:|---|---|
| TIM1 Break | 0 | 否 | 最高安全级，直接关断 |
| ADC3 FOC（当前；早期为 ADC1/2） | 1 | 否 | 10 kHz硬实时控制；中断保持问题见 §18.7 |
| TIM1 Update（如保留） | 2 | 否 | 仅硬实时辅助 |
| SPI DMA（若通知任务） | 6 | 是 | 必须使用 FromISR API |
| CAN RX/TX | 6 | 是 | ISR入缓冲并通知 `CommTask` |
| UART | 7 | 是 | 仅在需要任务通知时使用 |
| SysTick/PendSV | 15 | 内核专用 | 由 FreeRTOS端口配置 |

迁移要求：

1. 使用全抢占优先级分组，不使用子优先级混排。
2. 优先级 0~4 的 ISR 不得调用任何 FreeRTOS API。
3. 调用 FreeRTOS API 的 ISR 必须使用 `...FromISR()` 版本，并检查是否需要 `portYIELD_FROM_ISR()`。
4. 开发阶段启用 `configASSERT()`，捕获错误的中断优先级和 API调用。
5. 当前 CAN RX/TX/SCE 均为 6，SPI/DMA 为 2；后者不调用 RTOS API 时可保留，若增加 FromISR 通知则需调整到 5~15。TIM4=4 仅作初始化期喂狗，TIM6=15 提供 HAL 时基。

### 4.1 高优先级故障如何通知状态机

TIM1 Break 和 ADC ISR位于 RTOS可管理中断范围以上，不能直接发送 FreeRTOS队列。采用两段式处理：

```text
高优先级 ISR：
  1. 关闭 PWM/MOE
  2. 锁存 fault_bits
  3. 设置 fault_pending = true

MotorStateTask（队列等待最长1 ms超时）：
  1. 读取并清除 pending标志
  2. 发送或直接处理 EVENT_FAULT
  3. 进入 FAULT，记录原因并禁止重新启动
```

安全关断延迟由硬件/ISR保证；任务只影响故障上报延迟。

---

## 5. 状态机设计

### 5.1 从参考项目迁移的状态

```text
BOOT → INIT → IDLE ───────────────> RUN ──> STOP ──> IDLE
          │      ├──> CALIB ──────> IDLE      │
          │      └──> STANDBY ────> IDLE      │
          │                                  │
          └──────── 可运行状态 ───────────────┴──> FAULT_ACTIVE
                                                    │ 故障源恢复/确认
                                                    ▼
                                              FAULT_RECOVERY
                                                    │ 电机静止+清故障授权
                                                    ▼
                                                   IDLE
```

目标状态与参考项目映射：

| 参考项目 | 目标状态 | 迁移说明 |
|---|---|---|
| 上电初始化前 | `MOTOR_STATE_BOOT` | 仅内核/BSP启动，不执行普通业务动作 |
| `S_INIT` | `MOTOR_STATE_INIT` | 参数加载、Safety初始化、传感器初始化 |
| `S_IDLE` | `MOTOR_STATE_IDLE` | PWM安全关闭，允许配置和校准请求 |
| `S_CALIB` | `MOTOR_STATE_CALIBRATING` | 电流偏置/编码器/参数校准；优先改成非阻塞子步骤 |
| `S_RUN` | `MOTOR_STATE_RUNNING` | 发布使能命令并启动PWM/FOC |
| `S_STOP` | `MOTOR_STATE_STOPPING` | 受控停机；完成后回到IDLE |
| `S_FAULT_NOW` | `MOTOR_STATE_FAULT_ACTIVE` | 已完成紧急关断，执行上报和恢复检测 |
| `S_FAULT_OVER` | `MOTOR_STATE_FAULT_RECOVERY` | 故障源已恢复但尚未授权重新运行 |
| `S_STANDBY` | `MOTOR_STATE_STANDBY` | 可选；仅在有明确低功耗需求时保留 |

保留参考项目的表驱动方式和 `Enter/Do/Exit` 接口，但重写状态机内核。`Do`动作由 `MotorStateTask` 在队列等待超时后周期执行；长流程必须拆为可中断的子步骤，不能长时间阻塞故障事件。

### 5.2 推荐事件

- `MOTOR_EVT_INIT_DONE`
- `MOTOR_EVT_CALIB_DONE`
- `MOTOR_EVT_START`
- `MOTOR_EVT_STOP`
- `MOTOR_EVT_FAULT`
- `MOTOR_EVT_CLEAR_FAULT`
- `MOTOR_EVT_COMM_TIMEOUT`
- `MOTOR_EVT_PARAM_CHANGED`
- `MOTOR_EVT_RECOVERY_READY`
- `MOTOR_EVT_WORK_DONE`

事件结构至少包含：

```c
typedef struct {
    MotorEventId id;
    uint32_t timestamp_ms;
    uint32_t arg_u32;
    float arg_f32;
} MotorEvent;
```

若后续事件负载变复杂，应改成判别联合体，不要不断增加无类型字段。

### 5.3 状态机并发规则

- 仅 `MotorStateTask` 修改 `current_state`、执行 `Enter/Exit/Do`。
- CAN、RTT和监控模块只产生事件，不直接调用 `StartPWM()`/`StopPWM()`。
- ADC ISR只读取已发布命令快照，不读取状态机内部临时变量。
- 故障位采用锁存语义；收到 `CLEAR_FAULT` 后还要检查母线、电流、温度、驱动器故障脚等恢复条件。
- 状态事件队列满时，`FAULT/STOP` 不得静默丢失：使用独立锁存位或任务通知位作为兜底。
- `RUNNING → RUNNING` 的重复 Start应定义为忽略、更新命令或非法事件，不应重复执行完整 Exit/Enter。

### 5.4 参考状态机中需要完善的部分

以下部分不原样迁移：

- `FSM_t` 只有一个 `stPendingEventID/stNextState/u8Flag`，多个事件会覆盖；改为静态 FreeRTOS队列。
- `action_perfrom()` 在 Exit/Enter期间开放中断、最后再提交状态，期间到来的新事件可能被清除；目标实现先从队列取一个事件，再由状态机任务串行完成转换。
- `S_RUN → S_RUN` 当前会重复 Exit/Enter；目标实现明确为命令更新或无操作。
- `EVENT_toFAULT` 不覆盖 `S_INIT`；目标实现区分“初始化失败”和“运行故障”，INIT失败也必须进入安全状态，不能悬停在INIT。
- `PT_*` 使用全局计算跳转协程状态，不可重入；RTOS版本使用状态子步骤、任务阻塞或完成事件，不移植这些宏。
- 状态动作中的 `HAL_Delay()`、Flash写入、传感器重初始化等可能阻塞；只允许在PWM关闭的状态执行，并设置超时。耗时操作必要时交给工作任务，完成后返回 `MOTOR_EVT_WORK_DONE`。
- 状态动作不直接发送CAN或写Flash日志；改为提交消息给 `CommTask/DiagTask`，防止阻塞状态机。
- 状态动作不使用大范围 `__disable_irq()`；PWM关断使用专用、短小、幂等的底层接口。

### 5.5 状态动作迁移顺序

1. 先实现空动作状态表并完成纯软件转换测试。
2. 接入 `IDLE` 的安全PWM关闭动作。
3. 接入 `RUN` 的命令提交、积分器复位和PWM启动动作。
4. 接入 `STOP` 的受控停机和超时强制关断。
5. 接入 `FAULT_ACTIVE/FAULT_RECOVERY`，验证故障锁存后再迁移初始化与校准。
6. 最后迁移 `INIT/CALIB/STANDBY` 中与具体传感器和参数相关的动作。

---

## 6. Safety模块迁移与完善

### 6.1 保留快/慢两级检测

参考项目已经形成合理的检测分层，目标工程按相同职责移植：

| 接口 | 目标执行上下文 | 周期 | 检测项 |
|---|---|---:|---|
| `Safety_FastStep()` | 10 kHz ADC/FOC ISR | 100 us | 母线过压、三相过流、短路 |
| `Safety_SlowStep()` | `SafetyTask` | 3 ms绝对周期 | I²t过载、堵转、缺相、通信超时、IPM/绕组/磁体温度、欠压、速度异常、看门狗诊断 |

快安全检测只使用本周期已经采集的 `vbus` 和三相电流，禁止在 `Safety_FastStep()` 内再次访问慢速外设、打印或调用RTOS。

### 6.2 `SafetyTask` 线程设计

`SafetyTask` 使用 `vTaskDelayUntil()` 保持3 ms绝对周期，并传入实际经过时间：

```c
for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(3));

    SafetySlowInput_t input = {0};
    input.elapsed_ms = actual_elapsed_ms;
    SafetyTelemetry_ReadSnapshot(&input);

    uint32_t before = SafetyFault_Load();
    Safety_SlowStep(&input, &Safety_Config, &local_fault);
    uint32_t new_faults = SafetyFault_Latch(local_fault.all) & ~before;

    if (new_faults != 0U) {
        MotorState_PostFault(new_faults);
    }
    SafetyTask_ReportAlive();
}
```

伪代码只表达所有权，最终接口名称可调整。实现要求：

- `SafetySlowInput_t` 每周期先清零，避免未赋值栈数据进入安全判断。
- 用一致遥测快照替代参考项目中复制多组全局变量时的长 `__disable_irq()`。
- 传入真实 `elapsed_ms`，所有消抖、I²t和超时累计不得假定任务永远准时3 ms。
- 周期超出允许范围（参考项目注释为2~30 ms）时记录 `safety_deadline_miss`；连续超限触发软件故障。
- `SafetyTask`只检测和锁存故障，不直接清故障、不直接启动PWM。
- 通信超时以“有效控制帧序号”推进，不以任意CAN帧或简单消息计数代替。
- 看门狗由独立健康汇总策略喂狗：FOC、Safety、State和Comm均按窗口报告存活后才刷新。

### 6.3 `Safety_FastStep()` 与紧急关断

参考项目的 `Safety_FastStep()` 会设置 `System_Fault`，然后FOC代码再切换 `ctrl_mode=STO`。目标实现进一步收紧：

```text
ADC ISR取得电流/Vbus
  → Safety_FastStep
  → 发现新增 Emergency fault
  → 当场清 TIM1 MOE / 停止门极输出
  → 原子锁存故障位与首故障快照
  → 退出ISR
  → MotorStateTask通过1 ms巡检发现pending并完成业务状态转换
```

要求：

- 硬件比较器/DRV故障/TIM1 Break仍是第一保护层，软件快检不是硬件保护替代品。
- `MotorPwm_EmergencyOffFromISR()` 必须短小、幂等且可从任意电机状态调用。
- 一旦紧急关断，本周期不得继续把新的SVPWM占空比重新使能到输出。
- 紧急故障锁存后只允许明确复位流程恢复，不能因测量值下一周期回落自动清除。
- FastStep的WCET计入100 us FOC总预算。

### 6.4 故障字、原子性与所有权

沿用参考项目32位故障掩码和 `FAULT_EMERGENCY_MASK/FAULT_NORMAL_MASK` 的思路，并进行以下改造：

- 对外序列化使用显式 `uint32_t mask`；位域仅可用于调试视图，不作为跨编译器协议布局保证。
- ISR和任务都可能置位时，普通 `fault->all |= mask` 是读改写竞争；使用真正的原子 fetch-or（编译器原子内建或验证过的 LDREX/STREX封装）。
- 维护 `active_faults`、`latched_faults`、`first_fault` 和可选 `recovered_faults`，不要只保留一个会被直接清零的全局变量。
- Safety模块拥有检测器内部累计量；状态机拥有恢复授权；Logger拥有持久化；CommTask拥有上报，职责不得混合。
- `Safety_Config` 在运行期只读。参数更新仅在PWM关闭状态构造新配置并原子发布，不能让FastStep读取到修改一半的阈值。

### 6.5 慢安全输入映射

| `SafetySlowInput_t` 字段 | 当前工程目标来源 |
|---|---|
| `elapsed_ms` | `SafetyTask` 实际周期 |
| `is_run/is_init` | 状态机只读快照 |
| `phase_current_A[]` | FOC遥测快照 |
| `motor_speed_rps` | 模型/PLL速度反馈 |
| `target_speed_rps` | 已发布的 `MotorCommand` |
| `planning_speed_rps` | 速度斜坡/外环内部参考；未实现前使用明确valid标志 |
| `target_iq_A` | 当前限幅后的 q轴目标 |
| `iq_A/id_A` | FOC遥测快照 |
| `vbus_adc_raw/vbus_V` | ADC慢速通道及标定结果 |
| `ipm_temperature_C` | NTC采样与标定结果 |
| 绕组/磁体温度 | 无传感器/估算器时将available置0，不填伪造温度 |
| `can_rx_valid_seq` | `CommTask`确认有效控制帧后递增的序号 |

若当前项目没有某项传感器或规划参考，必须通过有效标志禁用对应检查，而不是填0后继续判定。

### 6.6 Safety与状态机的故障握手

```text
Safety检测到故障
  → 原子锁存故障位
  → 发送 MOTOR_EVT_FAULT（事件队列满时由fault_pending兜底）
  → MotorStateTask进入 FAULT_ACTIVE
  → PWM保持关闭，CommTask上报，DiagTask抓取一次快照
  → Safety继续判断故障源是否恢复
  → 发送 MOTOR_EVT_RECOVERY_READY
  → 状态机进入 FAULT_RECOVERY
  → 收到CLEAR_FAULT且电机静止/电气条件安全
  → 清除允许清除的active位，保留历史latched记录
  → 返回IDLE
```

不能照搬参考项目在 `state_FAULT_exit()` 中直接执行 `System_Fault.all = 0`；必须逐类定义自动恢复、人工确认和上电复位三种清除策略。

### 6.7 相比参考实现的Safety完善项

- 将参考 `SafetyConfig_t` 中的阈值配置与 `*_accum_ms`、速度环形缓冲等运行状态拆开：`SafetyConfig` 只读，`SafetyRuntime` 由Safety模块独占修改。
- 对全部浮点输入先做 `isfinite()` 检查；NaN/Inf不得利用普通大小比较绕过保护。
- 初始化时校验阈值关系、单位、回差、持续时间和CRC；配置非法直接产生参数故障并禁止RUN。
- 所有毫秒累加器使用饱和加法，避免长时间运行溢出后保护失效。
- 快速过流/短路使用满足噪声要求的本周期电流，不对紧急量施加会显著增加延迟的慢滤波。
- 每类检测器定义触发、保持、恢复回差和清除策略，形成故障策略表。
- 参考实现中尚未完成或注释掉的检测（例如部分缺相逻辑）必须先单元测试和台架验证，不能因为接口存在就默认启用。
- Safety核心函数保持纯计算/状态更新，不包含HAL句柄、CAN发送、Flash写入、printf或任务API。
- 第一版可保留现有函数签名以降低迁移风险；行为验证后再考虑让Fast/SlowStep返回本周期新增掩码，减少对全局 `System_Fault` 的依赖。

---

## 7. ISR与任务之间的数据所有权

### 7.1 控制命令：任务单写、ISR只读

不能让任务逐字段修改 `MotorCtrl`，同时让 FOC ISR在修改过程中读取同一结构体。推荐使用双缓冲或序列号快照：

```text
MotorStateTask：构造完整 next_command → 短临界区发布
ADC ISR：每个FOC周期开始时读取一致的 command snapshot
```

命令至少包括：

- enable
- control mode
- speed reference
- iq/id reference
- position reference
- limits/version

临界区只用于交换索引或复制少量固定字段，不允许包含状态回调、日志或 HAL调用。

### 7.2 遥测：ISR单写、任务只读

FOC ISR发布一个轻量遥测快照：

- `id/iq`
- `speed_meas_rps`
- `theta_elec`
- `duty_u/v/w`
- `vbus`
- FOC执行时间和超限计数

任务通过序列号前后校验读取一致快照。不要在 ISR 中向大队列每周期发送一份遥测；10 kHz 数据应抽取到 100 Hz~1 kHz后再供诊断使用。

### 7.3 CAN数据路径

```text
CAN RX ISR
  → 写 SPSC 环形缓冲
  → vTaskNotifyGiveFromISR(CommTask)

CommTask
  → 批量弹出全部已收帧
  → 解析协议
  → 发送 MotorEvent / 更新影子参数
```

现有 16帧环形缓冲可以第一阶段保留，但需要保留并暴露 `rx_overrun/rx_drop` 统计。

---

## 8. FreeRTOS内核配置建议

初始建议：

```c
#define configCPU_CLOCK_HZ                    SystemCoreClock
#define configTICK_RATE_HZ                    1000
#define configUSE_PREEMPTION                  1
#define configUSE_TIME_SLICING                1
#define configMAX_PRIORITIES                  6
#define configSUPPORT_STATIC_ALLOCATION       1
#define configSUPPORT_DYNAMIC_ALLOCATION      0
#define configUSE_MUTEXES                     1
#define configUSE_RECURSIVE_MUTEXES           0
#define configUSE_COUNTING_SEMAPHORES         0
#define configUSE_TIMERS                      0
#define configCHECK_FOR_STACK_OVERFLOW        2
#define configUSE_MALLOC_FAILED_HOOK          1
#define configUSE_IDLE_HOOK                   1
#define configUSE_TICK_HOOK                   0
```

说明：

- 第一版关闭动态分配，降低碎片和运行期失败风险。
- 第一版不启用软件定时器任务，减少隐藏任务和栈预算。
- 若第三方模块暂时必须动态分配，单独记录例外，并限定为启动阶段；不能无说明地切换到 `heap_4`。
- 使用原生 FreeRTOS API，暂不增加 CMSIS-RTOS封装层。

### 8.1 SysTick与HAL时基

FreeRTOS Cortex-M端口需要接管 `SVC`、`PendSV` 和通常的 `SysTick`。当前代码使用 `HAL_GetTick()`，迁移时必须明确处理：

优先方案：

1. FreeRTOS 使用 SysTick。
2. HAL timebase 改为一个经资源审查后选定的普通/基本定时器。
3. 调度器启动前的 HAL初始化继续拥有可靠毫秒时基。

备选方案是让 `HAL_GetTick()` 在调度器运行后映射到 RTOS tick，但必须处理调度器启动前和中断上下文，复杂度更高，不作为首选。

---

## 9. 迁移阶段与验收门

每一阶段单独提交、单独测试。只有当前阶段通过验收，才进入下一阶段。

### Phase 0：修复当前控制问题并冻结裸机基线

工作项：

- [x] 修复 mode=1、目标1 RPS时不转且大电流的问题。
      → **根因 2026-09-27 定位**：编码器计数方向与相序约定相反。`identify pole` 报
      `pf=20.994 pr=21.007 dir_f=dir_r=-1`，且上电零点偏移恰为 3.1443 rad ≈ π
      （π 机械角 × 21 极对 = 21π 电角 = π 的奇数倍，正是电角度符号取反的特征），
      两个独立现象互相印证。已在硬件侧**对调电机两相引线**解决；对调后
      `identify flux` 四点 iq 仅 0.18~0.21 A 且 id≈0 → 方向/零点/相序确认正确。
- [x] 验证电流采样极性、相序、电角度方向/零位、极对数和单位。
      → 换 1 mΩ 分流 + CSA 增益 20 后重跑全部辨识。`I_SCALE = (3.3/4095)/(0.001×20)
      = 4.0293e-2 A/码`（40.3 mA/码，±82.5 A 量程）；启动时自动回读 CSACR 并校验
      `CSA_GAIN == CSA_GAIN_20`，与 `FOC_run.h` 的 `CSA_GAIN_VV=20.0f` 不一致会报 ERROR。
- [x] 验证速度反馈在手动转动和闭环运行时方向、量纲正确。
      → `identify flux` 的 we=1403/1873/2349/2813 rad/s 反算机械转速
      10.63/14.19/17.80/21.32 RPS，与打印 spd 逐点吻合 → 21 极对数与编码器标度交叉验证通过。
- [ ] 验证 Start/Stop重复操作不会残留 PI积分或异常PWM。
- [x] 测量 10 kHz ADC ISR平均、P99和最坏执行时间。
      → `g_foc_wcet_us`（FOC_run.c），RTT `diag` 可读；Phase 0 基线 63 us / 预算 100 us。
- [x] ~~特别测量 `bsp_as5600GetAngle()` 软件 I2C耗时及其关中断时间。~~
      → **该项已不适用**：编码器已换成 AS5047P 硬件 SPI（7.5 MHz，DMA 可选），
      软件 I2C 只作为 `Encoder_Adapt` 的第二种驱动保留为回退，不在实时路径上。
- [ ] 保存裸机 Flash/RAM、CAN吞吐、FOC时序和关键波形基线。
- [ ] 建立可回退标签或提交点。

验收门：

- 电机在转矩和速度模式下都能受控启动、停止。
- 零速空闲时三相占空比和相电流合理，无持续大电流。
- 10 kHz无漏周期；ISR最坏时间必须明显小于100 us，建议目标不超过周期的60%~70%。
- 软件 I2C如果无法满足时间预算，应先改为硬件外设/DMA、降低读取频率或采用角度预测。

### Phase 1：只接入FreeRTOS内核，不改变电机控制路径

工作项：

- [x] 添加固定版本的 FreeRTOS Kernel源码和许可证文件。
- [x] 加入 Cortex-M4F端口与 `FreeRTOSConfig.h`（位于 `Code_Source/CONFIG/`）。
- [x] 配置 SVC/PendSV/SysTick向量和 HAL timebase（TIM6）。
- [x] 创建一个低优先级 `BringUpTask`，仅翻转调试GPIO/递增计数器。
- [x] FOC、CAN、状态控制仍保持原路径，确认内核本身不影响控制。
- [x] 打开 `configASSERT`、栈溢出钩子和故障记录。

验收门（2026-08-18 硬件实测通过）：

- 调度器稳定运行，1 ms tick准确（RTT 心跳 `tick` 每 1s +1000）。
- 10 kHz FOC中断周期、执行时间相对 Phase 0无显著退化（电机控制正常）。
- Start/Stop和CAN行为与裸机基线一致（速度/模式/启停指令正常）。
- 中断优先级断言无触发。

bring-up 修复记录：

- **DRV83xx `Delay_us` 卡死**：HAL 时基迁到 TIM6 后 SysTick 计数器无人使能（drv83xx.c 直接读 `SysTick->VAL` 做微秒延时），在 `main_user()` 初始化处仅使能计数器（不开中断）解决；FreeRTOS 启动后接管 SysTick。
- **任务不运行（V11.3.0 Direct Routing）**：V11.3.0 端口默认 `configCHECK_HANDLER_INSTALLATION=1`，`xPortStartScheduler` 断言向量表 SVC/PendSV **直接**指向端口函数。采用 Direct Routing（FreeRTOSConfig.h 中 `#define vPortSVCHandler SVC_Handler` 等三个宏），并从 `stm32f1xx_it.c` 删除 SVC/PendSV/SysTick 空壳（见 §2.2 备注）。

回退点：移除调度器启动和 FreeRTOS构建源即可恢复 Phase 0。

### Phase 2：迁移CAN通信到 `CommTask`

工作项：

- [x] 将 CAN RX优先级调整到 RTOS API允许范围。
      → `can.c:92/94/96` 三个向量统一为 6（≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`）。
      **必须三者相同**：它们都调用非可重入的 `HAL_CAN_IRQHandler(&hcan)`。
- [x] CAN ISR保留收帧和环形缓冲写入，增加任务通知。
      → `stm32f1xx_it.c` 的 `USB_LP_CAN1_RX0_IRQHandler` → `vTaskNotifyGiveFromISR(xCommTaskHandle)` + `portYIELD_FROM_ISR`。
- [x] 将 `CANopen_OD_Process()` 从主循环移到 `CommTask`。
- [x] 一次唤醒批量处理缓冲中的全部帧。（`xTaskNotifyWait` + 内部 `while PopFrame`）
- [x] 保留 RX总数、丢帧、溢出和最大队列深度统计。
      → RX/溢出/丢帧 ✓（`CAN_bsp_GetStats`，`diag` 可读）；
      **环形缓冲"最大深度"峰值未记录** → 见 §16 待办。

验收门：

- 正常负载和突发16帧下协议解析正确。
- CAN高负载不改变 FOC周期。
- 无队列静默溢出；故障统计可从 RTT读取。

> **2026-08-19 状态**：实现已完成（CommTask 任务通知 + 批处理，can.c 优先级 6），
> 但 **CAN 硬件验收待设备到位后执行**（用户已购 创新科技 USB-CAN-2C）。
> 无 CAN 流量时 CommTask 阻塞等待不占 CPU，ISR 通知有 NULL 保护，电机仍可用 RTT 控制。

### Phase 3：迁移并强化业务状态机

工作项：

- [x] 建立 `MotorEvent` 队列和静态存储。（`motor_state_machine.c`，8 深度静态队列，`MOTOR_EVT_QUEUE_LEN`）
- [x] 建立 `MotorStateTask`，成为状态唯一写入者。（`app_tasks.c`，优先级 4，~1 ms 一拍）
- [x] 从参考项目提取状态/事件/动作表，建立目标映射和差异清单。（`eventMap` / `actionMap` 表驱动）
- [x] 重写 `fsm_state_transfer/action_perfrom` 调度内核，不移植单 pending槽和 `PT_*` 协程宏。
- [x] 先迁移 `IDLE/RUN/STOP/FAULT_ACTIVE/FAULT_RECOVERY`，再迁移 `INIT/CALIB/STANDBY`。
      （实际状态名：`S_IDLE/S_RUN/S_STOP/S_FAULT_NOW/S_FAULT_OVER/S_INIT/S_CALIB/S_STANDBY`）
- [x] 将 CAN/RTT Start、Stop、ClearFault改为发送事件。
      （`0x6040=0x1F/0x06` → `EVENT_toRUN/EVENT_RUNtoSTOP`；ClearFault 已按自动恢复删除）
- [x] 将 `StartPWM()`/`StopPWM()` 仅保留为状态机内部动作。（`PowerStage_Enable/Disable`，且 `Enable` 已加故障闸门）
- [x] 将当前 `MotorCtrl` 的影子参数提交改成一致命令快照发布。（`motor_command_snapshot.c`，`g_pending_cmd`）
- [x] 实现重复、非法、过期事件的明确处理和计数。
      （`g_stats_posted/dropped/illegal/fault_forced`，`diag` 可读；队列满时 Fault/Stop 走独立锁存不丢）
- [ ] 实现状态迁移日志，但日志输出放到低优先级任务。
      → **未达标**：迁移/非法事件日志仍在 `MotorStateTask` 内直接 `printf`
      （`motor_state_machine.c:406/413`，另有 206/233 两处）。RTT 为 `NO_BLOCK_SKIP`
      非阻塞所以危害有限，但不符合"日志放低优先级任务"的要求 → 见 §16 待办。
- [x] 为INIT/CALIB中的阻塞操作设置子步骤、超时和完成事件。
      （辨识框架 `identify_*` 各任务自带状态机 + 超时；`S_INIT` 单拍直通不阻塞）

验收门：

- 状态转换表测试全部通过。
- 快速连续 `Start/Stop/Start` 无事件覆盖和错误PWM状态。
- CAN与RTT同时发命令时状态一致、无直接并发写入。
- 队列满时 Fault/Stop仍可锁存并处理。
- 在任意状态动作执行期间注入Fault，均能立即硬件关断并最终进入 `FAULT_ACTIVE`。
- INIT失败有确定安全去向，不会无限停留且仍保持PWM关闭。

### Phase 4：迁移并完善Safety快慢保护

工作项：

- [x] 从参考项目迁移故障掩码、`SafetyFastInput_t/SafetySlowInput_t` 和必要配置结构，删除未使用平台耦合。
      （`Safety_Module.c/.h`，纯计算、不碰 HAL/RTOS/printf）
- [x] 将只读阈值配置与累计器/环形缓冲拆为 `SafetyConfig/SafetyRuntime`，补充配置合法性和NaN/Inf检查。
      （母线电压阈值用板级常量、电流阈值由 `Motor_Params.Current_Max` 推算；快慢两级都判非有限值）
- [x] 建立原子的 `active/latched/first fault` 和 `fault_pending`。
      → active/latched/pending ✓（`g_fault_bits` 原子或位 + `g_fault_pending`）；
      **first fault（首故障快照）未实现** → 见 §16 待办。
      本轮修掉一个相关缺陷：`EVENT_toFAULT` 走 `PostEvent` 时会被无条件
      `PostFault(0xFFFFFFFF)` 覆盖真实掩码，现场只剩"fault=0xFFFFFFFF"，已加
      `g_fault_bits == 0` 守卫保留真实掩码。
- [x] 将 `Safety_FastStep()` 接入10 kHz FOC ISR并纳入WCET测量。
- [x] 新增静态 `SafetyTask`，以3 ms绝对周期调用 `Safety_SlowStep()`，传入实际 `elapsed_ms`。
      （`vTaskDelayUntil` + tick 差分得到 `elapsed_ms`，非假设值）
- [x] 建立当前工程遥测/命令/状态到 `SafetySlowInput_t` 的一致快照映射。
- [x] TIM1 Break、FastStep紧急故障、驱动故障和严重软件故障执行立即PWM关断。
      → FastStep ✓（并修掉关键隐患：`__HAL_TIM_MOE_DISABLE()` 在 PWM 运行时是**空操作**，
      改用 `PowerStage_MoeOff()` 无条件清 MOE）；驱动故障 ✓（PA12 EXTI 回调，NVIC 优先级 0，
      只做"锁存故障位 + 无条件清 MOE"，不碰 SPI/printf/RTOS）；严重软件故障 ✓（任务看门狗 TWDT）。
      **TIM1 BKIN 未接入**：`tim.c:100 BreakState = TIM_BREAK_DISABLE`，硬件未把 nFAULT
      引到 BKIN，用 EXTI 替代；要硬件级 Break 关断需改板。
- [x] 实现 `FAULT_ACTIVE/FAULT_RECOVERY` 进入、保持、恢复判定和分类清除策略。
      （`state_FAULT_entry/do/exit` + `g_recovery_mask` 分类：编码器 / 栅极驱动 / CAN；
      栅极驱动类会读 FSR1/FSR2 解锁 DRV 的 `OCP_MODE=LATCH` 锁存）
- [x] 将故障CAN上报交给 `CommTask`，故障快照持久化交给 `DiagTask`，不阻塞状态机。
      > **2026-09-11 更新**：故障上报已实现为 **EMCY `0x081` + 32 位掩码**，由 `CommTask` 10ms 节拍发送（`FAULT_NOW/OVER` 期间每 10ms 重发）。
      > 同时改为**全自动恢复**：FAULT_NOW 内做定向软重初始化（编码器/CAN），完成即自动转 FAULT_OVER，`exit` 清故障字，FAULT_OVER 静止后回 IDLE；
      > **不再需要 `CLEAR_FAULT`/`clearfault`**（命令已删除）。详见 `Docs/Fault_Recovery_And_Reporting.md`。
      > 注：§6.3/§6.4 中“需明确复位流程/CLEAR_FAULT 授权”的表述随之作废（保留锁存语义在 FAULT 期间，恢复改为自动）。
      > **2026-09-27**：CAN 上报 ✓；**故障快照持久化到 `DiagTask` 仍未实现** → 见 §16 待办。
- [ ] 看门狗只在关键任务均报告健康且 FOC未超时时喂狗。
      → **部分完成**：任务健康 ✓（`Task_Watchdog.c` 巡检 5 个任务，卡死即
      "先锁存故障位→无条件关功率→上报→非关键任务延时重建"；关键任务不自动重建）；
      IWDG 运行期唯一喂狗点在 `SafetyTask` ✓，初始化期由 TIM4(5ms) 喂、启动调度器前关掉 ✓。
      **"FOC 未超时"未接入**：`g_foc_wcet_us` 目前只被 `diag` 打印，不参与喂狗决策 → 见 §16 待办。

验收门：

- 注入每一种故障时，PWM在规定硬实时延迟内关闭。
- 故障原因不会因后续故障覆盖而丢失，至少保存首故障和累计位。
- 未满足恢复条件时 ClearFault无效。
- 任务卡死、通信超时和 FOC超时能得到预期处理。
- `SafetyTask` 2~30 ms异常周期可观测，连续deadline miss能触发策略。
- FastStep加入后FOC ISR仍满足100 us预算；SlowStep压力不扰动FOC。
- 无传感器或无有效估算值的保护项不会因默认0值误触发或被错误启用。

### Phase 5：迁移RTT与诊断

工作项：

- [x] 将 `RTT_Cmd_Process()` 移入 `DiagTask`。（`app_tasks.c`，优先级 1，10 ms 节拍）
- [x] RTT命令通过状态事件或参数接口操作电机，不直接 Start/Stop。
      （`RTT_Cmd.c` 的 `post_cmd_event()` → `MotorStateMachine_PostEvent`）
- [x] 输出任务栈水位、运行时间、队列峰值、CAN统计和FOC WCET。
      → 栈水位/CPU%/WCET/CAN统计/FSM计数 ✓（`AppTasks_Diag`，RTT `diag` 命令）；
      **队列峰值（`uxQueueMessagesWaiting` 高水位）未记录** → 见 §16 待办。
- [x] 对高频遥测抽取，不在实时路径执行格式化浮点输出。
      （诊断输出改为 `diag`/`get` 命令触发，不再周期刷屏；RTT 为 `NO_BLOCK_SKIP`，printf 不阻塞）

验收门：

- 大量RTT输出不影响 FOC和CAN。
- 禁用调试输出后不改变控制逻辑。
- 所有任务栈余量满足发布要求。

### Phase 6（可选）：拆分速度/位置外环

第一版迁移稳定后再评估，不作为RTOS接入的必要条件。

候选方案A（优先）：在10 kHz ADC ISR中分频。

```c
CurrentLoop_Step();       // 10 kHz
if (++speed_div >= 10) {
    speed_div = 0;
    SpeedLoop_Step();     // 1 kHz，仍与FOC确定同步
}
```

候选方案B：1 kHz高优先级 `ControlTask` 执行外环，通过一致快照向 ISR发布 `iq_ref`。

选择标准：

- 需要严格与电流环相位同步：选A。
- 外环计算较重、允许有限抖动、需要复杂阻塞接口：才考虑B。
- 不以“代码看起来像任务”作为拆分理由。

验收门：

- 与未拆分模型的阶跃、稳态误差、超调和限流行为对比通过。
- 使用实际 `dt` 或严格固定采样时间，禁止混用。
- 模式切换和积分器复位无冲击。

### Phase 7：长期稳定性与发布收尾

工作项：

- [ ] 运行至少数小时启停、正反转、通信和负载循环测试。
- [ ] 执行 CAN突发、RTT洪泛、故障注入和欠压恢复测试。
- [ ] 记录全部任务栈高水位和 CPU占用。
- [ ] 关闭非必要动态分配、浮点日志和调试钩子。
- [ ] 更新架构图、接口说明、状态转换表和故障码文档。
- [ ] 建立正式发布标签和回退固件。

---

## 10. 测试矩阵

| 类别 | 测试 | 主要通过标准 |
|---|---|---|
| 构建 | Debug/Release完整编译 | 无警告升级、内核端口正确 |
| 启动 | 上电但不Start | PWM关闭、电流近零、任务均存活 |
| FOC时序 | GPIO/DWT测量10 kHz ISR | 无漏周期，WCET留足余量 |
| 电机控制 | 转矩/速度、正反转、空载/负载 | 行为不劣于裸机基线 |
| 状态机 | 全部合法/非法转换 | 无事件覆盖、唯一状态一致 |
| Safety快检 | 过流/短路/过压边界与注入 | ISR内锁存并立即关断，WCET合格 |
| Safety慢检 | 3 ms正常/延迟/跳周期 | 累计时间使用真实dt，deadline miss可见 |
| 通信 | CAN连续与突发流量 | 无不可解释丢帧、不扰动FOC |
| 并发 | CAN与RTT同时Start/Stop | 结果确定、无结构体撕裂 |
| 故障 | Break/过流/欠压/超温/通信超时 | 立即关断、正确锁存和恢复 |
| 资源 | 栈、RAM、CPU、队列峰值 | 栈余量>=30%，无溢出 |
| 稳定性 | 长时间循环与看门狗 | 无死锁、饥饿、复位和漂移 |

---

## 11. 代码目录建议

不要求立即移动现有文件，先通过清晰接口完成迁移。稳定后可整理为：

```text
FreeRTOS-Kernel/              # 根目录独立存放（V11.3.0 剪枝快照）
Simulink_Model/               # 根目录独立存放（模型 + generated + scripts + backups）

Code_Source/
  APP/
    app_tasks.c/.h              # 静态任务、队列创建
    motor_state_machine.c/.h    # 状态表、事件、Enter/Exit
    CANopen_OD.c/.h
    RTT_Cmd.c/.h

  SERVICE/
    MotorCtrl.c/.h              # 命令发布、状态查询
    Safety_Module.c/.h          # FastStep/SlowStep和阈值配置
    safety_task.c/.h            # 3 ms慢安全任务和输入组装
    motor_command_snapshot.c/.h
    motor_telemetry_snapshot.c/.h
    motor_fault.c/.h
    Logger_FaultLog.c/.h
    FOC_generated.c/.h

  BSP/
    CAN_bsp.c/.h
    AS5600.c/.h
    ...

  CONFIG/
    FreeRTOSConfig.h          # 配置（不放入 Core/，避免 CubeMX 再生成覆盖）

Core/
```

建议将 FreeRTOS源码作为固定版本的独立目录或 Git submodule管理；若当前工程不适合 submodule，则将版本号、来源提交和许可证一并记录在 `FreeRTOS-Kernel/README.md`。

> **目录命名备注（2026-08-18 实测）**：本工程不用 `Middlewares/` 名称——CubeMX 再生成会清理它认识的 `Middlewares/` 目录（曾导致 FreeRTOS-Kernel 源码被删）。故 `FreeRTOS-Kernel/` 独立于项目根目录。
>
> **MID_foc 清理（2026-08-18）**：旧手写 FOC 代码已最大化移除（Axis_transform/Over_Modulation/PI_Cale/Svpwm_dq/IQ_math/dead_comp + MID_foc_float 全部 + BSP/ThreeHall）。`MID_foc/` 现仅保留 Simulink 适配层依赖的 `foc.h`/`foc.c`（`PWM_PERIOD`/`PWM_HalfPerMax`/`FOC_Frequency`/`Motor_Params`/`foc_abc_current_i` 等）。FOC_run.c 的 `#else` 手写分支保留但 USE_GENERATED_FOC=1 时不编译，待后续清理。

---

## 12. 主要风险与对策

| 风险 | 表现 | 对策 |
|---|---|---|
| ISR中软件I2C过慢 | FOC超时、CAN/RTOS延迟 | Phase 0实测；硬件外设/DMA/降频/预测 |
| NVIC优先级配置错误 | FromISR断言、随机HardFault | 统一优先级分组，启用 `configASSERT` |
| SysTick与HAL冲突 | 延时错误、调度器不运行 | HAL timebase迁移到独立定时器 |
| 命令结构撕裂 | mode与目标值不一致 | 单写者 + 双缓冲/序列号快照 |
| 状态事件丢失 | Fault/Stop未处理 | 队列统计 + Fault/Stop独立锁存兜底 |
| 原样复制参考FSM | 单pending覆盖、转换中丢事件、阻塞动作 | 保留表和动作语义，重写队列化状态机内核 |
| ISR/任务同时OR故障字 | 新故障位被读改写覆盖 | 原子fetch-or，区分active/latched/first |
| Safety输入未完全初始化 | 温度/有效位随机导致误报 | 每周期零初始化并由一致快照完整映射 |
| Safety_Config并发修改 | 快检读取到半更新阈值 | 仅IDLE构造并原子发布只读配置 |
| SlowStep周期漂移 | I²t、超时和消抖时间不准 | 3 ms绝对延时 + 真实elapsed_ms + deadline统计 |
| 任务优先级过高 | 通信/日志抢占控制管理 | 最小任务集、阻塞等待、运行时间统计 |
| 栈估算错误 | 随机崩溃 | 静态栈、染色、高水位、30%余量 |
| 一次迁移范围过大 | 无法定位回归 | 分阶段提交和硬验收门 |
| 动态内存碎片 | 长时运行失败 | 运行期静态分配 |
| 误把RTOS任务当硬实时 | 100 us任务抖动 | FOC留在硬件ISR，外环优先ISR分频 |

---

## 13. 最终完成标准

只有同时满足以下条件，混合架构迁移才算完成：

- [x] 10 kHz FOC由硬件同步中断稳定运行，无漏周期。（WCET 63us < 100us）
- [x] FreeRTOS tick为1 kHz，未用于产生FOC周期。
- [x] ADC/Break等高优先级ISR不调用 FreeRTOS API。（FOC ISR 仅原子写 PostFault）
- [x] 所有普通状态转换仅由 `MotorStateTask`执行。
- [x] 参考项目状态/事件/动作语义已迁移，单pending槽和 `PT_*` 宏未进入目标实现。
- [x] `Safety_FastStep()` 在10 kHz ISR内运行并满足WCET预算。
- [x] `Safety_SlowStep()` 由 `SafetyTask` 每3 ms调用并使用真实elapsed时间。
- [x] 故障关断不依赖任务调度，Fault保持锁存。（MOE 当场关断 + fault_pending 兜底）
- [x] 故障位的ISR/任务并发置位是原子的，历史锁存不会被清除流程丢失。
      （PostFault 原子或位 + `state_FAULT_exit` 分类清除；**首故障快照仍未实现**，见 §16）
      > **2026-09-27 勘误**：本条此前写作"首故障与历史锁存不会被清除流程丢失"，但代码里
      > 只有 "当前锁存位"（`g_fault_bits`），**没有首故障快照变量**。更严重的是
      > `EVENT_toFAULT` 经 `PostEvent` 会被无条件写 `0xFFFFFFFF`，把真实掩码覆盖掉
      > （实测上电日志出现 `FSM: FAULT_NOW, fault=0xFFFFFFFF`）。掩码覆盖已修复。
- [x] CAN/RTT不再直接修改运行状态或调用 Start/Stop。（改为发事件）
- [x] ISR与任务之间使用一致快照，不存在多字段并发撕裂。（命令快照双缓冲）
- [x] 所有队列、栈和错误计数可观测。（`diag`：FSM/CAN/栈水位/WCET/CPU%/任务看门狗）
      > 队列**深度峰值**未记录（§16 待办④），其余均可观测。
- [x] 栈高水位、CPU占用、FOC WCET合格；通信压力测试待 CAN 设备到位。
- [x] 控制性能不低于 Phase 0裸机基线。（电机启停/速度正常；启动"抖"现象在 RTOS 侧排查中）
- [x] 每个迁移阶段都有独立提交和明确回退点。（Phase 0-5 共 7 次提交）

> **2026-08-27 状态**：Phase 1-5 全部完成并提交。剩余事项：
> ① CAN 硬件验收（用户已购 USB-CAN-2C，到货后验证 CommTask 协议/统计）
> ② 启动电机"抖"排查（用 `diag` CPU%/WCET + 示波器看 FOC 周期，嫌疑为软 I2C 偶发超时）
> ③ 可选：SystemView 时间线（如需更细的任务/中断时序）
>
> **2026-09-27 更新**：② 的根因已定位并解决 —— 不是软 I2C，而是**编码器计数方向与相序
> 约定相反**（`identify pole` 报 `dir=-1`，零点偏移≈π）。硬件对调两相引线后
> `identify flux` 四点 iq 仅 0.18~0.21 A、id≈0，方向/零点/相序确认正确；
> 编码器也已从软 I2C 的 AS5600 换成硬件 SPI 的 AS5047P。
> 本轮同时补齐了看门狗分层、栅极驱动故障路径、ADC/DRV 标度修正，并修掉 7 个缺陷。
> **完整状态与待办见 §16。**

---

## 14. 实施时的首个工作包

当前控制故障修复并通过 Phase 0后，第一次RTOS实施只完成以下内容：

1. 固定 FreeRTOS Kernel版本并加入工程。
2. 添加 `FreeRTOSConfig.h` 和 Cortex-M4F端口。
3. 处理 SVC/PendSV/SysTick与 HAL timebase。
4. 建立静态 `BringUpTask`。
5. 校验所有 NVIC优先级，但暂不迁移 CAN和状态机。
6. 对比 RTOS接入前后的 FOC ISR周期、WCET和电机行为。

该工作包通过后，再开始 Phase 2；不要在第一个RTOS提交中同时重构 `MotorCtrl`、CAN和状态机。

---

## 15. 参考资料

- FreeRTOS Kernel：<https://github.com/FreeRTOS/FreeRTOS-Kernel>
- FreeRTOS Cortex-M中断优先级：<https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4>
- FreeRTOS任务优先级：<https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/01-Tasks-and-co-routines/03-Task-priorities>
- VESC固件：<https://github.com/vedderb/bldc>
- VESC FOC实现：<https://github.com/vedderb/bldc/blob/master/motor/mcpwm_foc.c>

---

## 16. 实施状态汇总（2026-09-27）

### 16.1 需求完成度总表

| 阶段 | 工作项 | 验收门 | 状态 |
|---|---|---|---|
| Phase 0 裸机基线 | 8 项 | 4 项 | **部分**：方向/相序/速度量纲已验证（根因=相序反向，已硬件解决）；WCET 已测（63us）；Start/Stop 重复性、基线存档、回退标签未做 |
| Phase 1 接入内核 | 6 项 | 4 项 | **完成**（2026-08-18 硬件实测通过） |
| Phase 2 CAN→CommTask | 5 项 | 3 项 | **代码完成**；队列深度峰值未记录；CAN 硬件验收待执行（设备已到位） |
| Phase 3 状态机 | 11 项 | 6 项 | **10/11 完成**；仅"迁移日志移到低优先级任务"未达标（printf 仍在 MotorStateTask） |
| Phase 4 Safety | 10 项 | 8 项 | **7/10 完成**：首故障快照未做、FOC WCET 未接入喂狗、TIM1 BKIN 未接（用 EXTI 替代）；故障快照持久化到 DiagTask 未做 |
| Phase 5 RTT/诊断 | 4 项 | 3 项 | **完成**（队列峰值一项除外） |
| Phase 6 外环拆分 | — | — | 可选，未做（模型内已分频） |
| Phase 7 发布收尾 | 6 项 | — | 未开始 |
| §13 最终完成标准 | 13 项 | — | 12 项达成；"所有队列/栈/错误计数可观测"因缺队列峰值打折扣；"首故障不丢"已修正表述 |

### 16.2 本轮（2026-09-24 ~ 2026-09-27）改动清单

**新增文件**

| 文件 | 功能 |
|---|---|
| `Code_Source/BSP/AS5047.c/.h` | AS5047P/D 磁编码器驱动。SPI3 重映射（PC10/11/12）+ PA15 软件 NSS（硬件 NSS 在重映射后跑到 PA4 且跟随 SPE，不可用）。阻塞/DMA 双模式，**默认阻塞**便于上电验证。每帧偶校验、ERRFL、DIAAGC（MAGL/MAGH/COF/AGC）、多圈跟踪、速度 LPF、零点/方向/极对数/采样时间可配置 |
| `Code_Source/SERVICE/Encoder_Adapt.c/.h` | 编码器服务层（vtable 注册式）。`Encoder_Driver_AS5047` / `Encoder_Driver_AS5600`，换编码器只改 `ENCODER_SERVICE_DEFAULT_TYPE` 一个宏或调一次 `Encoder_Attach()`；挂不上自动回退。连续 `ENCODER_FAULT_STREAK_N=100` 拍（10ms）取不到有效样本 → `PostFault(FAULT_ENCODER_MASK)`，之后每 4096 拍重报 |
| `Code_Source/SERVICE/Task_Watchdog.c/.h` | 任务看门狗（TWDT）+ RCC 复位原因诊断。5 任务策略表（BringUp 1000ms / Diag 100ms / Comm 100ms / MotorState 50ms / Safety 不监视）、10ms 巡检、卡死动作链："先锁存故障位 → `PowerStage_MoeOff()` 无条件关断 → `PowerStage_Disable()` → 上报 → 非关键任务两阶段重建（`vTaskDelete` + 延时 20ms 等 idle 回收 TCB 再 `xTaskCreateStatic`）/ 关键任务停监视等人工" |
| `Code_Source/CONFIG/Board_Config.h` | 板级常量。`WWDG_FEED_INTERVAL_100us=50`（TIM4 tick 100us → 5ms 喂狗间隔）、IWDG 超时 40.8ms 的完整推导 |

**修改文件**

| 文件 | 功能 |
|---|---|
| `Core/Inc/main.h` | `USER CODE Includes` 引入 `Board_Config.h`，补上 `WWDG_FEED_INTERVAL_100us` 缺失（此前 `tim.c:130` 编译不过） |
| `Core/Src/main.c` | `HAL_TIM_PeriodElapsedCallback` 增加 TIM4 分支喂 IWDG —— 调度器启动前唯一的喂狗点 |
| `Core/Src/tim.c` + `.ioc` | 新增 TIM4：PSC 12000-1（120MHz→100us/tick）、ARR = `wwdg_feed_interval_100us-1`（49→5ms）、NVIC 优先级 4 |
| `Core/Src/iwdg.c` + `.ioc` | IWDG PSC32 / Reload50 → 超时 40.8ms |
| `Core/Src/adc.c` + `.ioc` | 注入通道采样时间 7.5 → 13.5 周期（ADCCLK 15MHz → 0.9us） |
| `Core/Src/can.c` + `.ioc` | 三个 CAN 向量优先级统一为 6（都调用非可重入的 `HAL_CAN_IRQHandler`，必须同优先级） |
| `Code_Source/APP/main_user.c` | 编码器服务层接入（`Encoder_ServiceInit/Configure/LogStartupInfo`）；上电强拖零点标定改用服务层 API；**辨识参数写回**；`DRV83XX_Init()` 返回值检查 + `DRV83XX_PrintConfig()` + CSA 增益一致性检查；TIM4 启动/交棒；复位原因打印 |
| `Code_Source/APP/app_tasks.c` | 5 个任务各加 `Twdt_Feed()`；`AppTasks_Init` 加 `Twdt_Init()` + 5 次 `Twdt_Register()`；SafetyTask 加 `Twdt_Check()` + **运行期唯一 IWDG 喂狗点**；`diag` 增加看门狗段（复位原因 + 各任务 feeds/stalls/rebuilt/monitored/CRIT） |
| `Code_Source/APP/FOC_run.c` | `Encoder_Update()` 替换 `bsp_as5600GetAngle()`；10kHz 快检路径改用 `PowerStage_MoeOff()`（原来调的是空操作宏） |
| `Code_Source/APP/motor_state_machine.c/.h` | `state_IDLE_do()` 兜底 S_INIT 期间被 eventMap 丢弃的故障位；`PostEvent` 加掩码守卫；`Init` 保留预锁存故障；`state_FAULT_do` 增加栅极驱动类恢复；新增 `MotorStateMachine_GetFaultBits()` |
| `Code_Source/APP/RTT_Cmd.c` | `get` 增加母线电压/量程/OV·UV 阈值/编码器自检（err/miss/streak/posts）/看门狗自检（复位原因+卡死+重建计数） |
| `Code_Source/SERVICE/MotorCtrl.c/.h` | `PowerStage_Enable()` 增加故障闸门；新增 `HAL_GPIO_EXTI_Callback()`（PA12 nFAULT，优先级 0，只做"锁存 + 无条件清 MOE"）；新增 `PowerStage_MoeOff()`；限幅常量对齐 `Motor_Params` |
| `Code_Source/SERVICE/Safety_Module.c/.h` | 板级过压常量（40V 硬闸 / 33V 软积分起点 / 9V 欠压）；电流阈值改由 `Current_Max` 推算（×1.2 过流 / ×1.5 短路）；新增 `FAULT_GATE_DRIVER_MASK`、`FAULT_TASK_WATCHDOG_MASK` |
| `Code_Source/BSP/drv83xx.c/.h` | `CSA_GAIN_5 → CSA_GAIN_20`；`SEN_LVL 0.25V → 1.0V`；`VDS_LVL 0.94V → 0.45V`；`OCP_MODE REPORT → LATCH`；`DRV83XX_Init` 改为返回回读校验结果；新增 `DRV83XX_PrintConfig()`、`DRV83XX_ReadAndClearFaults()`（读 FSR1/FSR2 解锁 DRV 锁存 + 逐位解码打印 + 返回 nFAULT 状态） |
| `Code_Source/APP/FOC_run.h` | `SHUNT_RESISTENCE 0.1f → 0.001f`；新增 `CSA_GAIN_VV=20.0f`；`I_SCALE` 改为公式推导（含量程/码值/功耗推导注释） |
| `Code_Source/CONFIG/FreeRTOSConfig.h` | `INCLUDE_vTaskDelete 0 → 1`（TWDT 重建卡死任务需要） |
| `Simulink_Model/CyberDog_Motor_FOC.slx` + `generated/` | 新增根级 `AccelEst` 子系统（一阶 LPF，fc=100Hz）+ 根输出 `accel_meas_rps2`，加速度估计只走模型这一条路 |

### 16.3 本轮修掉的缺陷

| # | 缺陷 | 影响 | 修复 |
|---|---|---|---|
| 1 | `__HAL_TIM_MOE_DISABLE()` 在 PWM 运行时是**空操作**（F1 HAL 宏带 `CCER` 通道使能条件判断） | FOC 10kHz 快检的"当场快速关 PWM"从未生效；TWDT 与 EXTI 的第一道关断也失效 | 新增 `PowerStage_MoeOff()` 用 `__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY()`，三处全部替换 |
| 2 | `EVENT_toFAULT` 经 `PostEvent` 被无条件 `PostFault(0xFFFFFFFF)` | 真实故障掩码被覆盖，现场只剩 `fault=0xFFFFFFFF`，且 `g_recovery_mask` 变成三类恢复全跑 | `PostEvent` 加 `g_fault_bits == 0` 守卫 |
| 3 | `MotorStateMachine_Init()` 清 `g_fault_pending` 但不清 `g_fault_bits` | 调度器启动前 FOC ISR 锁存的故障"有值没人管" | `g_fault_pending = (g_fault_bits != 0)` |
| 4 | S_INIT 期间锁存的故障被 eventMap 当非法事件丢弃 | 上电瞬间的故障静默丢失 | `state_IDLE_do()` 进 IDLE 时补检 `g_fault_bits` 并转 toFAULT |
| 5 | AS5600 初始化失败导致整个系统无角度（`as5600_init` 任一失败即返回 -1） | 编码器挂不上时系统完全不可用 | 改为用 `bsp_as5600GetAngle()` 探针回退 |
| 6 | `WWDG_FEED_INTERVAL_100us` 全工程未定义 | `Core/Src/tim.c` 编译失败 | 新建 `Board_Config.h` 定义 |
| 7 | `drv83xx.c` `#define DEBUG` 与 CMake 的 `-DDEBUG` 冲突 | 每次编译一条 redefined 告警 | 加 `#ifndef` 守卫 |
| 8 | **上电假 nFAULT 故障**：CubeMX 在 `MX_GPIO_Init()`（`main()` 的第一个调用）里就配好 PA12 的 EXTI 并**立刻** `HAL_NVIC_EnableIRQ`，而那一刻 DRV 还没供电、引脚电平未定义 → 捕到一个假下降沿 → 上电必现一次 `fault=0x00000008`。证据：那次故障的 `FSR1/FSR2=0x0000` 且 `nFAULT(PA12)=1` —— **器件根本没报故障** | 上电凭空走一遍 FAULT→恢复流程，干扰诊断，且会掩盖真故障 | ① `HAL_GPIO_EXTI_Callback` 增加"**当前电平复核**"，丢弃已恢复的毛刺；② 加 `g_drv_ready` 门控：`DRV83XX_Init()` + CSACR 一致性校验通过（`MotorCtrl_DrvReady()`）之前一律忽略 nFAULT —— DRV 未配置、PWM 未开时它没有任何保护意义；③ `real/spurious/last_tick` 三个计数进 RTT `diag` |
| 9 | **AS5600 软件 I2C 与霍尔引脚冲突**：PA5/PC4 被 6 路线性霍尔的 ADC 输入占用后，CubeMX 再生成时 `IIC_SDA_Pin/IIC_SCL_Pin` 宏消失 → `Soft_IIC.c` 报 12 个 `undeclared` 错误 | **构建直接失败** | 三处一起改：`Encoder_Adapt.h` 新增 `ENCODER_SERVICE_ENABLE_AS5600=0` 并把 `ENCODER_SERVICE_ENABLE_FALLBACK` 置 0；AS5600 适配器整段条件编译（含头文件）；`CMakeLists.txt` 排除 `MID_SoftIIC/` 与 `BSP/AS5600.c`（`text` −3.5 KB）。**恢复时要两边一起改** |
| 10 | 停 PWM 可能影响计数器，但不是无条件停止 | HAL 条件宏仅在全部 CCxE/CCxNE 清零时清 CEN；CC4E 也在掩码内 | 当前保存/恢复 CR1；不能认定这是只进一次中断的已证实根因。需分别检查 CEN、CC4E、JEOCIE |
| 11 | 触发源由 CC4 改为 OC4REF/TRGO | 当前 ADC2/3 使用 TIM1_TRGO，需保持计数器运行 | 保留该分配；撤回“所有 CC 事件必受 MOE 门控”和“触发相位完全不变”的泛化结论。PWM1 的 OC4REF 上升沿在下计数穿过 CCR4，需实测采样窗口 |
| 12 | 本地 HAL 在特定配置下清 JEOCIE | JAUTO=0、规则组软件触发、ContinuousConvMode=DISABLE 时，第一次回调前关闭注入中断使能；不等于硬件不支持注入抢占 | **已修复（2026-09-27）**：① FOC ISR 回调**每拍**重新 `__HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC)`，不再依赖"某个 ADC 必须配成 ContinuousConvMode=ENABLE"的隐式耦合（HAL 是在调用回调**之前**关的，所以在回调里重开最直接）；② `adc.c`/`.ioc` 里 ADC3 仍配 `ContinuousConvMode=ENABLE` 做双保险。反汇编已确认 ISR 入口有 `orr #128`（JEOCIE=bit7）。上板验收见 §18.7.5 |
| 13 | **电流零偏校准没有完成判据**：`Offset_Current_Start()` 发完启动就顺序执行，若采样链没起来（ADC 没触发/EPADC 没配好），它照样用一份"全 0 累加器"算出 `offset=0`，然后带着错误的零偏去做强拖和编码器零位校准 | 零偏错了电流环直接有静差/偏置，而且**不会报任何故障**，现场表现为"电机能动但电流波形是歪的"，极难定位 | `Offset_Current_Start()` 改为 `int`：① `g_offset_cal_done` 由 ISR 在**算完均值之后**置位（不是采满就置，避免读到半成品）；② 启动后以 `OFFSET_CAL_SAMPLES=1000` 拍 / `OFFSET_CAL_TIMEOUT_MS=500` 做**有界等待**（调度器前，`HAL_GetTick` 由 TIM6 提供，可用）；③ 超时则退回中点 `ADC_MID_SCALE=2048`、`MotorStateMachine_PostFault(FAULT_CURRENT_SENSE_MASK)`（`1u<<5`）、打印 `[CAL] ERROR`、返回 −1；④ 调用侧 `if (Offset_Current_Start() != 0)` 直接**跳过强拖与编码器零位校准**并 `PowerStage_Disable()`。新增 `g_offset_cal_count` 记实际拍数，RTT `get` 的 `cur:` 行可查 `cal_done / n / offset ib / ic` |
| 14 | **AS5047 阻塞读的 SPI 超时用 `HAL_GetTick()` 计时**，但该路径会被 FOC ISR（优先级 1）抢占，而 TIM6 中断优先级 15 **无法抢占 FOC ISR** → 一旦 FOC ISR 长时间占用（或 ISR 自身调用该路径），`HAL_GetTick()` 停止推进，超时判据永不成立 → **死等**，表现为电机 10kHz 中断被 SPI 拖死 | 单点故障直接升级成整机卡死，且 IWDG 只能兜底复位、定位不到原因 | 改成**寄存器级轮询 + DWT 周期计时**，彻底不依赖任何中断：直接写 `SPI3->DR`、轮询 `SPI_SR_TXE`/`SPI_SR_RXNE`、读回 `SPI3->DR`（`uxth` 取 16 位），超时用 `DWT->CYCCNT` 与 `AS5047_TIMEOUT_CYCLES = (SystemCoreClock/1000U) * AS5047_SPI_TIMEOUT_MS`（10ms）比较；DMA 等待路径同样改为 DWT 截止时刻。反汇编已确认：无任何 `HAL_SPI_TransmitReceive` 调用点，字面量池里 `0xe0001000`+`[+4]` 即 `DWT->CYCCNT`、`SystemCoreClock`(0x20000000) ÷1000 用魔数 `0x10624dd3` 实现后 ×10 |
| 15 | FOC ISR 的 WCET 换算把 HCLK 写死成 `/120U` | 与 AS5047 驱动用 `SystemCoreClock` 推导超时的口径不一致；改主频（或换回 72MHz 的 STM32F103）时 `g_foc_isr_us`/`g_foc_wcet_us` 会**静默失真**（数字看着正常，实际全错），而 §17 的优化决策正是建立在这两个数上 | 两处改为 `/ (SystemCoreClock / 1000000U)`（`SystemCoreClock=120MHz` 时结果不变，反汇编里从立即数变成 2 条 `UDIV`，各自 ~2–12 周期，对 12000 周期预算可忽略） |
| 16 | **DWT 周期计数器没有任何"明确"的使能点**：`main_user.c` 里那句 `DWT_Init()` 是**被注释掉的**，全系统实际是靠 `SEGGER_RTT_PLUS_Port_Init()`（即"J-Scope 调试变量曲线"那句）顺手使能的 | 这是一整类"坏了不报错、只是数字静默变 0"的隐患，牵连四处：① AS5047 阻塞读的 SPI 安全超时——一旦 DWT 不走，`elapsed` 恒为 0，超时判据**永不成立**，退回死等（即缺陷 14 白修）；② FOC ISR 的 `g_foc_isr_us`/`g_foc_wcet_us` 恒为 0，§17 会拿 0 去做优化决策；③ FreeRTOS 运行时间统计全 0；④ SystemView/RTT 时间戳全 0。触发条件却只是"谁把一句调试初始化删掉"——做生产固件时几乎必然发生 | 三层一起加固：① `main_user.c` 恢复 `DWT_Init()`，提前到 J-Scope 之前，去掉"安全超时依赖调试工具"这层耦合；② AS5047 驱动自带 `as5047_dwt_ensure()`（幂等）：自己置 `DEMCR.TRCENA`+`DWT_CTRL.CYCCNTENA`，并**实测**计数器是否真的在走（读两次 + 64 个 NOP 比较）；③ 若实测不走，超时退化为"最大轮询次数"`AS5047_TIMEOUT_POLLS = AS5047_TIMEOUT_CYCLES/8`（精度没了但**保证不死等**——"不死等"正是该超时存在的唯一意义）。状态经 `as5047_dwt_state` 暴露，RTT `get` 新增 `enc-timebase:` 行（1=正常，2=兜底，正常必须恒为 1）。反汇编已确认 `main_user→DWT_Init`、`bsp_as5047HwInit→as5047_dwt_ensure`、`as5047_timed_out` 双路径分支 |

| 17 | **（自查回归，2026-09-27 当日发现并修复）把 AS5047 阻塞读改成寄存器级轮询时漏了使能 SPI 外设（SPE）**：`HAL_SPI_TransmitReceive()` 在收发前有 `if ((CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) __HAL_SPI_ENABLE(hspi);`（`stm32f1xx_hal_spi.c:1244`），而 **`HAL_SPI_Init()` 不会留下 SPE=1** —— 该文件里 9 处 `__HAL_SPI_ENABLE` 全在传输函数内部，`HAL_SPI_Init`（335~455 行）一处都没有 | **编码器从那一刻起彻底读不到**，而现场症状是一串看似无关的 FSM 刷屏：SPE=0 时写 DR 不产生任何时钟，而 TXE 复位后本来就是 1（第一层循环直接通过），RXNE 永远不来 → 每帧 10ms 超时 → streak 到 10 首报，之后每 `ENCODER_FAULT_REPOST_CYCLES=4096` 拍重报一次（10kHz 下 ≈2.4 次/秒）→ 每次重报都走完整一圈 `S_IDLE→S_FAULT_NOW→S_FAULT_OVER→S_IDLE` 并打印 4 行，看起来像"无限循环"。**根因在编码器，状态机只是放大镜** | ① `as5047_transfer_polling()` 每帧传输前镜像 HAL 做一次 SPE 检查+使能（**必须每帧做**：`HAL_SPI_Abort()`/`HAL_SPI_DMAStop()` 会 `__HAL_SPI_DISABLE()`）；② `bsp_as5047SpiInit()` 末尾显式 `__HAL_SPI_ENABLE(&hspi3)`，让上电状态确定。③ 顺带修掉同一次改动引入的另一个隐患：超时分支**提前 return 不读 DR**，在途/滞留字节会被下一帧当成有效数据（角度静默错一帧）；现在超时统一走新增的 `as5047_spi_resync()`（清 SPE 再置 SPE，复位 SPI 状态机与标志）。反汇编已确认 SPE 检查与 resync 均在位 |
| 18 | **持续性故障下状态机刷屏**：`Encoder_Adapt.c` 的周期重报（设计如此，注释写明"不会安静地停在 IDLE"）叠加 `state_FAULT_over_do()` 的 `speed<0.1` 立即退出（静止时恒成立） | 一个持续存在的故障会产生 ~10 行/秒的 `FSM:` 日志，把真正的首因（编码器错误计数）埋在噪声里；且 `PowerStage_Disable()` 与 `Param_init+Param_deinit+FOC_Generated_Reset()` 会以 2.4 次/秒无意义地反复执行。**注意这是诊断可读性问题，不是安全问题**（重报机制本身是有意设计） | 待定，见 §16.5 ⑱：候选方案是给日志降频（状态机行为不变）、或在 `FSM: FAULT_NOW` 行附带首因提示（如编码器 `err/streak`）、或对持续性故障引入"停驻态"直到病因消失 |

### 16.4 电机参数基线（2026-09-27 台架实测，已写入 `main_user.c`）


| 参数 | 值 | 来源 |
|---|---|---|
| `Pole_Pairs` | 21 | `identify pole`：pf=20.994 / pr=21.007 |
| `Phase_R` | 0.2631 Ω | `identify r` |
| `Ld` / `Lq` | 195 uH / 393 uH | `identify l`（Lq/Ld = 2.02） |
| `Flux` | 0.003694 Wb | `identify flux`（4 点，残差 ±0.015V <0.15%，R²>0.9999） |
| `Kt` / `Ke` | 0.116345 / 0.077564 | 由 Flux 推算，与辨识一致 |
| `friction_B_nm_rps` / `friction_Coulomb_nm` | 0.000347 / 0.017025 | `identify flux`（自检：0.0074+0.0170 ≈ 0.0244 N·m vs iq·Kt = 0.0249 N·m） |
| `Rotor_inertia` | **0（未辨识）** | **待跑 `identify j`** |
| `Current_Rating` / `Current_Max` | 5 A / 40 A | 台架保守设定 |
| 采样链 | 1 mΩ + CSA 增益 20 | `I_SCALE = 4.0293e-2 A/码`（±82.5 A 量程） |

> **母线电压上限提醒**：SVPWM 线性区最大相电压峰值 = Vbus/√3 = 13.86 V @24V，
> 而 1800 RPM(=30 RPS) 反电动势峰值 = 0.077564×2π×30 = **14.62 V > 13.86 V**
> → 24 V 下无载最高约 **1706 RPM**，`MOTOR_SPEED_LIMIT_RPS=30` 不可达。
> 要跑满 1800 RPM 需把母线抬到 ~27 V 以上（板级 OV 软闸 33V / 硬闸 40V 有余量）。

### 16.5 待办清单

**P1 — 文档明确要求但尚未实现**

| # | 待办 | 对应条目 |
|---|---|---|
| ① | 首故障快照（first fault + 累计位），`state_FAULT_entry` 记录首个故障掩码与时间戳 | Phase 4 工作项 3、验收门 2 |
| ② | 喂狗条件加入 **FOC WCET 超时**判定（`g_foc_wcet_us` 目前只被 `diag` 打印） | Phase 4 工作项 10 |
| ③ | 故障快照持久化交给 `DiagTask` | Phase 4 工作项 9 |
| ④ | 队列深度峰值统计（`uxQueueMessagesWaiting` 高水位） | Phase 2/5 工作项 |
| ⑤ | 状态迁移日志移到低优先级任务（现仍在 `MotorStateTask` 内 `printf`） | Phase 3 工作项 10 |

**P2 — 硬件/结构性问题**

| # | 待办 | 说明 |
|---|---|---|
| ⑥ | TIM1 BKIN 未接入 | `tim.c:100 BreakState = TIM_BREAK_DISABLE`，硬件未把 nFAULT 引到 BKIN；当前用 PA12 EXTI 替代。要硬件级 Break 关断需改板 |
| ⑦ | 活路径无速度/电流限幅 | `MotorCtrl_Start/Stop/Process` 是**死代码**（零调用点，已被 `--gc-sections` 回收），其中的 `clampf` 从未执行；真正生效的提交点是 `state_RUN_entry()`，那里直接拷贝 `g_pending_cmd`。建议把限幅搬到 `state_RUN_entry` |
| ⑧ | 24V 母线电压上限（见 16.4 注） | 1800 RPM 不可达 |
| ⑨ | ADCCLK = 15 MHz 超 F1 的 14 MHz 上限 7% | PCLK2=120MHz，`ADCPresc` 仅 /2/4/6/8，/8 已是最低档，无法再降 |

**P3 — 验证与收尾**

| # | 待办 |
|---|---|
| ⑩ | ~~`identify j` → 填入 `Rotor_inertia`~~ **已完成**：`identify r → l → pole → flux → j` 全部跑完，已写入 `main_user.c`（J=7.92e-5，err=0.175） |
| ⑪ | CAN 硬件验收（USB-CAN-2C 已到位）：协议解析、突发 16 帧、统计 |
| ⑫ | Phase 0 基线存档（Flash/RAM/CAN 吞吐/FOC 波形）+ 建立回退标签 |
| ⑬ | Phase 7：长时间启停/正反转/故障注入/欠压恢复测试，栈高水位与 CPU 占用记录 |
| ⑭ | **DMA 路径的预测补帧**：当前只在阻塞读取路径实现（见 16.6），切到 `AS5047_READ_MODE_DMA` 时需要在 `HAL_SPI_ErrorCallback` 的错误分支补同样的外推 |
| ⑮ | **电流零偏校准上板验收**（缺陷 13）：确认 `get` 的 `cur:` 行 `cal_done=1 n=1000`、`ib/ic` 均在 2048±60；再注入失败（临时改小 `OFFSET_CAL_TIMEOUT_MS` 或掐 ADC3 触发）验证走 `[CAL] ERROR` + `FAULT_CURRENT_SENSE_MASK` + **不开功率** |
| ⑯ | **SPI 阻塞读超时上板验收**（缺陷 14/16）：拔掉 AS5047 供电/短接 MISO，确认报 `ERR_TIMEOUT` 并在 `ENCODER_FAULT_STREAK_N=10` 拍后上 `FAULT_ENCODER_MASK`，而**不是** FOC 中断卡死（同时看 `diag` 的 `FOC ISR count` 是否仍在推进）；另确认 `get` 的 `enc-timebase: dwt=1` |
| ⑰ | 输出端 6 路霍尔：`bsp_hall6` 驱动 + `Encoder_Driver_Hall6` 实例化（§18.3/§18.4），极对数 P_out 用 `identify hallpole` 最小二乘确认（§18.5） |
| ⑱ | **持续性故障的日志可读性**（缺陷 18，需先定方案）：候选 ① 只给 `FSM:` 打印降频/去重（状态机行为不变，最保守）；② 在 `FSM: FAULT_NOW, fault=0x...` 行附带首因，如 `enc err=1234 streak=9999`，让"编码器挂了"一眼可见；③ 引入"停驻态"直到病因消失（改动最大，涉及安全语义，需评审）。**建议先做 ②**：本次排查里"故障字只给出掩码、不给首因"是真正浪费时间的地方 |

### 16.6 2026-09-27 追加：编码器速度预测补帧（超出原计划的新增设计）

**动机**：读失败时若把角度冻结，模型内的 `EncoderPLL` 会看到"输入冻结"。
PLL 是**相位跟踪器**而不是速度外推器 —— 输入恒定时它先按估计速度冲过去、
再被冻结值拉住（θ̂→冻结值、ω̂→0），也就是丢帧瞬间速度估计先**过冲**再**塌陷**，
闭环会抖一下。与其让 PLL 兜底，不如驱动层直接给出连续角度。

**实现（`Code_Source/BSP/AS5047.c/.h`）**

- 把 `as5047_track()` 的簿记部分抽成 `as5047_publish()`：真实样本与预测帧共用同一套
  `angle_delta` / 多圈计数 / 速度滤波 / `sample_seq` 逻辑，避免两条路径行为漂移。
- 新增 `as5047_predict()`：`θ_pred = θ_last + ω_filtered × Ts`，同步推进
  `mech_angle` / `angle` / `eleangle`，并走 `as5047_publish()`。
  - `speed_rad_s` 已含 `direction`，先还原成"原始编码器方向"再外推
    （`dir × direction ≡ 1`，对 ±1 都成立）。
  - `angle_last` 会前进到预测值 → 真实样本回来时 `delta = 实测 − 预测`：
    预测超前则 delta 偏小、速度滤波被自动拉回，**预测误差不会累积**。
  - 预测期间 `raw_speed` 恰好等于外推用的速度，所以速度滤波在预测帧上自洽不变。
- 上限 `AS5047_PREDICT_MAX_FRAMES = 10`，与 `ENCODER_FAULT_STREAK_N` **对齐**：
  预测到"故障上报那一刻"就停，超限返回 `AS5047_ERR_NO_DATA`、角度冻结。
- 新返回码：`AS5047_PREDICTED = 1`（正值，与负错误码区分）、`AS5047_ERR_NO_DATA = -7`。
- 开关 `AS5047_ENABLE_PREDICTION`（默认 1）。
- **未覆盖的路径**：DMA 流水线。`bsp_as5047AsyncFetch()` 无新样本时返回
  `AS5047_ERR_BUSY`（上层按 REPEAT 处理），硬错误在 `HAL_SPI_ErrorCallback` 里
  计入 `async_error_streak`。切 DMA 时需在错误回调处补同样的外推 → 见 16.5 ⑭。

**适配层配合（`Code_Source/SERVICE/Encoder_Adapt.c/.h`）**

- 新增 `ENC_SAMPLE_PREDICTED`。预测帧**照常更新 `mec` / `ele`**（这正是机制目的：
  保证喂给模型 PLL 的输入连续），但**仍然计入 `fault_streak`** ——
  否则编码器彻底断线会被预测永远掩盖、永不报故障。
- `ENCODER_FAULT_STREAK_N: 100 → 10`（10 帧 = 1 ms @10 kHz），与预测窗口一致。
- **修掉一个"装了但不生效"的问题**：`bsp_as5047SetSampleTime()` **全工程零调用点**，
  驱动默认 `sample_time_s = 0` → `speed_rad_s` 恒为 0 → `as5047_predict()` 直接返回
  `AS5047_ERR_PARAM`，预测永远不生效。现在在 `as5047_init()` 里设
  `Ts = 1/FOC_Frequency`。
- 速度 LPF 从驱动默认 `1.0`（完全不滤波）改为 `0.1`：16384 cpr、Ts=100 µs 时
  单拍速度量化台阶 = (2π/16384)/1e-4 = **3.83 rad/s**，不滤波的"上一拍速度"
  拿去外推会把这个量级的噪声带进角度。τ ≈ 10 拍 = 1 ms，刚好覆盖预测窗口。
  注：`speed_rad_s` 目前只有预测补帧在用（模型内转速来自它自己的 `EncoderPLL`），
  所以改这个系数不影响控制环。

**观测**：RTT `get` 新增一行
`enc-pred: pred=<累计补帧> now=<当前连续补帧> max=<历史最长> (window=10 frames)`。

**验证**（反汇编级）
- `as5047_init` 实际下发 `Ts=1.0e-4`（`0x38d1b717`）与 `speed_lpf=0.1`（`0x3dcccccd`）✓
- `encoder_note_sample_bad` 门限为 `cmp r3, #10`，上报 `PostFault(0x4000)` = `FAULT_ENCODER_MASK` ✓
- `main_user` 字面量池含 `2d18a638` = `0.0000792f`（辨识 J 已写入）✓

---

## 17. FOC 中断时间优化计划（2026-09-27 立项）

> 2026-09-28 更新：下方保留历史方案讨论；当前默认已改为单帧阻塞流水线，DMA 使用专用寄存器传输与有界完成处理，详见 [AS5047 单帧读取实现](AS5047_Single_Frame_Readout.md)。原两帧默认、HAL DMA 回调和固定延迟描述不再代表当前实现。

### 17.1 现状与疑点

`g_foc_wcet_us` 实测 **60+ µs**（预算 100 µs，Phase 0 基线 63 µs）。
但 AS5047 阻塞读取两帧 16bit @7.5MHz 理论只有 4.3 µs，所以 60 µs 里绝大部分不是编码器。
已定位的可疑项（按预期收益排序）：

| # | 疑点 | 证据 | 预期收益 |
|---|---|---|---|
| A | **编译优化等级只有 `-Og`** | `build/Debug/build.ninja` 的 `FLAGS` 实测为 `... -Og -g3 ...`，全工程无 `-O2/-O3`。模型是纯 float DSP 代码，`-Og`（≈`-O1`）相对 `-O2` 通常慢 1.5~2.5 倍 | **最大**，预计 60→25~35 µs |
| B | **生成模型里有 2×`sqrtf` + 3×`fmodf` + 1×`floorf`，每拍都跑** | `CyberDog_Motor_FOC.c:987`（LESO `sqrtf`）、`:1376`（`VmaxCoeff/fmaxf(sqrtf(...))`）、以及 ESO/SVPWM 的角度回绕 `fmodf`。`fmodf` 是**软件例程**（无 FPU 指令，~100+ 周期）；`sqrtf` 未加 `-fno-math-errno` 时也要走库例程而不是单条 `vsqrt.f32` | 中，合计数 µs |
| C | **编码器走阻塞 HAL SPI，每拍两帧** | `AS5047_READ_MODE_BLOCKING`（当前默认）。两帧 = 4.3 µs 总线时间，但 `HAL_SPI_TransmitReceive` 在 `-Og` 下的状态机开销可能数倍于此 | 中，且驱动已支持 DMA 模式 |
| D | **HAL 中断分发链** | `ADC1_2_IRQHandler` → `HAL_ADC_IRQHandler` → `HAL_ADCEx_InjectedConvCpltCallback`，`-Og` 下每层都有寄存器搬运 | 小~中 |
| E | 外环（速度/位置/LESO）也在 10 kHz 里跑 | 模型内未分频 | 中（对应 Phase 6 候选 A） |

### 17.2 工作项（必须先测量，再优化）

- [x] **17.2.1 分段计时**（前置条件，不做这一步后面全是猜）
      在 FOC ISR 内用 DWT->CYCCNT 分别测：`Encoder_Update()` / `Safety_FastStep()` /
      `FOC_Generated_Step()` / 占空比写出，各自记 max 到 `g_foc_t_enc_us` 等全局量，
      由 RTT `diag` 打印。只有拿到占比才能判断 A~E 哪个是真凶。
      **第一段已实现（2026-09-27）**：`Encoder_Update()` 用 DWT 括起来，写入
      `g_enc_read_us` / `g_enc_read_wcet_us`（`FOC_run.c`，名字未用 `g_foc_t_enc_us`
      以免与既有 `g_foc_isr_us`/`g_foc_wcet_us` 风格冲突），`diag` 新增
      `ENC read: <us> (max <us>)` 行。其余三段（Safety_FastStep / FOC_Generated_Step /
      占空比写出）**仍未测**，补齐后才有完整占比。
      预期值（据实测反汇编，非板上实测）：AS5047 阻塞读 = 2 帧 × 16bit @7.5MHz = 4.27 µs
      + 4 次 `t_CSn`/`t_L` 间隔 × 450 ns ≈ 1.8 µs + 4 次 `HAL_GPIO_WritePin` ≈ 0.6 µs
      **≈ 6.7 µs / 100 µs**。若 `diag` 读数明显大于此，先查 `as5047_cs_gap` 与 SPI 分频。
- [ ] **17.2.2 新增 `-O2` 构建档并实测**
      保留现有 `Debug`（`-Og -g3`，源码级调试用），另加 `RelWithDebInfo`（`-O2 -g3`，
      保留调试信息但优化）。用同一个测试工况对比 WCET。
      **这是收益最大且风险最低的一项**；若 `-O2` 后仍超标再动算法。
- [ ] **17.2.3 数学库开销**
      `-fno-math-errno`（让 `sqrtf` 退化成单条 `vsqrt.f32`）；
      把角度回绕的 `fmodf` 换成"条件加减 2π"（角度范围已知，用 `fmodf` 是浪费）。
      注意**不要**上 `-ffast-math`（会改变 NaN/Inf 语义，而 Safety 模块依赖
      `isfinite()` 判故障）。
- [x] **17.2.4 编码器读取开销优化 —— 结论是"不要改协议，改开销"**
      原计划"切 `AS5047_READ_MODE_DMA` 单帧流水线"**经查手册后否决为默认方案**，
      依据与替代方案见下（这也顺带回答了"能不能照 HAL 用 DMA"这个问题）。

      **一、2 帧/样本是器件固有的，改不掉。**
      AS5047P 手册 v2-00 p13 图 15（SPI Read）明确画出：CSn 是**每 16bit 帧一个窄高脉冲**，
      命令帧与数据帧各自一个 CSn 低窗口；且正文写明 *"The SPI read is sampled on the rising
      edge of CSn and the data is transmitted on MISO with the next read command"* ——
      即第 N 帧发命令、第 N+1 帧才回数据。所以：
      ① 不能把命令+数据合并成一个 32bit 的 CS 低窗口（图中 CSn 必须拉高）；
      ② 无法"2 帧降到 1 帧"，最后一次命令在流内永远拿不到应答，必须补一个 NOP 帧。
      驱动现在的 `命令帧 + NOP 帧` 就是最小形式。

      **二、SPI 时钟不能提高。** 手册 `tMISO = 51 ns`（CLK 沿到数据有效）。模式 1 下
      数据在第一个沿变化、第二个沿采样，可用时间只有半周期：7.5 MHz → 66.7 ns（余量 15.7 ns，合规）；
      15 MHz → 33.3 ns **< 51 ns，违规**。所以 `/8` 已是上限，**不要再试图提到 `/4`**。

      **三、流水线（DMA 单帧）的代价是 2 个 FOC 周期，不只是 1 个。**
      器件 1 帧流水延迟 + 消费端 1 拍 ⇒ 第 k 拍消费到的角度是第 k−2 拍锁存的。
      1800 RPM、21 对极下电频率 630 Hz、电周期 1587 µs，于是：
      | 方案 | 延迟 | 折合电角度 @1800RPM |
      |---|---|---|
      | 阻塞读（当前 A） | 4.27 µs | **0.48°** |
      | 流水线（B） | 200 µs（2 拍） | **45.4°** |
      | 流水线 + 单 CS 窗口 32bit（不可行） | 100 µs | 22.7° |
      10 kHz 电流环在 1800 RPM 下只有 ~16 个采样点/电周期，45° 的反馈滞后不可接受。
      **故 A 保持默认；B 作为编译开关保留**（`AS5047_READ_MODE_DMA`）。

      **四、B 的静态结论（已编译+反汇编核对，未上板）**：
      `Encoder_Adapt.c:129/151` 已分别调 `bsp_as5047AsyncStart()` / `bsp_as5047AsyncFetch()`；
      `as5047_read_register_raw()` 有 `s_bus_mode == AS5047_BUS_ASYNC` 提前返回守卫，
      所以 ISR 不会碰到那个会死等的 `as5047_transfer_dma()`（反汇编确认它只被
      `as5047_transfer()` 调用）；`bsp_as5047AsyncFetch()` 内部只有
      `as5047_async_kick()`（= `HAL_SPI_TransmitReceive_DMA`，fire-and-forget）与
      `as5047_track()`（纯计算）⇒ **ISR 路径零阻塞**。`DMA2_Channel1/2_IRQHandler`
      是驱动内的强符号，会覆盖启动文件的 weak `Default_Handler`。
      两种模式均已编译链接通过（DMA 档 FLASH +2.2 KB）。**上板未验证**。

      **五、结论：A 的优化点是"开销"不是"协议"。** 已实施：
      ① `as5047_cs_gap()` 原为 24 轮 `volatile` 循环，编译器生成 7 条指令/轮（含两次栈访问）
      ⇒ **≈240 周期 ≈2 µs**，是手册要求（`t_CSn`/`t_L` 各 350 ns，p12 时序表）的 5.7 倍；
      每样本要等 4~6 次 ⇒ 光等 CSn 就吃掉 FOC 周期的 ~12%。现改为 **DWT 计时的 450 ns**
      （=54 周期，1.3× 余量，由 `SystemCoreClock` 推导，不依赖编译器周期估算），
      DWT 不可用时退化为 16 轮 NOP 兜底（保守偏长但绝不低于 350 ns）。
      ② 去掉每帧开头**重复**的 `CS_HIGH + gap`（上一帧结尾刚做过 t_CSn），
      每样本 gap 由 6 次降到 4 次。每帧变为 `CS_LOW, gap(t_L), 传输, CS_HIGH, gap(t_CSn)`。
      预期编码器读取 **17 µs → 6.7 µs**（约省 10% 的 FOC 中断预算），零延迟、零协议风险。
      **收益待 `diag` 的 `ENC read` 实测确认，不要当成已实现的事实。**
      顺带补 §16.5 ⑭ 的 DMA 路径预测补帧。
- [ ] **17.2.5 移除 `-save-temps`**
      当前 FLAGS 里有 `-save-temps`，会在源码目录留一堆 `.i/.s`，拖慢构建
      （不影响运行时间，顺手清掉）。
- [ ] **17.2.6 按测量结果决定是否绕开 HAL**
      若 D 项占比可观，把 `ADC1_2_IRQHandler` 改成直接服务（跳过 `HAL_ADC_IRQHandler`）。
- [ ] **17.2.7 验收门**：任一工况下 `g_foc_wcet_us` ≤ 周期的 60%（即 ≤60 µs），
      且 `-O2` 档下控制行为与 `-Og` 一致（同工况对比速度/电流波形）。

---

## 18. 双编码器（电机端 + 输出端）方案（2026-09-27 立项）

### 18.1 现状与目标

- 电机端：AS5047P（SPI3 重映射 PC10/11/12 + PA15 软 NSS），14bit，**给 FOC 提供电角度**。
- 输出端（新增）：环形磁铁 + **6 个线性霍尔**，提供输出轴（轮端）角度。
- 两者之间是 **6:1 减速箱**，`Motor_Params.Reduction_ratio = 6.0f`（此前从未被消费）。

### 18.2 引脚与外设配置（当前工作区，2026-09-27）

线性霍尔为模拟电压输入，使用 ADC；定时器 Hall 接口用于数字霍尔，不适用于直接读取这六路模拟量。

| 用途 | HAL 命名与引脚 | 当前配置 |
|---|---|---|
| 输出端六路霍尔 | ADC1 CH4/5/6/7/14/15，PA4/5/6/7、PC4/5 | 规则组连续扫描，DMA1_Channel1 循环，6 个 halfword |
| 电流 B | ADC2 CH11 / PC1 | 注入 rank1，TIM1_TRGO，无 ADC 中断 |
| 电流 C | ADC3 CH10 / PC0 | 注入 rank1，TIM1_TRGO |
| 母线电压 | ADC3 CH13 / PC3 | 注入 rank2，与电流 C 共用 ADC3_IRQn |

ADC1 六路采样时间为 28.5 周期；按 15 MHz、12 位转换另加 12.5 周期计算，单通道约 2.73 us、整轮约 16.4 us。这是顺序采样，不是六路同时采样；循环 DMA 数组也不自动构成同一轮一致快照，角度解算需设计稳定帧读取。

PA5/PC4 已配置为模拟输入；ENCODER_SERVICE_ENABLE_AS5600=0、ENCODER_SERVICE_ENABLE_FALLBACK=0，构建排除旧软 I2C 驱动。AS5047 使用驱动内初始化的 SPI3 重映射 PC10/11/12 + PA15 软件片选，不是 CubeMX 的 SPI2。

### 18.2.1 触发源与芯片命名

工程使用 STM32 HAL 命名，实际目标为 GD32F303RET6。HAL ADC1/2/3 对应 GD32 ADC0/1/2，TIM1 对应 GD32 TIMER0。不可把 STM32F103 的时钟限制或全部触发细节无条件套用到 GD32。

本地 HAL 的规则组与注入组使用不同的 EXTSEL/JEXTSEL 选择表。TIM1_CC4 是注入组可用项，不能当作 ADC1 规则组的同名触发项。当前霍尔使用软件启动后连续转换，无需新增 TIM2/TIM3。ADC2/3 注入组使用 TIM1_TRGO；ADC1 的 .ioc 中残留注入触发参数不表示已启用注入组。

### 18.2.2 规则组与注入组的抢占

STM32F103 的 RM0008 §11.3.9 明确：外部注入触发会复位当前规则转换，执行注入序列，再从被打断的规则通道重新转换。不能从“共用 SAR、不能并行”推导出“必须等待当前规则转换完成”。原文的 2.73/16.8 us 最坏等待推论及“不允许电流注入与规则组共用 ADC”结论撤回。

依据：[ST RM0008 §11.3.9](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。GD32 的具体时序应另外依据 [GD32F30x 用户手册](https://www.gd32mcu.com/data/documents/userManual/GD32F30x_User_Manual_Rev2.9.pdf)，不可仅凭兼容 HAL 认定全部相同。

将霍尔与电流放在不同 ADC 是可选的资源隔离设计，但不能解决 HAL 第一次注入中断中清 JEOCIE 的软件路径。当前分配仍需完成 §18.7 的修复与验收。

### 18.2.3 母线电压与慢安全的依赖

当前 FOC ISR 读取 ADC3 注入 rank2 的 JDR2 后发布 Vbus；SafetyTask 读取同一个 Vbus。ADC3 规则 CH13 虽仍配置，但 main_user 没有启动它，也没有独立母线电压 DMA。

当前快慢安全共用采集/发布链，不能声称慢安全独立于 FOC。FOC 中断停止而任务继续运行时，Vbus 可以冻结，SafetyTask 仍可能喂 IWDG。需增加采样序号/时间戳超时检测，或设计并验证独立慢采集路径；目前不能当作已完成。

同一引脚分别做规则和注入采样，只是不同时间点的数据，不必数值相同，也不构成独立传感器冗余。不得依赖“读 JDR 自动清 JEOC”的未经核实假设判断新鲜度；应按目标芯片的标志清除规则管理。本地 HAL IRQ 在回调后显式清 JEOC。

### 18.3 软件架构：从单例改成实例

现状：`Encoder_Adapt` 是**单例**（`Encoder_t Encoder`），`FOC_run.c` 直接调
`Encoder_Update()`。

兼容双编码器的改法（尽量不动现有调用点）：

1. **实例化**：`static Encoder_t s_enc[ENCODER_ID_COUNT]`，
   `Encoder_Update()` 等现有 API 变成"作用于 `ENCODER_ID_MOTOR`"的薄封装，
   另加 `Encoder_UpdateId(id)` 一族。这样 `FOC_run.c`、`main_user.c` 一行不用改。
2. **注册表扩展**：新增 `Encoder_Driver_Hall6`，与 `Encoder_Driver_AS5047/AS5600`
   并列注册进同一张 vtable 表 —— "换编码器只注册一下"这个性质自然延续到第二个实例。
3. **两条路径的职责必须分开，不要合并成一个角度**：
   | | 电机端 AS5047 | 输出端 Hall6 |
   |---|---|---|
   | 物理量 | 电机轴机械角 → **电角度** | 输出轴机械角 |
   | 用途 | FOC 换相（**实时性关键**） | 位置环 / 轮端角度 |
   | 节拍 | 10 kHz（FOC ISR 内） | 1~2 kHz |
   | 权威性 | 换相与转速 | **位置** |
   - 减速箱有**回隙**，`电机角/6` 与 `输出角` 必然差一个回隙带，**不能互相替代**。
   - 建议：位置环用输出端角度；FOC 换相与速度环继续用电机端。
     `Reduction_ratio` 终于有了消费者——用于回隙/传动比的一致性观测（诊断量）。
4. **解码放在哪**：6 通道解码 = 3 组 atan2 + 偏移/增益校正，是 float 重活，
   **不要**放进 10 kHz 主路径。推荐在 FOC ISR 里 10:1 分频执行
   （对应 Phase 6 候选方案 A，与电流环确定同步），或另开 1 kHz 任务。
5. **标定接口**：`Encoder_SetDirection/SetZeroOffset` 已有，两个实例各自独立配置。

### 18.4 输出端角度驱动的实现步骤

`bsp_hall6.c/.h` + `Encoder_Driver_Hall6`：

1. **归一化**：每通道 `h_i = (raw_i − off_i) × k_i`，消除偏置与幅度差。
2. **角度解算**：先标定各通道实际电相位。相差 180° 的 0/180、60/240、120/300 三对是反相信号，**不是正交 sin/cos 对**，不能分别直接 atan2。
   若确认六路均匀间隔 60°，可先做对径差分，再按已标定相位投影：
   `C = Σ h_i cos(φ_i)`、`S = Σ h_i sin(φ_i)`，按模型 `h_i ≈ A cos(θ−φ_i)` 取 `θ = atan2(S,C)`；方向和零位由标定确定。也可对独立相位通道做最小二乘重建，再查表校正。
   若磁环极对数 P_out>1，仅靠周期信号存在机械角的极周期歧义，不能直接当作上电绝对单圈角；需参考零位或可信初值与连续跟踪。
3. **健康判据**（免费得到的诊断量）：
   - 三相对称性残差：理想情况下 `h_a + h_b + h_c ≈ 0`，残差大 = 霍尔老化/偏心
   - 某通道 AC 幅度≈0 = 该霍尔失效/虚焊
4. **标定方法（关键技巧）**：**用电机端编码器当参考来标定输出端**
   —— 电机转 6 圈 = 输出 1 圈，`θ_out_ref = θ_motor/6`（扣除回隙），
   用它做参考拟合每只霍尔的 offset / gain / 相位误差。
   这比"假设霍尔装得很准"可靠得多。
5. **输出接口**：与 AS5047 驱动同构（`angle`/`mech_angle`/`angle_delta`/速度 LPF/
   多圈累加），直接复用 `Encoder_Adapt` 的 vtable 与上层逻辑。
6. **电角度**：`eleangle = wrap(mec × P_out)`，`P_out` 是环形磁铁的极对数。

### 18.5 环形磁铁极对数 P_out 的确认方法

**可以，而且你提的方法（转 6 圈看周期数）就是对的。** 三条路，推荐第 2 条：

1. **人工数**：输出轴转一整圈，看某一路霍尔的完整周期数 —— 周期数就等于 `P_out`。
   （注意必须按**输出轴**一圈算；电机转 6 圈正好对应输出 1 圈。）
2. **自动化辨识（推荐，复用现有 identify 框架）**：加一条 `identify hallpole`，
   命令电机以恒定低速转 **36 个电机圈 = 6 个输出圈**（用电机端编码器判终点），
   同时 1~2 kHz 采一路霍尔；对候选整数 P=1..20 用
   `θ_out = θ_motor/6` 做参考做最小二乘拟合 `A·cos(P·θ_out + φ)`，
   **拟合残差最小的 P 就是答案**（和 `identify pole` 用 pf/pr 取整是同一套路）。
   比数零穿越稳，因为不依赖阈值和噪声。
3. **静态看磁环**：多极磁环能直接看到交替的 N/S 分段，数一下即可（粗验）。

**几何自检（重要）**：6 只霍尔要构成对称的 6 相（电角度间隔 60°），必须满足
`P_out × Δθ_mech ≡ 60° (mod 360°)`，其中 `Δθ_mech` 是相邻霍尔的**机械**间隔。
- 若 `Δθ_mech = 60°` → P 必须是 1/7/13/19…
- 若 `Δθ_mech = 30°` → P 必须是 2/8/14/20…
- 若 `Δθ_mech = 20°` → P 必须是 3/9/15/21…

测出的 `P_out` 若**不满足**这个同余关系，说明要么霍尔间距不是设计值，
要么有霍尔装歪/失效，要么磁环根本不是多极环 —— 这本身就是一条很有价值的诊断。
（PCB 上霍尔的实际间距是可量的，先量一下就能把候选 P 缩小到少数几个。）

> 注：`Motor_Params.Pole_Pairs = 21`，`Reduction_ratio = 6`，**21/6 = 3.5 不是整数**，
> 但输出磁环的极对数与电机极对数是独立参数，不能用 21/6 推断 P_out。理想无回隙时，连续机械角可按减速比换算；输出磁相位还需 P_out、零位及极周期定位，实际还受回隙和弹性影响。

### 18.6 待办（状态由 §18.7 复核补充）

| # | 待办 |
|---|---|
| ⑮ | 已在当前源码释放 PA5/PC4：AS5600 与 fallback 均关闭；再生成后保持模拟输入并复核构建排除项 |
| ⑯ | 实现 `bsp_hall6.c/.h` + `Encoder_Driver_Hall6` + 服务层实例化改造 |
| ⑰ | 实现 `identify hallpole`（拟合式极对数辨识） |
| ⑱ | 6 霍尔标定（以电机端编码器为参考）+ 健康诊断量接入 RTT `get` |

### 18.7 外设与启动流程复核（2026-09-27，静态审查，未上板验收）

**结论：ADC 分配与 FreeRTOS 优先级总体合理，但当前配置和启动代码尚未配套，不能标记为正确可用或已定稿。** 本次只修订文档，不改固件和 .ioc。

#### 18.7.1 当前配置与问题

| 项目 | 当前事实 | 结论 |
|---|---|---|
| ADC1 | 连续规则扫描六路霍尔，DMA1_CH1，未启用注入组 | 与用途一致；解码时需保证 DMA 帧一致性 |
| ADC2 | CH11 注入 rank1，TIM1_TRGO，无 IRQ | 与电流 B 用途一致；需验证与 ADC3 rank1 的采样对齐 |
| ADC3 | CH10/CH13 注入 rank1/2，TIM1_TRGO，IRQ=1 | 分配一致，但 ContinuousConvMode=DISABLE 仍触发本地 HAL 关闭 JEOCIE 的分支 |
| 电流零偏校准 | Offset_Current_Start/Reading 仍启动/停止 hadc1，读 ADC1->JDR1 | **迁移遗漏**：应配套迁到 ADC3，确认采满 1000 次，失败禁止继续强拖/运行 |
| FOC 回调 | 已筛选 hadc3，正常路径读 ADC3 JDR1/JDR2、ADC2 JDR1 | 正常路径已迁；校准提前 return，g_foc_isr_cnt 不计校准回调，不能单凭此计数判断校准 IRQ 是否进入 |
| SystemView | ADC3_IRQHandler 仅调用 HAL，无 FOC Enter/Exit 埋点 | **迁移遗漏**，时间线缺失不等于硬件没进中断 |
| 慢安全 | SafetyTask 读取 FOC 发布的 Vbus | 缺采集新鲜度检查，不能声称独立采集或已消除 FOC 停止风险 |
| SPI 编码器 | 2026-09-28：默认单帧寄存器轮询，DWT 微秒截止时间；DMA 专用传输；时间戳补偿 | 两种构建与协议测试通过，校准使用即时两帧；硬件时序及闭环效果待验收，见 AS5047_Single_Frame_Readout.md |

#### 18.7.2 分离 ADC 后仍可能只进入一次回调

`Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c` 的 `HAL_ADC_IRQHandler()` 在如下条件成立时清 `ADC_IT_JEOC`：

```c
ADC_IS_SOFTWARE_START_INJECTED(hadc) ||
(HAL_IS_BIT_CLR(hadc->Instance->CR1, ADC_CR1_JAUTO) &&
 ADC_IS_SOFTWARE_START_REGULAR(hadc) &&
 hadc->Init.ContinuousConvMode == DISABLE)
```

当前 ADC3 的第二组条件全部成立，即使规则组没有启动也一样：判断检查配置位与句柄，不检查规则组是否正在转换。第一次回调前 JEOCIE 就被清除，同时 HAL 清 INJ_BUSY。ADC3 取代 ADC1 做 FOC 源本身不会避开这一分支。

先在该分支和 ADC3 回调入口观察 CR1.JEOCIE、CR2.JEXTSEL/JEXTTRIG、hadc3.State。可用 ContinuousConvMode=ENABLE 做单变量诊断（保持规则组不启动、AutoInjectedConv=DISABLE），但是否作为最终兼容配置需验证；另一方向是核对 HAL 版本并修正/替代 IRQ 完成判断。不要开启 Auto Injected Mode 改变 PWM 外部触发语义，也不要只伪造句柄配置。

#### 18.7.3 TIM1 与 FreeRTOS 配置

- TIM1：120 MHz、PSC=0、ARR=6000、中心对齐，PWM 约 10 kHz；CH4 为 PWM1、CCR4=5940、TRGO=OC4REF。PWM1 的 OC4REF 上升沿位于下计数穿越 CCR4，不能声称与 CENTERALIGNED2 的上计数 CC4 事件相位相同。按配置距顶部约 0.5 us；两 rank 转换约 3.47 us（不含触发同步与 IRQ 延迟），需实测采样窗口及总 WCET。
- HAL 停 CH1~3 不一定停止计数器或清 MOE：条件宏要求所有 CCxE/CCxNE 都为零，CC4E 也在掩码中。当前 PowerStage_Disable 由 DRV COAST 关输出并保存/恢复 CR1；PowerStage_MoeOff 才无条件清 MOE。两条路径分别验证，不把 CEN/MOE 与 JEOCIE 混为一谈。
- BreakState=DISABLE：当前未启用 TIM1 硬件 Break。PA12 nFAULT EXTI=0 做软件关断；驱动器自身保护与 MCU Break 是不同机制。
- NVIC 分组为 4 个抢占位，MAX_SYSCALL=5：EXTI=0、ADC3=1、SPI/DMA=2、TIM4=4 不调用 FreeRTOS API；CAN RX/TX/SCE=6、USART2=7 允许使用 FromISR API。SysTick/PendSV=15 由端口接管，TIM6=15 提供 HAL 1 ms 时基，当前配置符合该分层。
- TIM1_UP_IRQn 虽在 NVIC 使能，计数器启动使用不带 IT 的 HAL_TIM_Base_Start；NVIC 使能不等于 UIE 已开。当前无需新增 10 kHz TIM1 更新 ISR。
- TIM4 为初始化期 5 ms 喂狗；调度器启动前关其更新中断，之后 SafetyTask 每 3 ms 喂狗。40.8 ms 是按 40 kHz LSI 算的估算超时，需核实实际 LSI 容差。

中断规则依据：[FreeRTOS Cortex-M 官方说明](https://freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)。

#### 18.7.4 CubeMX 再生成核对清单

| 项目 | 应保持的内容 |
|---|---|
| ADC1 | 六路规则组、Continuous=ENABLE、Scan=ENABLE、Injected=DISABLE、DMA 循环 halfword 长度 6 |
| ADC2 | CH11 注入 rank1、T1_TRGO、AutoInjected=DISABLE，无注入 IRQ |
| ADC3 | CH10 rank1 + CH13 rank2、Scan=ENABLE、T1_TRGO、AutoInjected=DISABLE；Continuous 最终值须与 §18.7.2 方案配套 |
| ADC3 IRQ | NVIC=1，向量调用 hadc3；校准/回调/启动停止使用同一 ADC |
| ADC1_2 IRQ | 当前无中断用户，可关闭；校准不可再依赖它 |
| TIM1 | 中心对齐、CH4 PWM1/CCR4、TRGO=OC4REF；关功率时保留采样，验证触发相位 |
| RCC | 目标 GD32 120 MHz；ADC DIV8=15 MHz，电气限制按实际 GD32 数据手册核验，不按 STM32F103 时钟上限验收 |
| IWDG/TIM4 | PSC32/Reload50；TIM4 PSC11999、ARR49；保留初始化到 SafetyTask 的喂狗交接 |
| SVC/PendSV/SysTick | 不生成与 FreeRTOS Direct Routing 重名的处理器 |
| CAN | RX/TX/SCE 均为 6，避免共用 HAL_CAN_IRQHandler 的嵌套重入 |
| 手写驱动 | 核查 AS5047 的 SPI3 重映射、PA15 片选以及未来 DMA2_CH1/2，与 CubeMX 配置一起审查 |

Encoder_Adapt.h 和顶层 CMakeLists.txt 是手写配置，不应写成 CubeMX 必然覆盖它们；再生成后核查构建排除项仍生效。当前 ADC1 DMA 未开启 NVIC 回调不妨碍 DMA 搬运，但也没有完成帧发布/错误回调机制。

#### 18.7.5 验收顺序

1. ~~修正校准 ADC 迁移遗漏；清零累计值，确认采满 1000 次、超时失败可观测且不继续使能功率。~~
   **已实现（静态，见 §16.3 缺陷 13）**：`Offset_Current_Start()` 已迁到 `hadc3`/`ADC3->JDR1`，累计值每轮清零，`g_offset_cal_done` 在算完均值后才置位，500ms 有界等待，超时→退中点+`FAULT_CURRENT_SENSE_MASK`+返回 −1，调用侧跳过强拖。**上板仍须确认**：`get` 的 `cur:` 行 `cal_done=1 n=1000`、`ib/ic` 都落在 2048±60 内；再人为把 ADC3 触发掐掉（或把 `OFFSET_CAL_TIMEOUT_MS` 临时改小）验证失败分支真的走到 `[CAL] ERROR` 且没有开功率。
2. 解决 ADC3 JEOCIE 保持问题；在 INIT、IDLE、RUN、FAULT 下确认持续回调，区分转换标志、IRQ 入口、回调计数。
3. 验证两路电流 rank1 的采样对齐、母线 rank2 更新、DMA 霍尔顺序与偏置标定结果。
4. 用硬件定时器参考沿和 ISR GPIO 测量触发到 ISR 延迟及 WCET；ISR GPIO 只表示 CPU 到达时间，不能单独证明采样时刻。
5. 分别测试 COAST、MOE 紧急关断、采样停滞和 SPI 异常；检查新鲜度保护与 IWDG 行为。
   **勘误**：原先此处写"恢复 ADC3 SystemView 埋点后再记录统计"——经全树检索，`SEGGER_SYSVIEW_RecordEnterISR()` **只有定义、没有任何调用点**（`stm32f1xx_it.c` 里也没有 `ADC1_2_IRQHandler` 了，`ADC3_IRQHandler` 的 USER CODE 段是空的），不存在"埋点挂错位置"这回事，也没有可"恢复"的东西。FOC ISR 耗时本来就用 DWT 自测：入口 `_isr_t0 = DWT->CYCCNT`、出口 `(DWT->CYCCNT - _isr_t0) / (SystemCoreClock/1000000U)` 写入 `g_foc_isr_us`/`g_foc_wcet_us`，RTT `diag` 的 `FOC ISR WCET` 行即读此值（校准路径在 `return` 前也记一次）。**若确实想要 SystemView 时间线**，那才需要新加钩子：在 `ADC3_IRQHandler` 的 `USER CODE BEGIN ADC3_IRQn 0/1` 里分别插 `traceISR_ENTER()`/`traceISR_EXIT()`（宏已在 `SEGGER_SYSVIEW_FreeRTOS.h:900` 备好），且优先级 1 的中断使用 SEGGER API 前要确认其内部临界区不与 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` 冲突——属新增功能，不是欠账。

本节来自源码与 .ioc 静态对照，未连接板卡，计算值及待验证项不记作实测通过。
