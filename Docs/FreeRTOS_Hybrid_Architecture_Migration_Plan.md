# FreeRTOS 混合式电机控制架构迁移计划

> 项目：Xiaomi CyberDog 电机驱动器  
> 目标平台：GD32F303RET6（Cortex-M4F，120 MHz；工程保留 STM32F1 HAL 兼容层）  
> 目标架构：PWM/ADC 同步中断执行硬实时 FOC与快速安全检测，FreeRTOS 管理状态机、慢速安全检测、通信和诊断  
> RTOS 选择：FreeRTOS，1 kHz 系统 tick  
> 文档日期：2026-08-16  
> 状态：规划稿；当前电机控制故障修复并完成基线验收后开始实施

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

## 2. 当前基线

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

- **`Core/Src/main.c`：`MX_IWDG_Init();` 会被重新启用**，而当前无人喂狗 → 每 ~160ms 复位。需重新注释为 `//MX_IWDG_Init();`（Phase 4 实现喂狗后再启用）。
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
| ADC1/2 FOC | 1 | 否 | 10 kHz硬实时控制 |
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
5. 重新审查当前 CAN RX=2、SPI DMA=2 等配置；一旦这些 ISR 使用 RTOS API，就必须移到允许的优先级范围。

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

- [ ] 修复 mode=1、目标1 RPS时不转且大电流的问题。
- [ ] 验证电流采样极性、相序、电角度方向/零位、极对数和单位。
- [ ] 验证速度反馈在手动转动和闭环运行时方向、量纲正确。
- [ ] 验证 Start/Stop重复操作不会残留 PI积分或异常PWM。
- [ ] 测量 10 kHz ADC ISR平均、P99和最坏执行时间。
- [ ] 特别测量 `bsp_as5600GetAngle()` 软件 I2C耗时及其关中断时间。
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

- [ ] 将 CAN RX优先级调整到 RTOS API允许范围。
- [ ] CAN ISR保留收帧和环形缓冲写入，增加任务通知。
- [ ] 将 `CANopen_OD_Process()` 从主循环移到 `CommTask`。
- [ ] 一次唤醒批量处理缓冲中的全部帧。
- [ ] 保留 RX总数、丢帧、溢出和最大队列深度统计。

验收门：

- 正常负载和突发16帧下协议解析正确。
- CAN高负载不改变 FOC周期。
- 无队列静默溢出；故障统计可从 RTT读取。

### Phase 3：迁移并强化业务状态机

工作项：

- [ ] 建立 `MotorEvent` 队列和静态存储。
- [ ] 建立 `MotorStateTask`，成为状态唯一写入者。
- [ ] 从参考项目提取状态/事件/动作表，建立目标映射和差异清单。
- [ ] 重写 `fsm_state_transfer/action_perfrom` 调度内核，不移植单 pending槽和 `PT_*` 协程宏。
- [ ] 先迁移 `IDLE/RUN/STOP/FAULT_ACTIVE/FAULT_RECOVERY`，再迁移 `INIT/CALIB/STANDBY`。
- [ ] 将 CAN/RTT Start、Stop、ClearFault改为发送事件。
- [ ] 将 `StartPWM()`/`StopPWM()` 仅保留为状态机内部动作。
- [ ] 将当前 `MotorCtrl` 的影子参数提交改成一致命令快照发布。
- [ ] 实现重复、非法、过期事件的明确处理和计数。
- [ ] 实现状态迁移日志，但日志输出放到低优先级任务。
- [ ] 为INIT/CALIB中的阻塞操作设置子步骤、超时和完成事件。

验收门：

- 状态转换表测试全部通过。
- 快速连续 `Start/Stop/Start` 无事件覆盖和错误PWM状态。
- CAN与RTT同时发命令时状态一致、无直接并发写入。
- 队列满时 Fault/Stop仍可锁存并处理。
- 在任意状态动作执行期间注入Fault，均能立即硬件关断并最终进入 `FAULT_ACTIVE`。
- INIT失败有确定安全去向，不会无限停留且仍保持PWM关闭。

### Phase 4：迁移并完善Safety快慢保护

工作项：

- [ ] 从参考项目迁移故障掩码、`SafetyFastInput_t/SafetySlowInput_t` 和必要配置结构，删除未使用平台耦合。
- [ ] 将只读阈值配置与累计器/环形缓冲拆为 `SafetyConfig/SafetyRuntime`，补充配置合法性和NaN/Inf检查。
- [ ] 建立原子的 `active/latched/first fault` 和 `fault_pending`。
- [ ] 将 `Safety_FastStep()` 接入10 kHz FOC ISR并纳入WCET测量。
- [ ] 新增静态 `SafetyTask`，以3 ms绝对周期调用 `Safety_SlowStep()`，传入实际 `elapsed_ms`。
- [ ] 建立当前工程遥测/命令/状态到 `SafetySlowInput_t` 的一致快照映射。
- [ ] TIM1 Break、FastStep紧急故障、驱动故障和严重软件故障执行立即PWM关断。
- [ ] 实现 `FAULT_ACTIVE/FAULT_RECOVERY` 进入、保持、恢复判定和分类清除策略。
- [ ] 将故障CAN上报交给 `CommTask`，故障快照持久化交给 `DiagTask`，不阻塞状态机。
- [ ] 看门狗只在关键任务均报告健康且 FOC未超时时喂狗。

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

- [ ] 将 `RTT_Cmd_Process()` 移入 `DiagTask`。
- [ ] RTT命令通过状态事件或参数接口操作电机，不直接 Start/Stop。
- [ ] 输出任务栈水位、运行时间、队列峰值、CAN统计和FOC WCET。
- [ ] 对高频遥测抽取，不在实时路径执行格式化浮点输出。

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

- [ ] 10 kHz FOC由硬件同步中断稳定运行，无漏周期。
- [ ] FreeRTOS tick为1 kHz，未用于产生FOC周期。
- [ ] ADC/Break等高优先级ISR不调用 FreeRTOS API。
- [ ] 所有普通状态转换仅由 `MotorStateTask`执行。
- [ ] 参考项目状态/事件/动作语义已迁移，单pending槽和 `PT_*` 宏未进入目标实现。
- [ ] `Safety_FastStep()` 在10 kHz ISR内运行并满足WCET预算。
- [ ] `Safety_SlowStep()` 由 `SafetyTask` 每3 ms调用并使用真实elapsed时间。
- [ ] 故障关断不依赖任务调度，Fault保持锁存。
- [ ] 故障位的ISR/任务并发置位是原子的，首故障与历史锁存不会被清除流程丢失。
- [ ] CAN/RTT不再直接修改运行状态或调用 Start/Stop。
- [ ] ISR与任务之间使用一致快照，不存在多字段并发撕裂。
- [ ] 所有队列、栈和错误计数可观测。
- [ ] 栈高水位、CPU占用、FOC WCET和通信压力测试合格。
- [ ] 控制性能不低于 Phase 0裸机基线。
- [ ] 每个迁移阶段都有独立提交和明确回退点。

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
