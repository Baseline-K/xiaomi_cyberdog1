# 电机驱动器 FreeRTOS 架构建议

## 1. 为什么在关节电机驱动器中使用 FreeRTOS

对于只包含 FOC、电流环、速度环、位置环和基础通信的电机驱动器，传统“中断 + while(1) 前后台”架构完全可以工作。

但机器人关节驱动器通常会逐渐加入：

- CAN / CAN-FD / RS485 / EtherCAT 接口
- 电机状态机
- 故障诊断
- 温度、电压等低频监测
- 参数管理与 Flash 保存
- Scope 波形采集
- Log 日志
- Watchdog
- USB 调试
- 后续固件升级等功能

如果全部放在 `while(1)` 中，随着功能增加，会出现大量：

```c
if(flag)
{
    ...
}

if(tick - last_tick > period)
{
    ...
}
```

这时实际上已经在手工实现一个简单调度器，而且会逐渐出现：

- 各模块执行周期不清晰
- 某个耗时模块阻塞其他模块
- 模块之间耦合越来越严重
- 优先级难以管理
- 后期维护和扩展困难

因此在机器人关节驱动器中引入 FreeRTOS，主要目的不是提高 FOC 性能，而是：

> **让系统功能按照任务、优先级和事件进行组织，同时继续保留电机控制的硬实时性。**

推荐总体思想：

```text
硬实时控制
    │
    └── ISR

系统功能
    │
    └── FreeRTOS Task
```

即：

> **ISR 负责必须准时发生的事情，Task 负责可以被调度的系统功能。**

---

# 2. 总体程序架构

推荐结构：

```text
                        STM32
                          │
        ┌─────────────────┴─────────────────┐
        │                                   │
        │          Hard Real-Time           │
        │                                   │
        │     PWM / ADC / Encoder ISR       │
        │                │                  │
        │                ▼                  │
        │          Motor Control            │
        │                                   │
        └─────────────────┬─────────────────┘
                          │
               Shared Data / Event
                          │
        ┌─────────────────▼─────────────────┐
        │             FreeRTOS              │
        │                                   │
        │  MotorManagerTask                 │
        │  CommunicationTask                │
        │  DiagnosticTask                   │
        │  ScopeTask                        │
        │  ParameterTask                    │
        │  LogTask                          │
        │  WatchDogTask                     │
        │  WorkQueue / BackgroundTask       │
        └───────────────────────────────────┘
```

核心原则：

```text
控制算法的严格时序
        ↓
由硬件定时器 / ADC / ISR 保证

通信、日志、状态机、参数等系统功能
        ↓
由 FreeRTOS 调度
```

---

# 3. ISR 应该负责什么

ISR 只负责对实时性要求最高、必须在确定时刻执行的功能。

典型包括：

## 3.1 PWM / ADC ISR

推荐负责：

```text
ADC Current Sample
        │
        ▼
Clarke
        │
        ▼
Park
        │
        ▼
Id / Iq Current Loop
        │
        ▼
Voltage Feedforward / Compensation
        │
        ▼
Inverse Park
        │
        ▼
SVPWM
        │
        ▼
PWM Register Update
```

也就是完整的高速 FOC 控制链。

例如：

```c
void MotorControl_ISR(void)
{
    CurrentSample();

    EncoderFastUpdate();

    Clarke();
    Park();

    CurrentLoop();

    VoltageCompensation();

    InvPark();
    SVPWM();

    PWM_Update();
}
```

---

## 3.2 速度环、位置环、扭矩环

如果这些控制环对周期确定性要求较高，建议继续由高速 ISR 分频执行。

例如：

```text
FOC / Current Loop : 40 kHz
Torque Loop        : 4 kHz
Speed Loop         : 2 kHz
Position Loop      : 1 kHz
```

可以采用：

```c
void MotorControl_ISR(void)
{
    CurrentLoop();

    if (++torque_cnt >= 10)
    {
        torque_cnt = 0;
        TorqueLoop();
    }

    if (++speed_cnt >= 20)
    {
        speed_cnt = 0;
        SpeedLoop();
    }

    if (++position_cnt >= 40)
    {
        position_cnt = 0;
        PositionLoop();
    }
}
```

这样可以保证：

- 各控制环周期固定
- 环路之间相位关系固定
- 抖动小
- 控制链路容易分析

---

## 3.3 通信接收 ISR

CAN / UART / RS485 等通信 ISR 不应直接做复杂协议处理。

推荐：

```text
CAN RX ISR
    │
    ├── 读取硬件 FIFO
    ├── 清中断标志
    └── 把数据放入 Queue / RingBuffer
                    │
                    ▼
          CommunicationTask
```

即中断只负责“尽快把数据接住”。

不要在 ISR 中执行：

- 大量协议解析
- Flash 擦写
- printf
- 日志格式化
- 文件系统操作
- 长时间循环
- 大量 memcpy

---

## 3.4 硬件保护 ISR

过流、驱动器 Fault 等硬件保护应具有很高优先级。

例如：

```text
Comparator / Gate Driver Fault
            │
            ▼
          ISR
            │
            ├── Disable PWM
            ├── Record Fault Code
            └── Notify MotorManagerTask
```

其中：

```text
Disable PWM
```

必须立即执行。

而：

```text
故障日志
参数保存
上报上位机
状态机切换
```

可以交给 Task。

---

# 4. FreeRTOS Task 如何划分

第一版建议不要创建过多 Task。

对于关节电机驱动器，建议控制在大约 6~8 个核心任务。

推荐：

```text
FreeRTOS
│
├── MotorManagerTask
├── CommunicationTask
├── DiagnosticTask
├── ScopeTask
├── ParameterTask
├── LogTask
├── WatchDogTask
└── WorkQueue / BackgroundTask
```

---

# 5. MotorManagerTask

这是整个关节驱动器的系统管理核心。

主要负责：

- 电机 Enable / Disable
- 控制模式切换
- 状态机
- 故障处理
- 标定流程
- 运行条件检查
- 接收上层目标值

推荐状态机：

```text
POWER_ON
    │
    ▼
INIT
    │
    ▼
CALIBRATION
    │
    ▼
READY
    │
    ▼
RUN
    │
    ├──────── Fault ───────► FAULT
    │
    └──────── Disable ─────► READY
```

MotorManagerTask 不负责高速 FOC 计算，而是负责决定：

```text
当前电机应该处于什么状态
当前应该使用什么控制模式
当前控制目标是什么
```

然后把这些目标提供给控制 ISR。

例如：

```text
CommunicationTask
       │
       ▼
Command Buffer
       │
       ▼
MotorManagerTask
       │
       ▼
Control Reference
       │
       ▼
Motor Control ISR
```

---

# 6. CommunicationTask

负责：

- CAN / CAN-FD
- RS485
- USB
- EtherCAT 上层接口
- 协议解析
- 指令检查
- 状态反馈
- 参数读写命令

推荐数据流：

```text
Communication RX ISR
        │
        ▼
       Queue
        │
        ▼
CommunicationTask
        │
        ├── Parse
        ├── Validate
        ├── Update Command
        └── Generate Response
```

例如：

```text
CAN Frame
    │
    ▼
CommunicationTask
    │
    ├── Torque Command
    ├── Speed Command
    ├── Position Command
    ├── Enable / Disable
    └── Parameter Command
```

其中真正给高速控制环使用的目标值，建议写入专门的 Command / Reference 数据结构，而不是让 ISR 直接访问复杂协议数据。

---

# 7. DiagnosticTask

负责低频状态监测，例如：

```text
10 Hz ~ 100 Hz
```

监测内容包括：

- MOSFET 温度
- 电机温度
- 母线电压
- 驱动芯片状态
- 编码器状态
- 通信错误
- 电流采样异常
- 电机运行状态

输出通常分为：

```text
Normal
Warning
Derating
Fault
```

例如：

```text
温度升高
   │
   ├── Warning
   │
   ├── Derating
   │
   └── Fault
```

DiagnosticTask 检测到故障后通知 MotorManagerTask，由状态机统一决定是否停机。

---

# 8. ScopeTask

ScopeTask 用于电机控制波形采集和上位机调试。

推荐：

```text
Motor Control ISR
      │
      ▼
High-Speed Ring Buffer
      │
      ▼
   ScopeTask
      │
      ▼
USB / CAN / Ethernet
      │
      ▼
     PC
```

控制 ISR 只负责快速写入数据：

```c
Scope_Push(iq);
Scope_Push(id);
Scope_Push(speed);
```

不要在 ISR 中进行真正的数据发送。

ScopeTask 再负责：

- 打包
- 降采样
- USB / CAN 发送
- Trigger 管理

---

# 9. ParameterTask

负责参数持久化，例如：

- PI 参数
- 电机参数
- 编码器 Offset
- 电流 Offset
- Dead-Time LUT
- 标定结果
- 设备配置

推荐：

```text
Control Runtime Parameters
          │
          ▼
         RAM
          │
          ▼
    ParameterTask
          │
          ▼
     Flash / EEPROM
```

控制 ISR 永远只使用 RAM 中已经准备好的参数。

不要在控制 ISR 中直接写 Flash。

---

# 10. LogTask

高速 ISR 或重要 Task 只产生 Log 事件：

```c
LogPush(LOG_OVERCURRENT);
```

然后：

```text
ISR / Task
    │
    ▼
 Log Queue
    │
    ▼
  LogTask
    │
    ├── UART
    ├── USB
    └── Flash
```

这样可以避免 `printf()`、字符串格式化和慢速 IO 影响实时控制。

---

# 11. WatchDogTask

推荐采用“系统存活确认”的方式喂狗。

```text
Motor Control Alive ──┐
Communication Alive ──┤
MotorManager Alive ───┤
Diagnostic Alive ─────┤
                      ▼
                WatchDogTask
                      │
                 All Healthy?
                  /        \
                Yes         No
                 │           │
             Feed IWDG    Stop Feed
```

对于严重故障，建议：

```text
Disable PWM
    │
    ▼
Record Fault
    │
    ▼
Stop Watchdog Feed
```

---

# 12. WorkQueue / BackgroundTask

用于处理不应该放在 ISR 中、又不值得单独建立 Task 的工作。

例如：

- 延迟处理
- 非关键协议处理
- 参数保存请求
- Flash 操作
- 后台维护

结构：

```text
ISR / Task
    │
    ▼
 Submit Work
    │
    ▼
 WorkQueue
    │
    ▼
Deferred Processing
```

这样可以避免 Task 数量不断增加。

---

# 13. ISR 与 Task 之间如何联动

这是整个架构中最关键的部分。

可以将 ISR 与 Task 的交互分成四类。

---

## 13.1 ISR → Task：事件通知

例如故障产生：

```text
Fault ISR
   │
   ▼
Task Notification
   │
   ▼
MotorManagerTask
```

适合使用：

- Task Notification
- Binary Semaphore
- Event Group

对于单一事件，优先考虑 Task Notification，因为开销较小。

---

## 13.2 ISR → Task：数据传输

例如 CAN 接收：

```text
CAN ISR
   │
   ▼
Queue / RingBuffer
   │
   ▼
CommunicationTask
```

适合：

- Queue
- Ring Buffer
- Stream Buffer
- Message Buffer

---

## 13.3 Task → ISR：控制目标更新

例如 CommunicationTask 收到速度指令：

```text
CommunicationTask
       │
       ▼
MotorManagerTask
       │
       ▼
Reference Buffer
       │
       ▼
Motor Control ISR
```

推荐使用简单、固定大小的数据结构：

```c
typedef struct
{
    float position_ref;
    float speed_ref;
    float torque_ref;
    float iq_ref;

    uint8_t control_mode;
    uint8_t enable;
} MotorReference_t;
```

ISR 每个控制周期读取当前 Reference。

不要让 ISR 去等待 Mutex。

---

## 13.4 ISR → Task：状态上传

例如 FOC ISR 产生：

```text
id
iq
speed
position
torque
fault
```

推荐写入一个状态缓冲：

```c
typedef struct
{
    float position;
    float speed;
    float iq;
    float id;
    float torque;
    float vbus;
} MotorRealtimeState_t;
```

然后：

```text
Motor ISR
    │
    ▼
Realtime State Buffer
    │
    ├── CommunicationTask
    ├── ScopeTask
    └── DiagnosticTask
```

推荐使用：

- 双缓冲
- 快照复制
- 原子变量
- 短临界区

避免 Task 长时间锁住 ISR 需要访问的数据。

---

# 14. ISR 与 Task 共享数据的原则

最重要的规则：

> **ISR 绝对不要等待 Task。**

因此不推荐：

```text
ISR
 │
 └── Take Mutex
        │
        └── 等待 Task
```

ISR 应始终能够快速完成。

推荐：

```text
ISR
 │
 ├── 读取输入
 ├── 计算控制
 ├── 写入结果
 ├── 更新简单共享变量
 └── 必要时通知 Task
```

Task 则负责：

```text
Task
 │
 ├── 读取状态快照
 ├── 处理协议
 ├── 记录日志
 ├── 写 Flash
 └── 更新下一周期参考值
```

---

# 15. 推荐的整体数据流

```text
                      上位机 / Robot Controller
                              │
                              ▼
                       CAN / EtherCAT
                              │
                              ▼
                       Communication ISR
                              │
                              ▼
                            Queue
                              │
                              ▼
                    CommunicationTask
                              │
                              ▼
                     MotorManagerTask
                              │
                              ▼
                       Reference Buffer
                              │
                              ▼
┌──────────────────────────────────────────────────┐
│                Motor Control ISR                 │
│                                                  │
│ ADC → FOC → Current → Torque/Speed/Position      │
│                       │                          │
│                       ▼                          │
│                  PWM Update                      │
└───────────────────────┬──────────────────────────┘
                        │
                        ▼
                Realtime State Buffer
                        │
          ┌─────────────┼──────────────┐
          ▼             ▼              ▼
    ScopeTask    DiagnosticTask   CommunicationTask
          │                            │
          ▼                            ▼
     Debug Data                   Status Feedback
```

这个结构可以作为后续程序设计的主框架。

---

# 16. 推荐优先级关系

示意：

```text
最高
│
├── Hardware Fault ISR
├── PWM / ADC Motor Control ISR
├── Encoder ISR
├── CAN / UART RX ISR
│
├── MotorManagerTask
├── CommunicationTask
├── ScopeTask
├── DiagnosticTask
├── WatchDogTask
├── ParameterTask
├── LogTask
├── BackgroundTask
└── IdleTask
最低
```

具体优先级需要根据系统测量结果调整，但原则是：

```text
控制实时性
    >
指令与状态管理
    >
通信处理
    >
诊断
    >
调试 / 日志 / Flash
```

---

# 17. 第一版工程建议

第一版建议不要把原来的裸机电机控制代码全部重构。

可以逐步迁移。

## Step 1：保留原有控制 ISR

原来的：

```text
ADC / PWM ISR
    │
    ├── Current Sampling
    ├── FOC
    ├── Speed Loop
    ├── Position Loop
    └── PWM Update
```

保持基本不动。

---

## Step 2：加入 FreeRTOS

先只创建：

```text
MotorManagerTask
CommunicationTask
DiagnosticTask
WatchDogTask
```

确认系统稳定。

---

## Step 3：加入 Scope 和 Log

```text
ScopeTask
LogTask
```

把原来散落在控制代码中的调试输出逐渐移出去。

---

## Step 4：加入参数管理

```text
ParameterTask
```

把 Flash 操作和控制环彻底分离。

最终形成：

```text
硬实时控制
      +
FreeRTOS 系统管理
```

而不是重新设计整个 FOC。

---

# 18. 后续“小脑 + 关节电机”总体架构

后续做轮足机器人、机器狗、人形机器人时，可以在当前关节驱动器之上增加整机“小脑”。

总体关系：

```text
┌─────────────────────────────────────┐
│            Robot Cerebellum         │
│                                     │
│ Linux RT + C++                      │
│ State Estimation                    │
│ Kinematics / Dynamics               │
│ WBC / MPC / Gait / RL Policy        │
└─────────────────┬───────────────────┘
                  │
            EtherCAT / CAN-FD
                  │
       ┌──────────┼───────────┐
       ▼          ▼           ▼
    Joint 1    Joint 2      Joint N
       │          │           │
 STM32+FreeRTOS STM32+FreeRTOS ...
       │
       ▼
 FOC / Torque / Speed / Position
```

其中：

### 小脑负责

- 整机状态估计
- 多关节运动学
- 刚体动力学
- 步态生成
- WBC
- MPC
- 强化学习策略
- 计算各关节目标位置 / 速度 / 力矩

### 关节驱动器负责

- 电流采样
- FOC
- 扭矩闭环
- 速度闭环
- 位置闭环
- 编码器
- 驱动保护
- 通信
- 执行小脑下发的目标

所以最终关系是：

```text
Robot Cerebellum
       │
       │ Joint Command
       ▼
Joint Driver
       │
       ▼
Motor Torque
```

当前设计好的 FreeRTOS 关节驱动器，可以直接作为未来整机控制系统的底层执行节点。

---

# 19. 最终推荐

对于机器人关节电机驱动器，推荐采用：

```text
STM32
  │
  ├── ISR
  │    ├── FOC
  │    ├── Current Loop
  │    ├── Torque Loop
  │    ├── Speed Loop
  │    └── Position Loop
  │
  └── FreeRTOS
       ├── MotorManagerTask
       ├── CommunicationTask
       ├── DiagnosticTask
       ├── ScopeTask
       ├── ParameterTask
       ├── LogTask
       ├── WatchDogTask
       └── BackgroundTask
```

核心原则只有三条：

1. **FOC 和关键闭环仍然由硬件同步 ISR 保证实时性。**
2. **FreeRTOS 负责系统功能、任务优先级和软件复杂度管理。**
3. **ISR 与 Task 之间通过 Notification、Queue、RingBuffer 和简单共享状态进行联动，ISR 永远不等待 Task。**

这套结构既适合当前单关节驱动器，也能自然扩展到未来的轮足机器人、机器狗和人形机器人整机控制系统。
