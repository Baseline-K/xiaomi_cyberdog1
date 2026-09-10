# 故障处理：定向软重初始化 + CAN(EMCY) 上报 + 全自动恢复

> 日期：2026-09-11；参考：AC_Control（StateMachine.c / Safety_Module.c / And_uartcom.c）；实施：本工程 FreeRTOS 状态机 + CommTask。
> 相关：[FreeRTOS_Hybrid_Architecture_Migration_Plan.md](FreeRTOS_Hybrid_Architecture_Migration_Plan.md) Phase 4。

## 1. 行为总览

```
任意态 ──(Safety 锁存故障位/new fault)──▶ S_FAULT_NOW
   entry: coast + PowerStage_Disable + 锁存故障位; 上报置 active/dirty; 分类需重初始化的故障类
   do() : ① 定向软重初始化(编码器/CAN，非阻塞，失败下拍重试) ② 完成即自动投 EVENT_FAULTtoOVER
   exit : 清 g_fault_bits 与 MotorCtrl.faults（自动恢复）
S_FAULT_OVER
   do() : 若又出现故障 → 回 S_FAULT_NOW（防抖）
          静止(|speed|<0.1 rps) → 停止上报 → 自动回 S_IDLE
```
**全程无需主机/手动清故障**（`clearfault` 命令已删除；`safedbg clear` 仅调试保留）。

## 2. 故障分类与重初始化范围（照 AC_Control“定向重初始化”）

| 故障类 | 掩码 | FAULT_NOW 动作 |
|---|---|---|
| 编码器 | `FAULT_ENCODER_MASK (1<<14)` | `Param_init()`(AS5600 跟踪软复位,无 I2C) + `Param_deinit()`(清 dq/PLL) + `FOC_Generated_Reset()` |
| CAN 通信超时 | `FAULT_COMM_TIMEOUT_MASK (1<<13)` | `CAN_bsp_Recover()`（bus-off → Stop/Start/重挂通知） |
| 其它（过流/短路/过压/欠压/堵转/转速异常…） | 各自掩码 | 无需重初始化：下一拍直接自动转 FAULT_OVER |

> 明确**不做**的阻塞性操作（会饿死 CommTask/心跳、且与 10kHz 角度读冲突）：`DRV83XX_Init()`(~120ms)、`Offset_Current_Start()`(~350ms)、`bsp_as5600HwInit()`(带 HAL_Delay)。
> 状态机运行在 `MotorStateTask`（prio 4，栈 1KB），只做非阻塞软重置。

## 3. CAN 上报帧（EMCY）

- **ID**：`CAN_ID_EMCY = 0x080 + CAN_NODE_ID = 0x081`（标准帧，8 字节）。
- **发送**：`CommTask`，节拍 10ms；`FAULT_NOW/OVER` 期间每 10ms 重发（进故障 ≤10ms 内首帧）；回 `S_IDLE` 停止。心跳仍为 `0x581`(0xFE 标记)，每 100ms（CommTask 10×10ms 计数）。

| 字节 | 含义 |
|---|---|
| d[0] | 故障类：`0x01`=设备故障 |
| d[1..4] | **32 位故障掩码，小端**（= `g_fault_report_word`，与 RTT `FSM: FAULT_NOW, fault=0x…` 一致） |
| d[5] | 故障时刻的 FSM 状态（`motor_state_t`） |
| d[6] | 保留 0 |
| d[7] | 递增计数（丢帧检测） |

- 掩码定义见 `Safety_Module.h`（bit0 过流、bit1 短路、bit2 过压、bit8 欠压、bit13 通信超时、bit14 编码器、bit18 堵转、bit19 转速异常）。
- 状态帧 `MotorCtrl_GetStatus`(4B) 仅用于 enable/mode/state；其 byte2 原与故障 bit0 别名，现已修为**仅使能位**；32 位故障的权威通道是 EMCY。

## 4. 相关文件
- `Code_Source/APP/motor_state_machine.c/.h`：FAULT entry/do/exit、FAULT_OVER、上报全局、定向重初始化。
- `Code_Source/BSP/CAN_bsp.c/.h`：`CAN_ID_EMCY`、`CAN_bsp_Recover()`。
- `Code_Source/APP/app_tasks.c`：CommTask 10ms 节拍 + `CAN_SendFaultReport()`。
- `Code_Source/SERVICE/MotorCtrl.c`：`MotorCtrl_GetStatus` byte2 修复。
- `Code_Source/APP/RTT_Cmd.c`：删除 `clearfault`。

## 5. 验证
1. RTT `fault`（或手捏轴造转速异常）→ 串口出现 `FSM: FAULT_NOW, fault=0x…` → **自动** `--[FAULTtoOVER]--> S_FAULT_OVER --[toIDLE]--> S_IDLE`，无需任何手动命令。
2. CAN 分析仪看 `0x081`：进故障 ≤10ms 首帧、之后每 10ms；d[1..4] 与打印掩码一致；回 IDLE 后停止。
3. （可选）通信类：制造 bus-off 验证 `CAN_bsp_Recover` 后 CAN 恢复正常收发。
4. 回归：正常 start/stop、辨识链、S 曲线规划/前馈/LESO 不受影响。
