# 速度环增强计划（前馈 / LESO 自抗扰研究 + 轨迹规划可行性）

> 项目：小米 CyberDog 电机驱动（STM32F103 / GD32F103，DRV8353，AS5600）
> 分支：`agent/speed-loop-baseline`　日期：2026-09-05
> 承接：此前在会话中推进的“速度环增强”讨论（会话因上下文超限中断，计划此前未落盘，本文档将其物化）。
> 上游主计划：[Simulink_Model_Plan.md](Simulink_Model_Plan.md)（模型结构/里程碑 M1–M6，本文档是速度环环节的深化）。

---

## 0. 现状快照（截至 53647e3，已推 github/main）

| 项 | 状态 / 数值 |
|---|---|
| 离线辨识套件 `MID/MID_identify/` | ✅ R / 死区 / 磁链 / 极对数 / Ld/Lq / J 全部完成，含电压饱和保护 + 状态超时（`identify_j.c`：iq 上限 1.0A、速度阈值 0.30/0.50/0.70）|
| 辨识落库参数 | J=`2.5e-6` kg·m²（`main_user.c:136`）、Flux=`0.002896`、Kt=`0.0304`（1.5·P·Flux）、Ld=`0.0018`、Lq=`0.0021`、R=`2.92` |
| 速度环频域整定入模 | Kp=`2π·J·ωc·sinφm/Kt`、Ki=`2π·J·ωc²·cosφm/Kt·1e-3`（RPS 域，1kHz，UseI*Ts）|
| S2.2 dq 电流环前馈 | ✅ Motor_Ld/Lq/Flux 全局 + 根输入 `ff_en`/`we_elec`，`vd−=we·Lq·iq`、`vq+=we·(Ld·id+Flux)`，`ff_en`=辨识时关 |
| FOC ISR 计时 | ✅ `g_foc_isr_us`（每周期 µs）/ `g_foc_wcet_us`（最大），Ozone 直接 Watch |

**当前速度环体系（重要背景）：**
- 速度环**整段在 Simulink 模型内**（`OuterLoop` 子系统 → `PID_speed` 块，1kHz），经 Embedded Coder 生成，固件只填输入。
- 固件喂 `U.ref_speed = cmd->speed_rps`（[FOC_generated.c:155](..%2FCode_Source%2FSERVICE%2FFOC_generated.c)），`cmd` 来自 CAN/RTT —— **给定是阶跃**，固件侧目前**没有生产用 S 曲线 / 速度斜坡规划器**（仅辨识任务内部有 ramp）。
- 高速抖动问题仍在排查（`g_foc_isr_us` 观察 + [AS5600 滤波延迟分析](AS5600滤波延迟导致FOC高速失稳与磁链辨识失败.md)）。

---

## 1. 参考项目清单（本地已 clone / 存在）

| 项目 | 本地位置 | 关注点 |
|---|---|---|
| **AC_Control**（400W New PCB）| `E:\BaiduSyncdisk\AND_Engineering_Project\AC_Control\AC_Control_400W_New\AC_Control_NewPCB\AND_Source\MID\MID_foc\` | 速度环 **PI + 惯量/粘性/库仑前馈 + LESO 叠加**；S 曲线规划 `Traj_Plan.c`；齿槽/死区/辨识 |
| **SguanFOC_Library** | `E:\Engineering_Project\Motor_Control\SguanFOC_Library\`（含 v3.0.0/v3.0.1/v3.1.0/v4.0.0 分版本目录）| 速度环**完整 LADRC**（TD + 3 阶 LESO + PD）、抗齿槽、参数辨识、Q31 定点 |
| **servo_hub** | `E:\Engineering_Project\Motor_Control\servo_hub-master\` | 齿槽标定（`tq_fc` 法）、多轴伺服结构 |

> 本项目模型（死区/齿槽 LUT、多电机参数、辨识框架、外环结构）自始参考 AC_Control 风格设计（见主计划 §1/§4）。

---

## 2. 参考研究结论

### 2.1 AC_Control 速度环（`Speed_Loop.c` + `observer.c`）——“增强现有 PI”

结构（实测代码）：
```
PI(带宽100Hz整定) ——P → + ─┬─ LESO_ApplyControl(2阶LESO) → 限幅 → iq_ref
前馈 FF：J·α_ref·kff + B·ω_ref + fc·smooth_sign(ω) ─┘
```
- PI 增益：`kp = (100Hz·2π·J)/(1.5·P·Flux)`（即 `Kt` 归一），Ki 由离散 dt 缩放（`ki_per_second`）。
- **前馈三项全部来自“对象参数”而非误差**：惯量项 `J·α_ref·kff`（α_ref = 轨迹给定加速度，来自其 `Traj_Plan.c` 的 `SCurve_Speed_Profile.current_accel`）、粘性项 `B·ω_ref`、库仑项 `fc·smooth_sign(ω_ref)`（eps≈0.1RPS 平滑过零）。
- **LESO 叠加在 PI+FF 之上**：观测器估残余总扰动，输出 `iq_comp` 修正 iq 指令（AC_Control 为 **2 阶** LESO：z1=转速、z2=总扰动，`β1=2ω0, β2=ω0²`）。

### 2.2 SguanFOC 速度环（`Sguan_Ladrc.c`）——“完整替换 PI”

```
TD:  v2 += h·(−r²(v1−Ref) − 2r·v2);  v1 += h·v2          （光滑参考）
LESO 3阶: e=z1−Fbk;  z3+=(−β3e)h;  z2+=(z3−β2e+b0·u)h;  z1+=(z2−β1e)h
控制率:   u = (Kp·e1 + Kd·e2 − z3)/b0                     （z3=总扰动，整体补偿）
带宽参数化: w0=4·Wc, β1=3w0, β2=3w0², β3=w0³, Kp=Wc², Kd=2Wc   （单参数 Wc）
```

### 2.3 对比表

| | **AC_Control**（Speed_Loop.c + observer.c）| **SguanFOC**（Sguan_Ladrc.c）|
|---|---|---|
| 速度环本体 | **PI**（带宽 100Hz 整定）| **纯 ADRC 替代 PI** |
| 前馈 | **显式惯量/粘性/库仑前馈**：`J·α_ref·kff + B·ω + fc·sign(ω)`（辨识 J/B/fc + 轨迹加速度）| 无（b0 是唯一对象增益）|
| LESO 层 | 叠加在 PI+FF 之上补偿残余扰动 | 内置（整个控制 = PD − z3）/b0 |
| LESO 阶数 | **2 阶**（z1=转速, z2=总扰动）`β1=2w0, β2=w0²` | **3 阶**（z1,z2,z3=扰动）`β1=3w0, β2=3w0², β3=w0³` |
| 参考平滑 | 无 TD（靠前馈 + 轨迹加速度）| **TD 跟踪微分器**（光滑参考）|
| 补偿式 | `iq_comp = −z2/b0` 加到 PI 输出 | `u = (u0 − z3)/b0`（整体）|
| b0 | = Kt/J（rad/s 域，A→rad/s²）| 同（对象增益）|

**关键差异**：AC_Control = PI + 模型前馈 + LESO 扰动观测器**叠加**（“增强现有 PI”）；SguanFOC = TD + LESO + PD 的**完整 ADRC**（“整体替换”）。

### 2.4 对我们最值得学的点

1. **辨识参数前馈（AC_Control）最实用**：`J·α_ref` 用轨迹给定加速度前馈惯量项——加减速瞬态跟踪大幅改善（纯 PI 跟不上）。辨识的 J/Kt 现成，B/fc 也可由磁链辨识给（见 §4.1 落库缺口）。
2. **LESO 作叠加层而非替换（AC_Control）**：保留 PI 的简单 + 扰动观测器补 `−z2/b0`，风险低。工程细节：LESO 输入**实测 iq** 加 ~0.99 重低通滤电流噪声（观测器稳定关键），`iq_comp` 限幅防冲击——值得学。
3. **SguanFOC 的 TD + 单带宽 Wc**：TD 光滑参考避免阶跃电流冲击；3 阶 LESO 对 1 阶机械对象（速度）偏过设计，AC_Control 的 **2 阶更贴合**。
4. **b0 = 对象增益**：J=2.5e-6、Kt=0.0304 → 数值见 §4.2。

---

## 3. 轨迹规划（Simulink）可行性 —— 已用本机 R2025b 核实

> 背景问题：走“轨迹前馈”需要 `α_ref`，而当前给定是阶跃、无规划器；是否可用 Simulink 现成模块做速度/位置 S 曲线轨迹规划。

**核实结论（2026-09-05，安装列表见附录）：**

1. **没有“拖进模型当速度/位置环 S 曲线参考发生器”的现成 Simulink 专用块**（基础 Simulink / Motor Control Blockset 均无）。
2. **有现成的 MATLAB 函数级轨迹生成**（Navigation Toolbox / Robotics 共享包，`%#codegen`，支持 MATLAB Coder / Embedded Coder 代码生成）：

| 函数 | 形态 | 说明 |
|---|---|---|
| `trapveltraj` | 梯形速度 | 限速 + 匀加减速（含 PeakVelocity / Acceleration / AccelTime 可选项）|
| `minjerkpolytraj` | 最小加加速度 | 最接近“S 型”的平滑轨迹（速度、加速度均连续）|
| `quinticpolytraj` | 5 次多项式 | 位置/速度/加速度端点约束 |
| `bsplinepolytraj` | B 样条 | 多段插值 |
| `waypointTrajectory` | 类 | 航点插值轨迹对象 |

   用法：给 **航点 + 时间矢量 → 一次性输出整段 `[位置;速度;加速度]` 样本**。可用于仿真/离线预生成，也可经 **MATLAB Function 块** 在 Simulink 内调用并生成代码。

3. **对 MCU 运行时（ert）的建议**：这些函数按“waypoint 批量”设计（可变尺寸矩阵、一次整段规划），**不适合在每个 1kHz 控制周期反复整段重规划**；在线逐周期走样本需要把整段轨迹缓存后按时间步进。故生产路径推荐其一：
   - **固件手写小规划器**（同 AC_Control `Traj_Plan.c` 的 `SCurve_Speed_Profile` / `SCurve_Position_Profile`，相位机 + jerk/accel 限幅），`Traj_Plan.c` 结构可直接移植参考；本模型 §1 原计划即“S 曲线规划保持手写”。
   - 模型内自建：离散“Jerk 限幅 → 二次积分（ref → ref· → ref··）”小子系统，同样可生成代码；适合规划逻辑想收进模型、且未来位置环也入模时。
4. **若只用于“验证轨迹前馈价值 / LESO 对比”，优先用现成模块**：`minjerkpolytraj`（≈S 型）或 `trapveltraj` 在测试台生成平滑 `ref_speed + α_ref`，比手写省事 —— 先仿真量化“前馈能省多少”，再决定是否投入做运行时规划器。

> 结论偏好（用户）：**有现成轨迹生成函数就优先用** → 模型/仿真验证阶段采用 `minjerkpolytraj`/`trapveltraj`；运行时嵌入式规划器再单独评估。

---

## 4. 三条落地方向（可移植方案，均已核实公式/参数）

### 4.1 辨识结果 → 前馈所需参数的落库缺口

- 磁链辨识（`identify_flux.c:203-206`）同时拟合摩擦 `iq = B_A·ωm[RPS] + f_A`，输出：
  - `res.B = B_A·Kt`（**N·m per RPS**）、`res.friction_Coulomb = f_A·Kt`（**N·m**）
- **缺口**：目前只有 `Motor_Params.Rotor_inertia` 被自动提交（`identify_framework.c:199`）；**B / fc 只留在 `Identify_Result_t`，没有 `Motor_Params` 字段、也没进模型全局**。走含“粘性/库仑前馈”的方案前需补：加 `Motor_Params.B_nm_rps / friction_Coulomb_nm`（或电流域 `B_A / f_A`）+ 磁链辨识完成后提交。

### 4.2 b0 与单位换算（跨方案都要用）

机械：`J·dω_mech/dt = Kt·iq`。

| 量 | 值 / 式 |
|---|---|
| b0（rad/s 域，A→rad/s²）| `Kt/J = 0.0304 / 2.5e-6 = 12160 rad/s²/A` |
| b0（RPS 域，A→RPS/s，模型速度环原生域）| `Kt/(2π·J) = 12160/2π ≈ 1935.6 (RPS/s)/A` |
| 前馈 iq_ff（RPS 域，用已辨识机械量）| `iq_ff = (2π·J·α_rps2 + res.B·ω_rps + res.friction_Coulomb·sign(ω)) / Kt` |
| （等效电流域）| `iq_ff = J·(2π·a_rps2)/Kt + (res.B/Kt)·ω_rps + (res.friction_Coulomb/Kt)·sign` |
| 落库建议（含单位）| `B_A[A/RPS] = res.B/Kt`；`f_A[A] = res.friction_Coulomb/Kt`（与 AC_Control 电流域存储一致）|

### 4.3 方向 A —— 轨迹前馈（仿 AC_Control，先做惯量项）

- 内容：模型速度环加 `iq_ff` 输入支路 `J·α_ref·kff`（或扩展为 +B·ω+fc·sign）；固件补规划器输出 `ref_speed + α_ref`（或模型内建）。
- 模型 hook：`OuterLoop` 输出与 `CurrentController` 输入之间；新增根输入 `ref_accel_rps2`（连同已有 S2.2 加的 `ff_en/we_elec` 同层）。
- 前提：**先有平滑轨迹**（否则 α_ref 仅在阶跃瞬间有效）→ 依赖 §3。
- 收益：加减速瞬态跟踪、减速度环超调；对“已知目标运动”（位置环 / S 曲线到目标）最有效。
- 风险：低（可开关），但当前“外部持续给阶跃指令”场景价值有限。

### 4.4 方向 B —— LESO 叠加层（仿 AC_Control observer.c，2 阶）

- 内容：在现 PID_speed 输出后加 2 阶 LESO 补偿 `iq_comp = −z2/b0`；LESO 输入：y=`speed_fbk`（RPS），u=`iq_ref`；内部对 **iq 实测重低通(~0.99)** + `iq_comp` 限幅（学 AC_Control 防噪/防冲击细节）。
- 模型 hook：`OuterLoop` 内 `PID_speed` 之后（LESO 与 PID 同属 1kHz 速度环）。
- 前提：无需规划器、无需 B/fc（扰动观测器把摩擦/齿槽/负载当 z2 一起估）——**当前阶跃指令 + 抗负载/抗高速抖动场景最匹配**。
- 收益：抗负载/摩擦扰动、抑制残余模型误差，直接对治“高速抖动/带载”问题；保留 PI 简单性。
- 风险：低–中（b0 需按 §4.2 设 RPS 域值 ~1936；观测器带宽 ω0 需整定）。

### 4.5 方向 C —— 完整 ADRC 替换（仿 SguanFOC，TD + 3 阶 LESO + PD）

- 内容：TD 安排过渡 + 3 阶 LESO（z3=总扰动）+ PD，`u=(u0−z3)/b0`，单带宽 `Wc` 整定，替换 `PID_speed` 子系统内部。
- 前提：无（TD 自带参考平滑）；但 3 阶对 1 阶机械对象过设计。
- 风险：高（控制器整体替换，整定参数面变化大）；作为对照/备选。

### 4.6 推荐次序（建议，待确认）

1. **先写仿真对比**（测试台 + 现成 `minjerkpolytraj`/阶跃，测现 PI vs 各方案在超调/带宽/抗载上的差距）——量化收益后再动模型。
2. **方向 B（LESO 叠加）先行落地**：不依赖规划器、参数现成（b0≈1936 RPS域），风险最低，直接服务高速抖动/负载场景。
3. **方向 A（轨迹前馈）随后**：待运行时规划器确定（§3 决策：固件手写 vs 模型内建），再给模型加 `ref_accel_rps2` 前馈。
4. 方向 C 作为对照实验保留。
5. 每个模型改动需同步：`motor_params.m` 加参数 → 模型加块 → 重新生成 → 固件 `FOC_Generated_Init/Step` 填输入（与 S2.2 流程一致）。

---

## 5. 相邻待办 / 未决清单

- [ ] **高速抖动诊断**：用 `g_foc_isr_us` 观察 ISR 耗时；对照 [AS5600 滤波延迟导致 FOC 高速失稳分析](AS5600滤波延迟导致FOC高速失稳与磁链辨识失败.md)，确认抖动主因（PLL 角滞后 / 电流环抗扰 / 齿槽）。
- [ ] **B/fc 辨识落库**（方向 A 前置）：`Motor_Params` 增字段 + `identify_flux` 完成后提交。
- [ ] **运行时轨迹规划决策**：固件手写（移植 `Traj_Plan.c`）vs 模型内建 jerk 限幅 vs 维持仿真用 `minjerkpolytraj`（§3）。
- [ ] **齿槽转矩辨识**（servo_hub `tq_fc` 法）：最后一个辨识任务，需位置环 + 扰动观测能力。
- [ ] 方向确认后：把选定方案细化成模型改造 / 固件改造步骤（本计划 §4 之下）。

---

## 6. v1 落地记录（2026-09-06，可开关实验）

> 方案演进：原计划“模型内 minjerkpolytraj 包 MATLAB Function”——P0 实测其 codegen 对 M4F(单精度 FPU、double 软算)每次重定向 ~1.5–3ms、无 FPU 无关、违背控制路径零 aeabi_d 约定，放弃；**转参考项目做法**（servo_hub / AC_Control 均为控制器 ISR 内 C 规划器）：**固件 C 规划器 + 模型纯块三项前馈**。

**架构**
```
固件 speed_plan.c(10kHz 10:1→1kHz, AC_Control 式 jerk 三段 S 曲线)
   │ 输出 平滑 ref_speed(RPS) + α_ref(RPS/s)
   ▼
Simulink 模型(速度环本体不变, 1kHz PID)
   新输入 U.ref_accel + OuterLoop 三项前馈(Sum/Gain/Switch)：
   iq_ff = SpeedFF_J2piKt·α_ref + SpeedFF_B·speed_fbk + SpeedFF_Fc·sign(speed_fbk)
   门控：ctrl_mode==1 && SpeedFF_En>0.5  (10kHz iq 指令层，B·ω/fc·sign 用实测转速)
```

**改动清单**
| 位置 | 改动 |
|---|---|
| `Simulink_Model/CyberDog_Motor_FOC.slx` | 根输入 `ref_accel`；OuterLoop 内 FF 块（G_FFJ/B/Fc、Sign_FF、Sum_FFraw、Rel_FFmode/en、And_FF、Sw_FF、Sum_iqff 加第三输入）；已 `slbuild` 重新生成到 `generated/`（含 `rtGetNaN.c/h`）|
| `Code_Source/SERVICE/speed_plan.c/.h`（新增）| AC_Control `SCurve_Speed_Profile` 移植：jerk 受限 T1/T2/T3 三段 + 方向反转两段（先到 0）+ 目标变化自动重规划；`SpeedPlan_Step10k(active,cmd_rps,fbk_rps,&ref,&acc)` |
| `Code_Source/SERVICE/FOC_generated.c/.h` | Init 算 `SpeedFF_J2piKt/B/Fc` 并默认关；Step 用规划器填 `U.ref_speed/U.ref_accel`（速度模式且未 coast 平滑，否则直通）；RTT 开关封装 `FOC_SpeedPlan_Enable/SetTimeMs/…`、`FOC_SpeedFF_Enable` |
| `Code_Source/MID/MID_foc/foc.h` + `main_user.c` + `identify_framework.c` | `Motor_Params` 增 `friction_B_nm_rps`(N·m/RPS)、`friction_Coulomb_nm`(N·m)；identify flux 完成后提交（`Identify_Params_Update` TASK_FLUX）|
| `Simulink_Model/scripts/motor_params.m` | 加 `SpeedFF_En/J2piKt/B/Fc`（模型消费，ExportedGlobal）；`SpeedPlan_En/T/Db` 保留未用（固件规划用自模块 + RTT）|
| `CMakeLists.txt` | 显式加 `speed_plan.c`、`generated/rtGetNaN.c` |
| `Code_Source/APP/RTT_Cmd.c` | 命令 `planner <0|1>`、`plT <ms>`、`ff <0|1>`；`showparams` 打 friction + plan_en |

**构建**：Debug 通过，RAM 58.65% / FLASH 19.64%（相对基线 +144B RAM / +~3KB FLASH）。

### 台架 A/B + 执行时间测量（RTT）

前置：烧录新固件，先 `identify flux`（提交 B/fc → Init 重算 SpeedFF_*），需要的话再 `identify j`。**注意 identify 会重置 planner/ff 到 0，开关放辨识完成后再设。**

```
identify flux            # 提交 B/fc（showparams 应看到 friction: B_nm_rps/fc_nm 非 0）
planner 1                # 开 S 曲线规划器
plT 300                  # 机动时间 300ms（可 100~500 调）
ff 1                     # 开三项前馈（可对比 ff 0）
mode 1 ; spd 5 ; start   # 看平滑启动（无阶跃 iq 尖峰）
spd 10 ; spd 20 ; spd -15 ; spd 5   # 连续重定向：S 曲线、无阶跃冲击
ff 0 … ff 1（同工况）     # A/B：对比 speed_meas/iq
planner 0 … planner 1    # 回退对照（planner 0 = 阶跃直通原行为）
stop
```

观测：Ozone/J-Scope 挂 `Y.speed_meas_rps、Y.iq_meas、g_foc_isr_us、g_foc_wcet_us`。
验收：① 规划器 on 时启动/换向平滑无 iq 阶跃尖峰；② ff on 比 off 加减速段跟踪误差或 iq 峰值更小（数据留档）；③ 稳态/机动中 `g_foc_isr_us` 增量 < ~3–5µs（规划查询+FF 求和，1kHz 每 10 拍才跑），`g_foc_wcet_us` 应基本不变；④ RAM/Flash 可接受。
回退：`planner 0; ff 0` = 旧行为；git revert 撤销整体。

---

## 7. v2 常开化（2026-09-06，规划/前馈永久开启）

用户确认该行为**长期保留**，去掉实验开关：

1. **归位**：`speed_plan.c/.h` 移到 `Code_Source/MID/MID_foc/`（与 AC_Control `Traj_Plan.c` 同层）。
2. **常开**：删 `SpeedPlan_En`/`SpeedFF_En` 使能体系（固件 wrapper、模型门控块 `C_FFen/Rel_FFen/And_FF`、RTT `planner/ff` 命令、motor_params 相关参数）。模型前馈门控退化为**仅 `ctrl_mode==1`**（避免污染位置/转矩/辨识模式）；`plT <ms>` 保留作机动时间调节。速度模式速度目标**总是** S 曲线平滑、前馈**总是**施加。
3. 固件 `FOC_Generated_Init` 仍每次重算 `SpeedFF_J2piKt/B/Fc`（identify flux 提交 B/fc 后生效）并 `SpeedPlan_Reset()`。

**构建**：Debug 通过，RAM 58.65% / FLASH 19.54%（相对带开关版 -0.5KB FLASH）。

### 7.1 规划器工作流程（Q3）
1. 10kHz FOC ISR → `FOC_Generated_Step` 非辨识段，且**速度模式 && 未 coast** 时调 `SpeedPlan_Step10k(active, cmd_rps, speed_meas_rps)`。
2. 模块内部 10:1 → 每 1kHz 推进规划一次，其余拍复用上一拍 `ref/acc`。
3. 首次 active → 以**当前实测转速**为起点强制规划到当前目标（无扰起步）。
4. 每 1kHz 拍：`|cmd_rps−target|>0.05RPS` 自动重规划；否则沿 jerk 受限三段（加加速/匀加速/减加速）推进；方向反转→“先刹到 0 再反相”两段。
5. 输出写 `U.ref_speed`（平滑）+ `U.ref_accel`；模型速度环(1kHz PID)用平滑 ref，10kHz iq 层并入三项前馈。

### 7.2 g_foc_isr_us vs g_foc_wcet_us（Q4）
- 均在 `FOC_run.c` ISR 用 `DWT->CYCCNT`(120MHz→µs)测量，每拍都记（含 CALIB 早退）。
- `g_foc_isr_us`：每周期瞬时耗时，每 ISR 刷新 → 看实时抖动。
- `g_foc_wcet_us`：迄今最大耗时，仅当 `_el>g_foc_wcet_us` 抬升，单调不回落、从不复位 → 平时“定住”，只在出现新最坏单拍时跳变；运行中才会出现更大单拍（大电流加减速、软 I2C 偶发慢拍等）。如需按单次运行统计峰值，可在 `MotorCtrl_Start()` 置 0（默认未加）。

---

## 附录：轨迹函数核实细节

- 已安装：Navigation Toolbox、Robotics System Toolbox、Motor Control Blockset、Simulink Coder / Embedded Coder、Fixed-Point Designer（R2025b）。
- 函数位置：`trapveltraj`/`bsplinepolytraj`/`transformtraj` → `toolbox\shared\robotics\robotcore\matlab\`；`minjerkpolytraj` → `toolbox\shared\uav_rst\matlab\`；`quinticpolytraj` → `toolbox\shared\polytraj\`；均含 `%#codegen`。
- 未发现面向电机驱动速度/位置环的现成 S 曲线 Simulink 块（`roboticslib`/`navlib` 库名未装载，无可拖专用块；若后续确认某工具箱确有专用块，本节更新）。
