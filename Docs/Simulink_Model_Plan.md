# Simulink 电机控制模型生成计划（CyberDog 电机驱动）

> 项目：小米 CyberDog 电机驱动（STM32F103 / GD32F103，DRV8353，AS5600）
> 目标：用 Simulink 建模 FOC 算法核心，Simulink Coder 直接生成 C 代码，固件集成生成的算法代码运行。
> 建模范围（v1）：电流环 + 速度环。位置环为后续迭代。

---

## 1. 背景与目标

当前固件中 FOC 三环控制为手写 C 代码（`Code_Source/MID/MID_foc_float/`），算法与 I/O 混杂在一个 10kHz 中断回调里（`Code_Source/APP/FOC_run.c` 的 `HAL_ADCEx_InjectedConvCpltCallback`）。

本次工作的目标：

1. 用 Simulink 重建 FOC 算法核心（坐标变换、三环、死区补偿、SVPWM）。
2. 用 Embedded Coder 生成可直接编译进固件的 C 代码。
3. 固件通过 10kHz 中断调用生成的算法函数，手写代码只负责 I/O 与业务逻辑。
4. 所有电机参数与控制器增益由固件在运行时写入，实现**一套算法适配多种电机**。
5. 表格类前馈（死区补偿、齿槽转矩）的**查表数据与使能全部由固件在代码侧提供/控制**，模型只负责算法本身。

**v1 范围：电流环 + 速度环**（对应现有 `current_q_pid / current_d_pid / speed_pid`）。位置环、S 曲线规划、VF 强拖启动、参数辨识保持手写代码，不在本次模型中。

> 参考项目：`E:\BaiduSyncdisk\AND_Engineering_Project\AC_Control\AC_Control_400W_New\AC_Control_NewPCB`
> 其死区前馈（`dead_comp_ff.c`）、多电机参数配置（`Params_Config.c/h`）、在线/离线辨识（`Online_Para_Identify.c` / `Para_identify.c`）为更完善的实现，后续移植到当前项目；本模型的接口按其风格设计。

---
## 1.5 实施状态快照（截至 2026-08）

> 本快照反映**实际已建成的模型**，与 §2~§8 的"设计意图"对照。M3 代码生成前的最终状态。

**已建成：**
- **算法模型** `Simulink_Model/CyberDog_Motor_FOC.slx`：
  - 输入 `ia, ib, theta_elec, iq_ref, id_ref, speed_fbk, ref_speed, ctrl_mode` → 输出 `duty_u/v/w`
  - **MCB 变换块**：Clarke / Park / 反 Park（探针验证与固件约定完全一致）
  - **电流环**：`CurrentController` 子系统，两个 **PID Controller 块**（离散 PI、UseI*Ts、反算抗饱和、积分+输出限幅），10kHz
  - **速度环**：`OuterLoop` 子系统内的 **PID_speed 块**（1kHz、UseI*Ts、反算抗饱和），另含模式选择 + 齿槽前馈 LUT
  - **DeadTimeComp**：死区补偿（|I|→Vcomp 矢量对齐投影 + 使能），当前为简化版（无 LPF/淡出）
  - **VoltageLimiter**：圆形限幅（Vbus/√3·MaxMod）子系统
  - **SVPWM**：七段式马鞍波（自封装子系统）
- **测试台** `Simulink_Model/CyberDog_Motor_FOC_Test.slx`：
  - Model Reference 引用 FOC 模型 + **MCB Surface Mount PMSM + Average-Value Inverter** plant（连续）
  - 多速率：plant↔FOC 交界有 **Rate Transition**；速度环 1kHz、电流环 10kHz
  - 交互输入：4 个可调常量（`T_iq_ref/T_ref_speed/T_ctrl_mode/T_TL`）+ 手动开关（默认手动），点 Run 即可交互
  - 观测：`Scope_Outputs`（ia/speed_fbk/iq 三路带信号名）
- **参数机制**：`scripts/motor_params.m` 集中定义全部可调参数（Simulink.Parameter + ExportedGlobal），设为两个模型的 **InitFcn**（点 Run 自动定义）。电流环增益按**固件公式**推导（Kp=L·500·6.18，Ki=R·500·6.18·timFactor，UseI*Ts 直接对齐固件 ki）；速度环增益手动整定（占位 plant）
- **脚本**：`motor_params.m` / `m1_simtest.m` / `m2_simtest.m` / `sim_scenarios.m`

**已验证：** 4 工况闭环仿真（电流环/电流+速度环 × 空载/负载）、m2_simtest（速度 5.02RPS、超调 1.8%、死区/齿槽通断生效）、交互点 Run 可用。

**待办（M3+）：** 数据字典 sldd 迁移、导出函数代码生成、全局配置结构体（MotorCfg）、固件集成。

---

## 2. 建模边界（三层划分）

| 层 | 内容 | 归属 |
|----|------|------|
| **I/O 层** | ADC 注入采样、偏置标定、电流/Vbus 定标、电角度测量 + PLL 滤波、速度估计、位置累计、TIM1 占空比写入、硬件死区时间配置（TIM1 DTG / DRV8353 DT）、控制模式状态机、SCurve/爬坡规划、故障处理、参数加载与 Flash 持久化（参考项目移植）、在线/离线辨识（参考项目移植） | **手写代码**（保留现有） |
| **算法层** | 电流滤波（Lpf on id/iq）、Clark、Park、电流环 PI（q/d）、速度环 PI、齿槽转矩前馈（角度→iq LUT）、死区补偿前馈（电流→电压 LUT + 矢量投影）、反 Park、圆形限幅（过调制）、SVPWM 占空比计算 | **Simulink 模型 → 生成 C** |
| **参数层** | Motor_Params（R/L/Flux/极对数/惯量等）、三环 Kp/Ki/限幅、Vbus、最大调制度、PWM 周期、FOC 频率、速度环分频 N、死区补偿 LUT 数据、齿槽前馈 LUT 数据、各前馈使能标志 | **固件写入模型导出的全局配置结构体** |

**接口定义：**

```
模型输入（由固件填充）：
  ia, ib          [A]   三相电流（ia = -ic - ib 由模型内计算或外部给三路）
  theta_elec      [rad] 电角度（AS5600 + PLL 滤波后）
  theta_mech      [rad] 机械角（供齿槽转矩前馈查表；亦可由模型内部 θelec/极对数 计算）
  speed_fbk       [RPS] 机械转速反馈（电角速度/(2π·极对数)）
  ref_speed       [RPS] 速度目标值（业务逻辑给定）
  ctrl_mode       [int] 模式选择：0=转矩(iq_ref 直给) / 1=速度
  iq_ref          [A]   转矩模式下的目标电流（速度模式下为速度环输出）

模型输出（固件使用）：
  duty_u, duty_v, duty_w  [0..1] 归一化三相占空比
  dVd, dVq               （可选，死区补偿量，供 RTT 调试/监控）
  iq_out / speed_out      （可选，供 RTT 调试/监控）
```

**采样率：**
- 电流环：**10kHz**（= 120MHz / PWM_PERIOD(6000) / 2，`FOC_Frequency`）
- 速度环：**1kHz**（独立采样率；参考项目用 `SPEED_LOOP_RATE`/`FOC_Speed_N` 分频）
- 模型采用**导出函数 + 多速率**（见 §3.5）：电流环函数 10kHz，速度环函数 1kHz。两个函数**由固件决定在哪个中断中调用**——可同一中断内顺序调用，也可分属两个中断，模型无需改动。

---

## 3. 模型设计

### 3.1 模型文件

- `CyberDog_Motor_FOC.slx` —— 主模型（算法模型，代码生成源）
- `CyberDog_Motor_FOC_Test.slx` —— 测试台（MCB PMSM plant + 交互输入/观测，纯仿真）
- ~~`CyberDog_Motor_FOC.sldd`~~ —— 数据字典，**尚未创建**（当前参数经 `scripts/motor_params.m` + 模型 InitFcn 定义；sldd 迁移列入 M3）

模型顶层含 `FOC_Algorithm` 子系统（内部分层：OuterLoop 外环 + CurrentController 内环 + MCB 变换 + VoltageLimiter + SVPWM + DeadTimeComp）。**导出函数结构（CurrentLoop_step/SpeedLoop_step）尚未配置，属 M3**。

### 3.2 算法链（实际实现，映射自现有 FOC_run.c）

```
[输入] ia, ib, theta_elec, speed_fbk, ref_speed, ctrl_mode, iq_ref
   │
   ├─ [MCB Clarke]   ia,ib → iα,iβ                                ← 探针验证=固件 Clark
   ├─ [MCB Park]     iα,iβ + sin/cos → id,iq                       ← 探针验证=固件 Park
   │   （注：电流 LPF 尚未实现，后续如需按固件 L110-113 加入）
   │
   ├─ 速度环（OuterLoop 子系统）：
   │     ref_speed - speed_fbk → PID_speed（1kHz, UseI*Ts, 反算抗饱和）→ iq_ref_speed
   ├─ 齿槽转矩前馈（已实现，使能可调）：
   │     theta_elec → 1-D LUT（角度→iq 补偿）→ iq_ff
   ├─ 模式选择：iq_ref_cmd = (ctrl_mode==速度 ? iq_ref_speed : iq_ref) + iq_ff
   │
   ├─ 电流环（CurrentController 子系统）：
   │     PID_q：ref=iq_ref_cmd，fbk=iq → vq
   │     PID_d：ref=0，          fbk=id → vd
   │     （两个 PID Controller 块：离散 PI, UseI*Ts, 反算抗饱和, 积分+输出限幅）
   │
   ├─ 死区补偿前馈（DeadTimeComp 子系统，使能可调）：
   │     |I| = sqrt(id²+iq²) → 1-D LUT（电流→补偿电压 Vcomp）
   │     ΔVd = Vcomp·id/|I|，ΔVq = Vcomp·iq/|I|（矢量对齐投影）
   │     vd += ΔVd，vq += ΔVq
   │     （简化版：暂未含 i_min 淡出与 LPF，后续补）
   │
   ├─ [MCB 反Park]  vd,vq + sin/cos → vα,vβ                        ← 探针验证=固件 Reverse_Park
   ├─ VoltageLimiter 子系统：圆形限幅（vmax=Vbus/√3·MaxMod）      ← foc.c RevPark_Circle_Limitation
   ├─ SVPWM 子系统：uα/uβ → 占空比 duty_u/v/w（七段式马鞍波+零序注入）← Foc_Svpwm
   │
[输出] duty_u, duty_v, duty_w
```

### 3.3 死区补偿前馈（LUT）—— 已实现（简化版）

采用参考项目 `AC_Control_400W_New` 的**矢量对齐**形式（`dead_comp_ff.c`），`DeadTimeComp` 子系统：

- **1-D Lookup Table**：输入电流幅值 `|I| = sqrt(id²+iq²)` → 输出补偿电压幅值 `Vcomp`（线性插值，数据 `DeadComp_Lut_I/V`）。
- 矢量投影：`ΔVd = Vcomp·id/|I|`，`ΔVq = Vcomp·iq/|I|`，加在电流环输出 `vd/vq` 上（反 Park 之前）。
- 使能：可调参数 `DeadComp_En` 控制（Switch），固件按 **宏 + `DeadComp_Calibrated`** 决定（§4.2）。
- **已验证**：高电流工况下通/断有明显效果（稳态电流差异 ~0.7A）。
- **简化项（待补）**：id/iq 低通、Vcomp 低通、电流幅值淡出（i_min）、保守系数 gain **尚未实现**——后续按 `dead_comp_ff.c` 补全。
- 硬件死区时间（TIM1 DTG / DRV8353 DT）属 I/O 层，保留在代码中。

### 3.4 齿槽转矩前馈（LUT）—— 已实现

- **1-D Lookup Table**：输入**电角度 `theta_elec`**（0..2π）→ 输出 `iq` 补偿量（A）。（当前按电角度索引；如需按机械角/极对数归一化周期，后续可改。）
- 位置：在 `OuterLoop` 内加在电流环 `iq_ref` 之前（转矩/电流指令前馈），由 `CoggingFF_En` 控制（Switch）。
- 表格数据 + 断点（`Cogging_Lut_Angle/V`）为可调参数（ExportedGlobal），机制与死区补偿一致。
- **已验证**：通/断有明显效果（稳态电流差异 ~0.4A）。

### 3.5 多速率与中断划分（当前为仿真多速率；导出函数属 M3）

**当前状态：** 模型已是**多速率**——电流环 10kHz（PID 块）、速度环 1kHz（PID_speed 块 SampleTime=1e-3）。仿真中 Simulink 自动处理跨速率交接（1kHz→10kHz 的分频保持逻辑，等价固件 N=10）。测试台在 plant↔FOC 交界加了 **Rate Transition** 保证数据完整性。

**M3 目标：** 配置为**导出函数**（Export functions），生成多个独立 step 函数：

| 函数 | 采样率 | 内容 |
|------|--------|------|
| `CurrentLoop_step()` | 10kHz | 电流环 PI、死区补偿、反 Park、圆形限幅、SVPWM |
| `SpeedLoop_step()` | 1kHz | 速度环 PI（或 LESO）、齿槽前馈、iq_ref 计算 |
| `PositionLoop_step()`（后续） | 1kHz 或更低 | 位置环 → ref_speed |

- **跨函数交接**（iq_ref、速度反馈等）为模型内部共享状态或固件 volatile 全局，固件不触碰算法内部。
- **由哪个中断调用哪个函数完全由固件决定**，模型无需改动：
  - 布置一（现状/过渡）：都在电流采样中断内顺序调用（先 SpeedLoop 后 CurrentLoop）。
  - 布置二（目标）：电流采样中断只跑 `CurrentLoop_step`；另一个定时器（如 TIM3，1kHz）中断跑 `SpeedLoop_step`/`PositionLoop_step`。
- 跨 ISR 共享遵循**单生产者/单消费者**（如 `iq_ref` 由速度函数写、电流函数读），用 volatile 全局即可（ARM 32 位对齐写读是原子的）。
- 生成代码中 Simulink 自动产生分频逻辑（1kHz 块每 10 个 10kHz 调用执行一次），**正好对应固件 `speed_pid.update_count` N=10**。

### 3.6 可扩展性：控制器子系统可替换 + 前馈接口

为保证后续新增算法（前馈、LESO 自抗扰等）无需改固件接口，电流环与速度环各自封装为**固定输入/输出接口的子系统**：

- **电流环子系统**：输入 `iq_ref、id_ref、id、iq、[电压前馈 vdff/vqff]`，输出 `vd、vq`
- **速度环子系统**：输入 `ref_speed、speed_fbk、[加速度前馈]`，输出 `iq_ref`

新增算法时只需替换子系统**内部**实现，接口与固件不变：

- 电流环前馈：反电动势/电压前馈（Kt·ω 等，参考项目 `Gain_current_*_kff`），可调增益 + 输入端口
- 速度环前馈：加速度前馈（J·dω/dt）
- 速度环 LESO 自抗扰：子系统内部用 ESO + 扰动补偿替换/叠加 PI，参数（ESO b0、带宽）作为可调参数加入 `MotorCfg`（参考项目已有 `FOC_LARDC_Params_t`）

### 3.7 建模方式：基于库搭建（实际落地）

Motor Control Blockset 25.2 的库块已实际使用，并**用探针逐块验证约定与固件一致**：

| 层级 | 实际用法 | 验证 |
|------|----------|------|
| 被控对象/测试台 | MCB **Surface Mount PMSM** + **Average-Value Inverter**（测试台 plant）| 探针确认端口/接口；闭环仿真通过 |
| 标准算法原语 | MCB **Clarke/Park/反Park 变换** + Simulink **PID Controller 块** | 探针数值与固件公式完全一致（iα/iβ/id/iq/vα/vβ 逐项比对）|
| 固件关键算法 | **SVPWM**、**VoltageLimiter（圆形限幅）** 自封装子系统 | 逻辑=固件 Foc_Svpwm / RevPark_Circle_Limitation |

**约束（已实际遵守）：**
- MCB DQ Limiter 与固件圆形限幅约定不符（实测未限幅）→ 未用，自封装 `VoltageLimiter`。
- PID Controller 块选用**并联型 + UseI*Ts + 反算抗饱和**；电流环增益映射固件公式（I 直接=固件 ki）。
- SVPWM 用基础块精确复现 `Foc_Svpwm`（七段式马鞍波+零序注入）。
- 验证过的子系统（CurrentController/SVPWM/VoltageLimiter/DeadTimeComp）**尚未存入 `Simulink_Model/library/`**（该目录为空，计划后续固化）。

---

## 4. 参数由代码给定（可调参数机制）

### 4.1 参数形态：导出一个全局配置结构体

所有需要固件运行时控制的量，在模型中一律定义为**可调参数对象**，不写死为常量。

**当前实际状态：** 参数为**独立 Simulink.Parameter + ExportedGlobal**（经 `scripts/motor_params.m` 集中定义 + 模型 InitFcn 自动加载），生成代码侧为多个全局变量（`CurrQ_Kp`、`DeadComp_En` 等）。

**M3 目标：** 用**参数总线（Simulink.Bus）/自定义存储类**打包为**单个全局配置结构体** `MotorCfg`，固件只需填一个结构体：

| 参数类别 | 生成代码形态（M3 目标） |
|----------|--------------|
| 电机参数 R、Ld/Lq、Flux、Pole_Pairs、J 等 | `MotorCfg.Motor_R` …（结构体成员） |
| 三环 Kp/Ki/限幅 | `MotorCfg.Gain_*` |
| Vbus、max_modulation、PWM_PERIOD、FOC 频率、分频 N | `MotorCfg.*` |
| 电流/速度 LPF 系数、死区淡出/增益系数 | `MotorCfg.*` |
| **死区补偿 LUT**（断点数组 + 表格数组） | `MotorCfg.DeadComp_Lut_I[20]`、`MotorCfg.DeadComp_Lut_V[20]` |
| **齿槽前馈 LUT**（角度断点 + iq 表） | `MotorCfg.Cogging_Lut_*[N]` |
| **前馈使能标志** | `MotorCfg.DeadComp_En`、`MotorCfg.CoggingFF_En` |

> **表格就"放在代码里"**：Simulink 的 Lookup Table 模块读取这些**全局数组**，而数组的定义、填充、Flash 持久化全部由固件负责。LUT 网格点数在代码生成时固定（如 20 点），数值完全运行时可写——换电机/重新标定后固件覆盖数组即可，**无需重新生成代码**。

### 4.2 使能/禁用（死区补偿、齿槽前馈）

模型内用**可调使能参数**（uint8/boolean，导出为全局）驱动 Switch 模块做前馈通断，**不改生成代码**：

```c
/* 编译期宏（固件侧） */
#define USE_DEAD_COMP   1
#define USE_COGGING_FF  0

/* 初始化（固件侧） */
MotorCfg.DeadComp_En = (USE_DEAD_COMP) && Driver_Params.DeadComp_Calibrated;
MotorCfg.CoggingFF_En = (USE_COGGING_FF) && Cogging_Calibrated;

/* 运行时可调：调试/测试时直接改写 MotorCfg.DeadComp_En */
```

既满足"宏定义编译期开关"，又保留"运行时灵活切换"，与参考项目 `DeadComp_FF_Enable()` 用法一致。

### 4.3 固件侧写入逻辑（手写代码）

1. 上电从 Flash 加载 `Motor_Config_t` + `Driver_Params_t`（参考项目结构，含电机参数、增益、死区标定表、转矩-电流标定表）。
2. 填充模型导出的全局配置结构体 `MotorCfg`。
3. 三环增益按带宽公式（如 `speed_pid_init`：`Kp = SpeedLoop_Bandwitch·2π·J/(1.5·P·Flux)`）或配置表写入。
4. 在线/离线辨识（`Para_identify.c` / `Online_Para_Identify.c`，待移植）结果回写电机参数与死区标定表。
5. 切换电机 = 换一套 `Motor_Config` 重新填充 `MotorCfg`，不重新编译算法。

> 注：生成代码侧导出的全局变量/结构体需定义一次（固件侧提供定义 + 初值），Simulink 生成头文件中以 `extern` 声明。

---

## 5. 代码生成配置

- **目标**：Embedded Coder（`ert.tlc`）
- **求解器**：离散、定步长，固定步长 `1/FOC_Frequency`（100µs）
- **数据类型**：single（float32），与现有 `arm_math` 一致；SVPWM 内可用 single
- **代码接口打包**：**导出函数（Export functions）**，生成多个独立 step 函数：`CurrentLoop_step()`（10kHz）、`SpeedLoop_step()`（1kHz）、后续 `PositionLoop_step()`。固件决定各函数由哪个中断调用。
- **参数存储**：全部 Tunable + ExportedGlobal
- **生成物**：`CyberDog_Motor_FOC.c / .h / _private.h / _types.h / rtwtypes.h` 等
- 生成后拷贝到 `Simulink_Model/generated/`（构建产物 `slprj/` 不入库）

---

## 6. 固件集成

**对接点：** 现有 `Code_Source/APP/FOC_run.c` 的 10kHz ADC 中断回调。

改造后 ISR 流程（以"双中断"目标布置为例）：
```
【电流采样中断 10kHz】
  ADC 采样 + 标定（手写）→ 角度测量 + PLL（手写）
  → 填充电流环输入（ia, ib, theta_elec）
  → CurrentLoop_step(...)               ← 生成的电流环函数
  → 取 duty_u/v/w → TIM1->CCR1/2/3 写入（手写）

【速度/位置定时器中断 1kHz（如 TIM3）】
  → 读速度反馈（由角度估计更新）
  → 填充输入（ref_speed, speed_fbk, ctrl_mode）
  → SpeedLoop_step(...)                 ← 生成的外环函数（速度/位置）
  → iq_ref 交接给电流环（模型内部状态或 volatile 全局）
```

**两种 ISR 布置均支持（模型结构相同，切换布置无需重新生成代码）：**
- 布置一（现状/过渡）：单中断——电流采样中断内顺序调用 `SpeedLoop_step` → `CurrentLoop_step`
- 布置二（目标）：电流采样中断只跑电流环；另一个定时器中断跑速度/位置环

跨 ISR 共享（`iq_ref`、`speed_fbk`）遵循单生产者/单消费者，用 volatile 全局或模型内部状态承担。

**CMake 集成：**
- 生成的 `.c` 文件添加到 `CMakeLists.txt` 的 `target_sources`（`Simulink_Model/generated/`，不在 `Code_Source` 的 GLOB 范围内，需显式加入）。
- 生成的 `.h` 路径加入 `target_include_directories`。
- 注意与现有手写 FOC 的**平滑替换**：v1 阶段可保留手写实现做 A/B 对比，用编译宏切换。

**架构约束：** 生成的算法函数必须**无静态状态残留于 I/O 层之外**、可由固件提供参数的纯算法函数；内部状态（PI 累加、计数器）封装在生成代码内部，固件不触碰。

**参数初始化时序：** 上电加载 Flash 配置（`Motor_Config` + `Driver_Params`）→ 填充全局配置结构体 `MotorCfg` → 置位使能标志（按宏 + 标定状态）→ 再启动 FOC。

---

## 7. 验证计划

| 阶段 | 方法 | 验收标准 |
|------|------|----------|
| 模型级 | Simulink Test 单元测试（阶跃/正弦响应） | 电流环带宽~500Hz、速度环~50Hz 目标达成 |
| 闭环仿真 | 电机 + 逆变器 Plant 模型（Simscape/数学模型）连模型闭环 | 与手写实现的响应一致 |
| 代码级 | 生成代码后 SIL 测试（生成代码跑同样 testbench） | SIL 与模型结果一致 |
| 硬件级 | 与现有手写 FOC 在相同台架 A/B 对比（电流/转速波形） | 关键指标一致（≤5% 偏差） |
| 参数注入 | 运行时改写全局参数（换电机参数） | 无需重编译，参数生效 |

---

## 8. 文件夹结构

```
Simulink_Model/
├── CyberDog_Motor_FOC.slx        ← 算法模型（代码生成源，纳入版本控制）
├── CyberDog_Motor_FOC_Test.slx   ← 测试台（MCB PMSM plant + 交互输入/观测）
├── library/                      ← 自定义复用库（暂空，待固化子系统）
├── scripts/
│   ├── motor_params.m            ← 【核心】全部可调参数定义（+ 电流环增益固件公式推导），设为 InitFcn
│   ├── m1_simtest.m              ← M1 骨架验证（duty 输出）
│   ├── m2_simtest.m              ← M2 闭环验证（速度/死区/齿槽通断）
│   └── sim_scenarios.m           ← 4 工况仿真（电流环/电流+速度环 × 空载/负载 + 绘图）
├── generated/                    ← 生成的 C 代码副本（M3 从 *_ert_rtw 拷入，暂空）
├── backups/                      ← 重构前模型备份
└── slprj/                        ← 构建中间产物（.gitignore 忽略）

Docs/
└── Simulink_Model_Plan.md        ← 本计划
```

---

## 9. 里程碑

| 里程碑 | 内容 | 完成标志 |
|--------|------|----------|
| M1 | 建立模型骨架（I/O 接口 + 电流环 + SVPWM） | 模型可仿真，输出 duty ✅ **已完成** |
| M2 | 加入速度环（1kHz）+ 死区补偿/齿槽前馈 LUT + MCB plant 闭环 + **库块化重构**（MCB 变换/PID 块/子系统分层）+ 交互测试台 | 闭环仿真通过，前馈通断生效，交互点 Run 可用 ✅ **已完成** |
| M3 | 代码生成 + 固件集成（导出函数调用 + ISR 布置 + CMake + 全局配置结构体 MotorCfg + sldd 迁移） | 台架电流/速度闭环运行 ⏳ **进行中** |
| M4 | A/B 对比验证 + 参数动态切换（含前馈开关） | 验收标准达成 |
| M5（后续） | 位置环 + 模式切换入模 | — |
| M6（参考项目移植） | 引入 `AC_Control_400W_New` 的在线/离线辨识、转矩-电流标定表、Flash 持久化等 | 移植功能可用，接入 `MotorCfg` |

---

## 附：现有代码参考对照

| Simulink 模块/环节 | 对应现有代码 |
|-------------------|--------------|
| Clark | `MID_foc/Axis_transform.c` / `foc.c` `Clark()` |
| Park / 反 Park | `foc.c` `Park()` / `Reverse_Park()` |
| 圆形限幅 | `foc.c` `RevPark_Circle_Limitation_cmsis()` |
| SVPWM | `foc.c` `Foc_Svpwm()` |
| 电流环 PI | `foc_pid.c` `current_pid_Control()` |
| 速度环 PI | `speed_pid.c` `Speed_pid_c()` / `speed_Loop_Handle()` |
| 电流 LPF | `foc_pid.h` `Lpf` 宏（FOC_run.c 中使用，系数 0.8） |
| 死区补偿前馈（目标） | 参考项目 `AC_Control_400W_New/AND_Source/MID/MID_foc/dead_comp_ff.c`（1-D 电流→电压 LUT + dq 矢量投影） |
| 齿槽转矩前馈（后续） | 1-D 角度→iq LUT，机制同死区前馈（模型内） |
| 参数结构 | `foc.h` `Motor_Params_t`、`foc_pid.h`、`speed_pid.h` |
| 多电机配置 / Flash 持久化（待移植） | 参考项目 `BSP/Params_Config.c/h`（`Motor_Config_t` + `Driver_Params_t` + FlashDB Record） |
| 在线/离线辨识（待移植） | 参考项目 `MID/MID_foc/Online_Para_Identify.c`、`Para_identify.c` |
