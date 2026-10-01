/**
 * MotorCtrl —— 电机控制层（SERVICE，同参考项目 controller.c）
 *
 * 影子→commit-on-start：CAN 写 *_temp 影子不生效，
 * 只有 MotorCtrl_Start()（0x6040=0x1F）一次性提交全部目标并使能。
 *
 * 统一结构体：CAN（写影子+Start 提交）与 J-Link（直接改命令字段）都作用于 MotorCtrl，
 * FOC ISR 读取 MotorCtrl 的命令字段。
 *
 * FOC sampling and power-stage output are controlled independently.
 */
#include "MotorCtrl.h"
#include "FOC_generated.h"   /* FOC_Generated_Reset */
#include "FOC_run.h"         /* MotorState, CTRL_MODE_* */
#include "main_user.h"       /* Param_init/Param_deinit */
#include "drv83xx.h"         /* DRV83xx COAST control */
#include "motor_state_machine.h"  /* MotorStateMachine_PostFault / GetFaultBits */
#include "Safety_Module.h"        /* FAULT_GATE_DRIVER_MASK */
#include "main.h"                 /* DRV8232_nFAULT_Pin / GPIO_PIN_x */
#include "tim.h"                  /* htim1 */

extern ADC_HandleTypeDef hadc1, hadc2, hadc3;

/* ================= FOC interrupt / power-stage control ================= */

void FOC_Interrupt_Start(void)
{
    /* 职责 = 参考工程的 PowerStage_StartControlLoop()：只负责"采样链路"。
     * 分配（2026-09-27 定稿）：
     *   ADC3 注入 rank1/rank2 = 电流C / Vbus  → JEOC = FOC ISR 源（唯一中断）
     *   ADC2 注入 rank1       = 电流B          → 无中断，ISR 里轮询 JDR1
     *   ADC1 规则 6 通道      = 输出端霍尔      → 连续 + DMA，与电流路径无交集 */
    HAL_ADCEx_InjectedStart(&hadc2);        /* 电流B：无中断 */
    HAL_ADCEx_InjectedStart_IT(&hadc3);     /* 电流C + Vbus：JEOC 触发 FOC ISR */
    (void)HAL_TIM_Base_Start(&htim1);       /* 计数器跑起来（**不带 IT**，参考工程同款） */
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  /* 使能 CC4E；触发实际走 TIM1_TRGO */
}

/* 采样触发链路（2026-09-27 修正，照参考工程 AND_Source/BSP/Power_Stage_TIM1_DFSDM.c）：
 *     TIM1(CH4, PWM Generation4 **No Output**, Pulse=PWM_ADC_TRGO)
 *       -> OC4REF（输出控制级**之前**的内部比较参考信号）
 *       -> TIM1_TRGO（tim.c 里 MasterOutputTrigger = TIM_TRGO_OC4REF）
 *       -> ADC1/2/3 注入组外部触发（adc.c 里 ExternalTrigInjecConv = T1_TRGO）
 *
 * 为什么不直接用 TIM1_CC4：
 *   CC4 事件会被输出控制级（MOE）gate 掉 —— MOE=0 时 CC4 不再送到 ADC。
 *   而 PowerStage_Disable() 会清 MOE（HAL_TIM_PWM_Stop 的条件宏），
 *   于是"关输出"就把整条采样链路一起关了：转速冻结、编码器停更、
 *   Safety_FastStep 失效、S_FAULT_OVER 因为拿不到速度而死锁。
 *   实测症状就是"INIT 里进过一次 FOC 中断，进 IDLE 后再没进过"。
 *   OC4REF 在 MOE 之前产生，换过去之后 MOE/CC4E 怎么动都不影响采样。
 *   触发时刻完全不变：仍是 CNT 上计数穿过 CCR4(=PWM_ADC_TRGO) 那一次。 */

void FOC_Interrupt_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStop_IT(&hadc3);
    HAL_ADCEx_InjectedStop(&hadc2);
}

void PowerStage_Enable(void)
{
    /* ===== 故障闸门（必须放在最前面）=====
     * 防的是这个 ≤1ms 竞态：本函数执行到一半，被更高优先级的关断路径抢占
     *   - EXTI nFAULT ISR（NVIC 优先级 0）
     *   - SafetyTask 任务看门狗（任务优先级 5）
     * 并被它们关断功率；随后本函数继续执行，又把功率打开了。
     * 这两条路径都是"先 PostFault 锁存、再关功率"，所以这里读到的一定是最新值。
     *
     * 不会把自己锁死：状态机离开 FAULT_NOW 时（state_FAULT_exit）会清 g_fault_bits，
     * 恢复流程走完就能正常使能。 */
    if (MotorStateMachine_GetFaultBits() != 0U) {
        return;
    }

    /* Keep zero line-to-line voltage until the first RUNNING FOC update. */
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_HalfPerMax;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    DRV8323_COAST_Disable(&stru_DRV8323Obj);

    /* HAL_TIM_PWM_Start 会置 CEN（这里需要它），顺便读回确认 —— 没有 CEN 就没有
     * CC4 触发、也就没有 FOC 中断。参考工程同样在 EnableOutput 里起通道。 */
    if ((TIM1->CR1 & TIM_CR1_CEN) == 0U) {
        printf("[PWR] ERROR: TIM1 CEN not set in PowerStage_Enable -> FOC ISR will not start\r\n");
    }
}

void PowerStage_Disable(void)
{
    uint32_t tim1_cr1_backup;

    /* 先让 DRV 把六个 MOSFET 全关（COAST 位与 INHx/INLx 电平无关，是权威关断）。 */
    DRV8323_COAST_enable(&stru_DRV8323Obj);

    /* ===== 关输出但**必须**保住 TIM1 计数器 =====
     * F1 HAL 的 HAL_TIM_PWM_Stop() 最后一句是 __HAL_TIM_DISABLE(htim)
     * （清 TIM1->CR1.CEN），会把整个 TIM1 停掉。而 FOC 中断的触发链是
     * TIM1 -> OC4REF -> TRGO -> ADC 注入，停掉 TIM1 就等于 FOC 中断永久消失，
     * 连带转速反馈、编码器刷新、Safety_FastStep 全部停摆，S_FAULT_OVER 也会
     * 因为拿不到速度而死锁。
     *
     * 写法照抄参考工程 AND_Source/BSP/Power_Stage_TIM1_DFSDM.c 的
     * PowerStage_DisableOutput()：
     *   备份 CR1 → 用 HAL 关三个通道（顺带正确处理 CCxE/MOE）→ 恢复 CR1。
     * 恢复 CR1 把 HAL 的清 CEN 副作用抵消掉，计数器继续跑、TRGO 继续触发采样。
     *
     * 职责边界（与参考工程一致，别再越界）：
     *   本函数只关 CH1/CH2/CH3；
     *   采样链路（Base + CH4 + TRGO）由 FOC_Interrupt_Start/Stop 负责，不在这里碰。 */
    tim1_cr1_backup = TIM1->CR1;
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    TIM1->CR1 = tim1_cr1_backup;

    /* 读回确认：CEN 必须还在，否则采样链路已经死了 */
    if ((TIM1->CR1 & TIM_CR1_CEN) == 0U) {
        printf("[PWR] ERROR: TIM1 CEN lost in PowerStage_Disable -> FOC ISR will stop\r\n");
    }
}

/* ================= 栅极驱动 nFAULT 诊断计数 =================
 * 供 RTT `diag` 读取，用来区分"真故障"与"上电假边沿"：
 *   real     > 0 → 引脚当时确实为低，是真故障（配合 FSR1/FSR2 看具体原因）
 *   spurious > 0 → 电平复核发现引脚已释放，被丢弃的假边沿
 * 上电稳定后两者都不应再增长。 */
volatile uint32_t g_nfault_count    = 0U;
volatile uint32_t g_nfault_spurious = 0U;
volatile uint32_t g_nfault_tick     = 0U;

/* DRV 是否已经完成初始化+配置校验。在此之前 nFAULT 不可信：
 * CubeMX 在 MX_GPIO_Init()（main() 的第一个调用）就配好 PA12 的 EXTI 并
 * 立刻使能 NVIC，而那一刻 DRV 还没供电、引脚电平未定义 ——
 * 实测上电必现一次假故障（FSR1/FSR2=0x0000 且引脚随后释放）。
 * DRV 未配置、PWM 未开的时候 nFAULT 没有任何保护意义，
 * 所以在那之前直接把这个边沿丢掉。 */
static volatile uint8_t g_drv_ready = 0U;

void MotorCtrl_DrvReady(void)
{
    /* 清掉配置期间可能已锁存的边沿，再放行后续中断 */
    __HAL_GPIO_EXTI_CLEAR_IT(DRV8232_nFAULT_Pin);
    g_drv_ready = 1U;
}

/* ================= 栅极驱动 nFAULT(PA12) 中断回调 =================
 * NVIC 优先级 0（数值最小 = 最紧急），在 Core/Src/gpio.c 里配置，
 * 向量由 Core/Src/stm32f1xx_it.c 的 EXTI15_10_IRQHandler 转到 HAL_GPIO_EXTI_IRQHandler。
 *
 * 优先级 0 < configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)，这条硬性约束决定了
 * 本回调里**绝对不能**出现：
 *   - 任何 FreeRTOS API（连 ...FromISR 都不行 → 直接触发 configASSERT 死循环）
 *   - printf / RTT
 *   - SPI 访问（DRV83XX_ReadAndClearFaults、PowerStage_Disable 内部都走阻塞 SPI）
 * 所以这里只做两件纯寄存器操作，其余交给 1ms 内必然会跑到的 MotorStateTask。
 *
 * 顺序：先 PostFault 锁存故障位，再关功率 ——
 * 这样并发的 PowerStage_Enable() 里的故障闸门一定拦得住（否则会出现
 * "ISR 刚关断、被抢占的任务又把功率打开"）。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != DRV8232_nFAULT_Pin) {
        return;
    }

    /* ===== 1) DRV 还没配置好之前，nFAULT 不可信 → 直接丢弃 =====
     * 见 g_drv_ready 的说明：CubeMX 一上电就把 PA12 EXTI 挂上并立刻使能 NVIC，
     * 而那时 DRV 未供电、引脚电平未定义，实测必现一次假故障。 */
    if (g_drv_ready == 0U) {
        g_nfault_spurious++;
        return;
    }

    /* ===== 2) 用当前电平复核这个边沿 =====
     * nFAULT 是开漏 + 上拉，**真故障**（OCP=LATCH / UVLO / OTSD / GDUV）都会把它
     * 拉低并保持，所以复核不会漏掉真故障；而"边沿已过、电平已恢复"的毛刺会被丢掉。
     * HAL_GPIO_ReadPin 是纯寄存器操作，优先级 0 的 ISR 里安全。 */
    if (HAL_GPIO_ReadPin(DRV8232_nFAULT_GPIO_Port, DRV8232_nFAULT_Pin) != GPIO_PIN_RESET) {
        g_nfault_spurious++;
        return;
    }

    g_nfault_count++;
    g_nfault_tick = HAL_GetTick();   /* HAL_GetTick 只读变量，ISR 里安全 */

    MotorStateMachine_PostFault(FAULT_GATE_DRIVER_MASK);   /* 1) 先锁存 */
    PowerStage_MoeOff();                                    /* 2) 再硬件级关断（无条件清 MOE） */

    /* 这里**不**读 FSR1/FSR2，两个原因：
     *   1) 读寄存器是阻塞 SPI，在优先级 0 的 ISR 里做会拖长中断，还可能和任务上下文抢 SPI2；
     *   2) 读操作会清掉 DRV 内部的故障锁存 —— 那是现场证据，留给
     *      state_FAULT_do 里调用 DRV83XX_ReadAndClearFaults() 解码打印。
     *
     * 残留风险说明：任务上下文里的 HAL_TIM_PWM_Stop 也可能在写 BDTR 的过程中被本 ISR
     * 插入（读-改-写竞态），理论上可能把 MOE 又置回去。这一层由 DRV 自己兜住：
     * OCP_MODE=LATCH 时 DRV 会自己锁死栅极直到读 FSR1/FSR2，
     * 所以"MCU 侧 MOE 状态"不是最后一道防线。 */
}

/* ================= 统一控制结构体 ================= */

volatile MotorCtrl_t MotorCtrl;

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void MotorCtrl_Init(void)
{
    /* 默认速度模式、speed=0、enable=0（安全：不启动则电机不动） */
    MotorCtrl.mode_temp      = MC_MODE_SPEED;
    MotorCtrl.speed_rps_temp = 0.0f;
    MotorCtrl.iq_ref_A_temp  = 0.0f;
    MotorCtrl.pos_ref_temp   = 0.0f;

    MotorCtrl.enable    = 0U;
    MotorCtrl.mode      = MC_MODE_SPEED;
    MotorCtrl.speed_rps = 0.0f;
    MotorCtrl.iq_ref_A  = 0.0f;
    MotorCtrl.pos_ref   = 0.0f;

    MotorCtrl.state  = MC_STATE_STOPPED;
    MotorCtrl.faults = 0U;

    MotorState.ctrl_mode = MT_SPEED;
    MotorState.run_state = RUNSTATE_STOPPED;
}

void MotorCtrl_SetMode(uint8_t m)
{
    if (m <= MC_MODE_POSITION) MotorCtrl.mode_temp = m;
}
void MotorCtrl_SetSpeedRPS(float r) { MotorCtrl.speed_rps_temp = r; }
void MotorCtrl_SetTorqueA(float a)  { MotorCtrl.iq_ref_A_temp  = a; }
void MotorCtrl_SetPosition(float p) { MotorCtrl.pos_ref_temp   = p; }

void MotorCtrl_Start(void)
{
    uint8_t mode = MotorCtrl.mode_temp;
    float sp = clampf(MotorCtrl.speed_rps_temp, -MOTOR_SPEED_LIMIT_RPS, MOTOR_SPEED_LIMIT_RPS);
    float iq = clampf(MotorCtrl.iq_ref_A_temp,  -MOTOR_IREF_LIMIT_A,    MOTOR_IREF_LIMIT_A);

    /* 一次性提交全部影子 → 命令字段（参考 motor_start） */
    MotorCtrl.mode      = mode;
    MotorCtrl.speed_rps = sp;
    MotorCtrl.iq_ref_A  = iq;
    MotorCtrl.pos_ref   = MotorCtrl.pos_ref_temp;

    if (MotorCtrl.enable == 0U) {
        Param_init();
        FOC_Generated_Reset();   /* 清模型积分器（防上次运行残留） */
        MotorCtrl.enable = 1U;
        PowerStage_Enable();
    }
    MotorCtrl.state  = MC_STATE_RUNNING;
    MotorState.ctrl_mode = (mode == MC_MODE_POSITION) ? MT_POSITION :
                           (mode == MC_MODE_SPEED)    ? MT_SPEED : MT_TORQUE;
    MotorState.run_state = RUNSTATE_RUNNING;
}

void MotorCtrl_Stop(void)
{
    /* Let the ISR enter its safe branch before removing gate drive. */
    MotorState.run_state = RUNSTATE_STOPPED;
    MotorCtrl.enable = 0U;
    MotorCtrl.state  = MC_STATE_STOPPED;
    PowerStage_Disable();
    Param_deinit();
}

void MotorCtrl_Process(void)
{
    /* 预留：通信超时看门狗等（v1 先空） */
}

void MotorCtrl_GetStatus(uint8_t out[4])
{
    out[0] = (uint8_t)MotorCtrl.state;
    out[1] = (uint8_t)MotorCtrl.mode;
    out[2] = (MotorCtrl.enable ? 1U : 0U);              /* 仅使能位（原与故障 bit0 别名，已修） */
    out[3] = (uint8_t)((MotorCtrl.faults >> 8) & 0xFFU);/* 故障位 8..15；完整 32 位见 EMCY 0x081 */
}
