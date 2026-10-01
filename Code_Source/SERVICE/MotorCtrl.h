#ifndef MOTORCTRL_H
#define MOTORCTRL_H

#include <stdint.h>
#include "tim.h"      /* htim1（紧急关断直接写 TIM1->BDTR） */

/* 工作模式（映射到生成模型的 ctrl_mode：0=转矩,1=速度,2=位置） */
typedef enum {
    MC_MODE_TORQUE = 0,
    MC_MODE_SPEED = 1,
    MC_MODE_POSITION = 2      /* 位置环（模型内级联：位置→速度→电流） */
} MotorCtrl_Mode_t;

typedef enum {
    MC_STATE_STOPPED = 0,
    MC_STATE_RUNNING = 1,
    MC_STATE_FAULT   = 2
} MotorCtrl_State_t;

/* 安全限幅（与 Motor_Params 对齐；改了电机参数记得回来核这两个数） */
#define MOTOR_SPEED_LIMIT_RPS  30.0f    /* = Motor_Params.RPM_Rating 1800 RPM（电机轴）= 30 RPS。
                                         * 也用于 PositionPlan_SetMaxVelRps（FOC_generated.c:110）。
                                         * 改前是 40 RPS（注释写的"额定 2400 RPM"已过期）。 */
#define MOTOR_IREF_LIMIT_A     40.0f    /* = Motor_Params.Current_Max，相电流峰值。
                                         * 改前是 8A —— 会成为实际瓶颈，让软件过流 48A 永远碰不到。 */

/*
 * 统一控制结构体（CAN 与 J-Link 都改这一份，FOC ISR 读命令字段）：
 *   - 影子参数 *_temp：主循环里 CAN 写，Start() 一次性提交到命令字段
 *   - 命令字段 enable/mode/speed_rps/iq_ref_A/pos_ref：ISR 读；
 *     J-Link 调试可直接改这些（实时生效），或改 *_temp 后调 Start()
 */
typedef struct {
    /* 影子参数（主循环 CAN 写） */
    uint8_t  mode_temp;
    float    speed_rps_temp;
    float    iq_ref_A_temp;
    float    pos_ref_temp;
    /* 命令（FOC ISR 读；J-Link 直接改） */
    volatile uint8_t  enable;      /* 1=驱动，0=停止 */
    volatile uint8_t  mode;        /* MotorCtrl_Mode_t */
    volatile float    speed_rps;   /* 目标速度 (RPS) */
    volatile float    iq_ref_A;    /* 目标电流 (A, 转矩模式) */
    volatile float    pos_ref;     /* 目标位置 (rad, 预留) */
    /* 状态 */
    uint8_t  state;                /* MotorCtrl_State_t */
    uint32_t faults;
} MotorCtrl_t;
extern volatile MotorCtrl_t MotorCtrl;

void MotorCtrl_Init(void);
void MotorCtrl_SetMode(uint8_t mode);
void MotorCtrl_SetSpeedRPS(float rps);
void MotorCtrl_SetTorqueA(float a);
void MotorCtrl_SetPosition(float rad);
void MotorCtrl_Start(void);         /* 提交全部影子 → 使能 */
void MotorCtrl_Stop(void);
void MotorCtrl_Process(void);       /* 主循环 1ms（预留：看门狗等） */
void MotorCtrl_GetStatus(uint8_t out[4]);

/* FOC sampling remains active while the motor is stopped. */
void FOC_Interrupt_Start(void);
void FOC_Interrupt_Stop(void);

/* Three-phase PWM and DRV83xx gate-output control. */
void PowerStage_Enable(void);
void PowerStage_Disable(void);

/* 栅极驱动 nFAULT 诊断计数（由 EXTI 回调维护，RTT `diag` 读取）：
 *   real     = 电平复核确认引脚为低的真故障次数
 *   spurious = 电平复核发现引脚已释放、被丢弃的假边沿次数
 *              （上电时 MX_GPIO_Init 配好 EXTI 就会捕到一次，属正常） */
extern volatile uint32_t g_nfault_count;
extern volatile uint32_t g_nfault_spurious;
extern volatile uint32_t g_nfault_tick;      /* 最近一次真故障的 HAL tick */

/* 在 DRV83XX_Init() + 配置校验通过之后调用，放行 nFAULT 中断。
 * 之前它一直是屏蔽的：DRV 未配置时 PA12 电平未定义，配好 EXTI 就会捕到假边沿。 */
void MotorCtrl_DrvReady(void);

/* ===== 立即无条件关断三相输出（清 TIM1 的 MOE 位）=====
 * 用于"不能等状态机"的紧急关断：EXTI nFAULT ISR、任务看门狗、FOC ISR 快检。
 *
 * !! 千万不要在这里改回 __HAL_TIM_MOE_DISABLE() !!
 *    F1 HAL 的那个宏是**带条件**的（stm32f1xx_hal_tim.h:1104）：
 *        只有 (CCER & TIM_CCER_CCxE_MASK)==0 且 (CCER & TIM_CCER_CCxNE_MASK)==0
 *        才执行 BDTR &= ~MOE;
 *    而 PWM 正在输出时 CC1E/CC2E/CC3E 恰好是 1 → 宏整体变成空操作。
 *    也就是说在 PWM 运行中调用它"紧急关断"什么都没做（反汇编已确认：展开后是
 *    两个 tst/bne 直接跳过 BDTR 写）。
 *    这正是 FOC_run.c 快检路径曾经"当场快速关 PWM"却没真正关掉的原因。
 *
 * 一次 BDTR 读-改-写，纯寄存器、无 SPI/无 RTOS，任意优先级 ISR 内均可调用。
 * 关断后要恢复必须重新走 PowerStage_Enable()（HAL_TIM_PWM_Start 会重新置 MOE），
 * 而它带故障闸门，所以只要故障位还锁着就起不来。 */
static inline void PowerStage_MoeOff(void)
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

#endif /* MOTORCTRL_H */
