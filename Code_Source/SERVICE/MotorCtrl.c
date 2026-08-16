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

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1, hadc2;

/* ================= FOC interrupt / power-stage control ================= */

void FOC_Interrupt_Start(void)
{
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  /* ADC injected trigger */
}

void FOC_Interrupt_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStop_IT(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);
}

void PowerStage_Enable(void)
{
    /* Keep zero line-to-line voltage until the first RUNNING FOC update. */
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_HalfPerMax;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    DRV8323_COAST_Disable(&stru_DRV8323Obj);
}

void PowerStage_Disable(void)
{
    /* Disable all MOSFETs first, then disconnect the timer outputs. */
    DRV8323_COAST_enable(&stru_DRV8323Obj);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
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

    MotorState.ctrl_mode = CTRL_MODE_SPEED;
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
    MotorState.ctrl_mode = (mode == MC_MODE_SPEED) ? CTRL_MODE_SPEED : CTRL_MODE_TORQUE;
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
    out[2] = (MotorCtrl.enable ? 1U : 0U) | ((uint8_t)MotorCtrl.faults & 0xFFU);
    out[3] = (uint8_t)((MotorCtrl.faults >> 8) & 0xFFU);
}
