/**
 * Simulink 生成 FOC 算法 —— 固件侧适配层（SERVICE）
 *
 * 生成的算法函数（CyberDog_Motor_FOC_step）内部已包含：
 *   速度环(1kHz) + 齿槽前馈 + 电流环(10kHz) + 死区补偿 + 反Park + 圆形限幅 + SVPWM
 * 固件只负责：采样(ADC)、算角度(PLL)、填输入（来自 MotorCtrl）、调 step、写 TIM1。
 *
 * 参数约定：固件按电机参数 + 公式计算增益，传单值给模型可调全局，
 * 模型只消费这些全局（不内联多参数表达式，保证运行时可调）。
 *
 * 控制输入来自 MotorCtrl（影子→commit）：ref_speed/iq_ref/ctrl_mode/enable。
 */
#include "FOC_generated.h"
#include "CyberDog_Motor_FOC.h"   /* 生成的算法接口 */
#include "MotorCtrl.h"
#include "foc.h"
#include "FOC_run.h"
#include "AS5600.h"               /* Encoder_AS5600.eleangle */
#include <math.h>

/* FOC_run.c 的全局量（未在头文件声明，此处 extern） */
extern float Predict_eleangle;

#define GEN_DEAD_COMP   0   /* 死区补偿使能 */
#define GEN_COGGING_FF  0   /* 齿槽前馈使能（默认关） */

/* 电流环带宽 / 增益系数（与固件 current_q_pid_Init 一致） */
#define GEN_CURR_BW     500.0f
#define GEN_CURR_KF     6.18f
#define ONE_BY_SQRT3    0.57735027f
#define TWO_PI_F        6.283185307f

void FOC_Generated_Init(void)
{
    /* ---- 电流环增益：固件公式（UseI*Ts 约定，模型 I 增益直接=固件 ki） ---- */
    CurrQ_Kp = Motor_Params.Phase_L * GEN_CURR_BW * GEN_CURR_KF;
    CurrQ_Ki = Motor_Params.Phase_R * GEN_CURR_BW * GEN_CURR_KF * 1e-4f;
    CurrD_Kp = CurrQ_Kp;
    CurrD_Ki = CurrQ_Ki;
    Curr_MaxOut = Motor_Params.VBUS * ONE_BY_SQRT3;   /* q 轴 ±VBUS/√3 */
    Curr_MinOut = -Curr_MaxOut;
    CurrD_MaxOut = 3.0f;                               /* d 轴按固件 ±3V */
    CurrD_MinOut = -3.0f;

    /* ---- 母线电压 / 调制度（模型用 VmaxCoeff/InvVbus 两个可调全局） ---- */
    VmaxCoeff = Motor_Params.VBUS * ONE_BY_SQRT3 * 0.95f;   /* = Vbus/√3·MaxMod */
    InvVbus   = 1.0f / Motor_Params.VBUS;

    /* ---- 速度环增益（手动整定，占位；换真实电机后重新整定） ---- */
    Speed_Kp = 0.3f;
    Speed_Ki = 0.0006f;
    Speed_MaxOut = 5.0f;
    Speed_MinOut = -5.0f;

    /* ---- 死区补偿 LUT（占位标定数据；真实标定后写入） ---- */
    DeadComp_En = GEN_DEAD_COMP;
    for (int i = 0; i < 16; i++) {
        DeadComp_Lut_I[i] = i * 3.0f / 15.0f;
        DeadComp_Lut_V[i] = i * 0.6f / 15.0f;
    }

    /* ---- 齿槽前馈 LUT（默认关，占位数据） ---- */
    CoggingFF_En = GEN_COGGING_FF;
    for (int i = 0; i < 32; i++) {
        Cogging_Lut_Angle[i] = i * TWO_PI_F / 31.0f;
        Cogging_Lut_V[i] = 0.15f * sinf(2.0f * Cogging_Lut_Angle[i]);
    }

    /* ---- PLL 速度换算：电角速度 rad/s -> 机械 RPS (1/(2π·极对数)) ---- */
    /* PLL 增益 Kp/Ki 为 Simulink 掩码构建期常量(MCB 块不可调), 改带宽需重新生成代码 */
    InvTwoPiPolePairs = 1.0f / (TWO_PI_F * Motor_Params.Pole_Pairs);

    CyberDog_Motor_FOC_initialize();
}

/* 启动时清零模型积分器与饱和输出（防上次运行残留导致的瞬间过冲） */
void FOC_Generated_Reset(void)
{
    CyberDog_Motor_FOC_DW.Integrator_DSTATE   = 0.0f;   /* d 电流积分 */
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_b = 0.0f;   /* 速度环积分 */
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_k = 0.0f;   /* q 电流积分 */
    CyberDog_Motor_FOC_B.Saturation           = 0.0f;
}

void FOC_Generated_Step(void)
{
    /* ---- 填充模型输入（来自 MotorCtrl 命令 + 编码器） ---- */
    CyberDog_Motor_FOC_U.ia         = foc_abc_current_i.ia;
    CyberDog_Motor_FOC_U.ib         = foc_abc_current_i.ib;
    CyberDog_Motor_FOC_U.eleangle   = Encoder_AS5600.eleangle;   /* 原始电角度 rad, PLL 在模型内部 */
    CyberDog_Motor_FOC_U.pll_reset  = (MotorCtrl.enable) ? 0.0f : 1.0f;  /* 禁用时保持 Position=0 */
    CyberDog_Motor_FOC_U.id_ref     = 0.0f;
    CyberDog_Motor_FOC_U.iq_ref     = MotorCtrl.iq_ref_A;     /* 转矩模式目标 */
    CyberDog_Motor_FOC_U.ref_speed  = MotorCtrl.speed_rps;    /* 速度模式目标 */
    CyberDog_Motor_FOC_U.ctrl_mode  = (MotorCtrl.mode == MC_MODE_SPEED) ? 1.0f : 0.0f;

    /* ---- 启停门控：使能才跑算法，否则输出零差模电压 ---- */
    if (MotorCtrl.enable) {
        CyberDog_Motor_FOC_step();
        /* 回写 PLL 输出到固件全局（供其他模块/legacy 路径读取） */
        Predict_eleangle = CyberDog_Motor_FOC_Y.theta_elec_filt;                   /* 滤波后电角度 rad */
        Encoder_PLL_eleFilter->PLL_Omega_filtered =                               /* 电角速度 rad/s */
            CyberDog_Motor_FOC_Y.speed_meas_rps * (TWO_PI_F * Motor_Params.Pole_Pairs);
        TIM1->CCR1 = (uint16_t)(CyberDog_Motor_FOC_Y.duty_u * PWM_PERIOD);
        TIM1->CCR2 = (uint16_t)(CyberDog_Motor_FOC_Y.duty_v * PWM_PERIOD);
        TIM1->CCR3 = (uint16_t)(CyberDog_Motor_FOC_Y.duty_w * PWM_PERIOD);
    } else {
        /* 三相 50% 占空比 = 零线间电压 = 零转矩（兜底，正常由 StopPWM 已停 PWM） */
        TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_HalfPerMax;
    }
}
