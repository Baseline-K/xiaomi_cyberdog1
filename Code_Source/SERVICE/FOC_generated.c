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
#include "motor_command_snapshot.h"
#include "foc.h"
#include "FOC_run.h"
#include "AS5600.h"               /* Encoder_AS5600.eleangle */
#include "identify.h"             /* g_ident_eleangle_ovr（极对数辨识开环电角覆盖） */
#include <math.h>

/* FOC_run.c 的全局量（未在头文件声明，此处 extern） */
extern float Predict_eleangle;

#define GEN_DEAD_COMP   0   /* 死区补偿使能 */
#define GEN_COGGING_FF  0   /* 齿槽前馈使能（默认关） */

/* 电流环带宽 / 增益系数（与固件 current_q_pid_Init 一致） */
#define GEN_CURR_BW     400.0f
#define GEN_CURR_KF     6.18f
#define ONE_BY_SQRT3    0.57735027f
#define TWO_PI_F        6.283185307f

void FOC_Generated_Init(void)
{
    /* ---- 电流环增益：固件公式（UseI*Ts 约定，模型 I 增益直接=固件 ki） ---- */
    CurrQ_Kp = Motor_Params.Lq * GEN_CURR_BW * GEN_CURR_KF;   /* q 轴用 Lq */
    CurrQ_Ki = Motor_Params.Phase_R * GEN_CURR_BW * GEN_CURR_KF * 1e-4f;
    CurrD_Kp = Motor_Params.Ld * GEN_CURR_BW * GEN_CURR_KF;   /* d 轴用 Ld */
    CurrD_Ki = CurrQ_Ki;
    Curr_MaxOut = Motor_Params.VBUS * ONE_BY_SQRT3;   /* q 轴 ±VBUS/√3 */
    Curr_MinOut = -Curr_MaxOut;
    CurrD_MaxOut = Motor_Params.VBUS * ONE_BY_SQRT3;  /* d 轴与 q 轴一致 ±VBUS/√3（原 ±3V 限死 d-PI 抵消耦合的能力） */
    CurrD_MinOut = -CurrD_MaxOut;

    /* ---- 母线电压 / 调制度（模型用 VmaxCoeff/InvVbus 两个可调全局） ---- */
    VmaxCoeff = Motor_Params.VBUS * ONE_BY_SQRT3 * 0.95f;   /* = Vbus/√3·MaxMod */
    InvVbus   = 1.0f / Motor_Params.VBUS;

    /* ---- 速度环增益：频域整定（用户公式 + 2π，模型速度环工作在 RPS 域 A/RPS）
     *   Kp = 2π·J·ωc·sinφm / Kt            （A/RPS）
     *   Ki_block = 2π·J·ωc²·cosφm / Kt × 1e-3  （速度环 1kHz UseI*Ts）
     *   ωc=2π·50Hz≈314.16 rad/s、φm=60°；J/Kt 来自惯量/磁链辨识（FOC_Generated_Init 每次调用重算） ---- */
    {
        float kt_spd = (Motor_Params.Kt > 1e-6f) ? Motor_Params.Kt
                                                 : (1.5f * Motor_Params.Pole_Pairs * Motor_Params.Flux);
        const float wc = 314.159265f;          /* 2π·50Hz */
        const float sin_pm = 0.8660254f;       /* sin60° */
        const float cos_pm = 0.5f;             /* cos60° */
        Speed_Kp = TWO_PI_F * Motor_Params.Rotor_inertia * wc * sin_pm / kt_spd;
        Speed_Ki = TWO_PI_F * Motor_Params.Rotor_inertia * wc * wc * cos_pm / kt_spd * 1e-3f;
    }
    Speed_MaxOut = 5.0f;
    Speed_MinOut = -5.0f;

    /* ---- 位置环增益（PD 结构，先只给 P 项；D 预留，仿真整定后直接写值即可） ---- */
    Pos_Kp     = 10.0f;    /* 位置环比例增益 RPS/rad */
    Pos_Kd     = 0.0f;     /* 微分预留 */
    Pos_MaxOut = 5.0f;     /* 位置环输出上限(RPS, 作速度环参考) */
    Pos_MinOut = -5.0f;

    /* ---- 死区补偿 LUT（离线辨识标定，2026-08；高电流段平台限幅 0.15V）
     * 注：辨识时母线电压 13V、最大电流 1.5A，与 MOS 管正常工作电压/电流范围不完全一致，
     *     辨识结果仅供参考。
     * 电流断点（A）0~1.5 密低疏高；Vcomp：过零区先升后稳、高电流段限幅 0.15V（只补死区+少量余量）。 */
    static const float DEAD_LUT_I[20] = {
        0.00f, 0.05f, 0.10f, 0.15f, 0.20f,
        0.25f, 0.30f, 0.35f, 0.40f, 0.50f,
        0.60f, 0.70f, 0.80f, 0.90f, 1.00f,
        1.10f, 1.20f, 1.30f, 1.40f, 1.50f
    };
    static const float DEAD_LUT_V[20] = {
        0.0000f, 0.0797f, 0.0970f, 0.1100f, 0.1190f,
        0.1233f, 0.1254f, 0.1257f, 0.1226f, 0.1152f,
        0.1107f, 0.1045f, 0.1062f, 0.1139f, 0.1304f,
        0.1500f, 0.1500f, 0.1500f, 0.1500f, 0.1500f
    };
    DeadComp_En = GEN_DEAD_COMP;
    for (int i = 0; i < 20; i++) {
        DeadComp_Lut_I[i] = DEAD_LUT_I[i];
        DeadComp_Lut_V[i] = DEAD_LUT_V[i];
    }

    /* ---- 齿槽前馈 LUT（默认关，占位数据，360 点电角度每度） ---- */
    CoggingFF_En = GEN_COGGING_FF;
    for (int i = 0; i < 360; i++) {
        Cogging_Lut_Angle[i] = i * TWO_PI_F / 359.0f;
        Cogging_Lut_V[i] = 0.0f;    /* 占位；齿槽辨识后写入 */
    }

    /* ---- PLL 速度换算：电角速度 rad/s -> 机械 RPS (1/(2π·极对数)) ---- */
    /* PLL 增益 Kp/Ki 为 Simulink 掩码构建期常量(MCB 块不可调), 改带宽需重新生成代码 */
    InvTwoPiPolePairs = 1.0f / (TWO_PI_F * Motor_Params.Pole_Pairs);

    CyberDog_Motor_FOC_initialize();
}

/* 启动时清零控制环积分器与饱和输出（防上次运行残留导致的瞬间过冲）
 * 注：只清控制环状态；PLL 角度积分器(Integrator_DSTATE)保持跟踪，不清。 */
void FOC_Generated_Reset(void)
{
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_l = 0.0f;   /* d 电流积分 */
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_m = 0.0f;   /* 速度环积分 */
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_k = 0.0f;   /* q 电流积分 */
    CyberDog_Motor_FOC_DW.Filter_DSTATE      = 0.0f;    /* 位置环 D 滤波(预留) */
    CyberDog_Motor_FOC_B.Saturation          = 0.0f;    /* 位置环输出(饱和后) */
    CyberDog_Motor_FOC_B.Saturation_f        = 0.0f;    /* 速度环输出(饱和后) */
}

void FOC_Generated_Step(void)
{
    /* ---- 母线电压实时化：模型 SVPWM/限幅/电流限随实测母线（Motor_Params.VBUS 每 ISR 由 FOC_run.c 更新） ---- */
    VmaxCoeff   = Motor_Params.VBUS * ONE_BY_SQRT3 * 0.95f;
    InvVbus     = 1.0f / Motor_Params.VBUS;
    Curr_MaxOut  = Motor_Params.VBUS * ONE_BY_SQRT3;
    Curr_MinOut  = -Curr_MaxOut;
    CurrD_MaxOut = Curr_MaxOut;   /* d 轴与 q 轴一致（辨识 d-PI 抵消耦合需更大权限） */
    CurrD_MinOut = -CurrD_MaxOut;

    /* ---- 填充模型输入（来自命令快照 + 编码器） ---- */
    const MotorCommand_t *cmd = MotorCommand_Get();

    CyberDog_Motor_FOC_U.ia         = foc_abc_current_i.ia;
    CyberDog_Motor_FOC_U.ib         = foc_abc_current_i.ib;
    if (g_ident_eleangle_ovr) {
        CyberDog_Motor_FOC_U.eleangle = g_ident_eleangle;   /* 极对数辨识：受控开环电角度（反 Park 用） */
    } else {
        CyberDog_Motor_FOC_U.eleangle = Encoder_AS5600.eleangle;   /* 原始电角度 rad, PLL 在模型内部 */
    }
    CyberDog_Motor_FOC_U.pll_reset  = 0.0f;   /* PLL 常跟踪（coast 也跟踪，保证转速实时） */
    CyberDog_Motor_FOC_U.id_ref     = 0.0f;   /* 辨识也要 id=0 */
    if (MotorState.run_state != RUNSTATE_IDENTIFYING) {
        /* 正常运行时由命令映射；辨识时已由 Identify_FocIsrStep 填好
         * （电压模式 ctrl_mode=3/4 填 vd/vq_ref；J 转矩模式 ctrl_mode=0 填 iq_ref——若这里覆盖会丢失） */
        CyberDog_Motor_FOC_U.iq_ref     = cmd->iq_ref_A;     /* 转矩模式目标 */
        CyberDog_Motor_FOC_U.ref_speed  = cmd->speed_rps;    /* 速度模式目标 */
        CyberDog_Motor_FOC_U.ctrl_mode  = (cmd->mode == MC_MODE_POSITION) ? 2.0f :
                                          (cmd->mode == MC_MODE_SPEED)    ? 1.0f : 0.0f;
        CyberDog_Motor_FOC_U.pos_ref    = cmd->pos_ref;   /* rad, 连续，位置模式目标 */
        CyberDog_Motor_FOC_U.pos_fbk    = g_mech_pos_rad; /* rad, 连续，10kHz ISR 累加 */
        CyberDog_Motor_FOC_U.coast      = cmd->coast ? 1.0f : 0.0f;  /* 模型内 duty 强制 0.5（coast） */
    }

    /* ---- 始终 step：PLL 实时给 speed_meas_rps；coast 时模型输出 duty=0.5 ---- */
    CyberDog_Motor_FOC_step();

    /* 回写 PLL 输出到固件全局（供其他模块/legacy 路径读取） */
    Predict_eleangle = CyberDog_Motor_FOC_Y.theta_elec_filt;                   /* 滤波后电角度 rad */
    Encoder_PLL_eleFilter->PLL_Omega_filtered =                               /* 电角速度 rad/s */
        CyberDog_Motor_FOC_Y.speed_meas_rps * (TWO_PI_F * Motor_Params.Pole_Pairs);

    TIM1->CCR1 = (uint16_t)(CyberDog_Motor_FOC_Y.duty_u * PWM_PERIOD);
    TIM1->CCR2 = (uint16_t)(CyberDog_Motor_FOC_Y.duty_v * PWM_PERIOD);
    TIM1->CCR3 = (uint16_t)(CyberDog_Motor_FOC_Y.duty_w * PWM_PERIOD);
}

float FOC_Generated_GetSpeedRps(void)
{
    return CyberDog_Motor_FOC_Y.speed_meas_rps;   /* 机械转速 RPS（模型 PLL 输出） */
}
