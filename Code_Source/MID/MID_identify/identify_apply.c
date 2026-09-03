/**
 * identify_apply.c — 辨识公共执行原语：复用模型电压模式（ctrl_mode=3 dq / =4 αβ）
 *
 * 注意：辨识时 coast 必须置 0——S_CALIB 里 publish_coast 会置 coast=1，
 * 而模型根级 Sw_duty_* 在 coast≠0 时强制 duty=0.5，会覆盖电压模式的 SVPWM 输出。
 */
#include "identify.h"
#include "CyberDog_Motor_FOC.h"   /* U/Y 全局 */

/* dq 电压模式：填 vd/vq_ref + ctrl_mode=3（模型内部反 Park 用 theta_elec_filt） */
void identify_apply_voltage(float vd, float vq)
{
    CyberDog_Motor_FOC_U.vd_ref    = vd;
    CyberDog_Motor_FOC_U.vq_ref    = vq;
    CyberDog_Motor_FOC_U.ctrl_mode = 3.0f;
    CyberDog_Motor_FOC_U.coast     = 0.0f;   /* 关键：释放 coast 对 duty 的强制 0.5 */
}

/* αβ 电压模式：填 v_alpha/v_beta_ref + ctrl_mode=4（HFI 注入任意方向脉冲用） */
void identify_apply_alpha_beta(float v_alpha, float v_beta)
{
    CyberDog_Motor_FOC_U.v_alpha_ref = v_alpha;
    CyberDog_Motor_FOC_U.v_beta_ref  = v_beta;
    CyberDog_Motor_FOC_U.ctrl_mode   = 4.0f;
    CyberDog_Motor_FOC_U.coast       = 0.0f;
}

/* ---- 开环电角度覆盖（极对数/方向辨识用） ---- */
float   g_ident_eleangle    = 0.0f;
uint8_t g_ident_eleangle_ovr = 0U;

void identify_set_eleangle_override(float theta)
{
    g_ident_eleangle = theta;
    g_ident_eleangle_ovr = 1U;
}

void identify_clear_eleangle_override(void)
{
    g_ident_eleangle_ovr = 0U;
}

/* 转矩模式：填 iq_ref + ctrl_mode=0（模型电流环闭环保持 iq，J 辨识用） */
void identify_apply_torque(float iq)
{
    CyberDog_Motor_FOC_U.ctrl_mode = 0.0f;
    CyberDog_Motor_FOC_U.iq_ref    = iq;
    CyberDog_Motor_FOC_U.coast     = 0.0f;
}
