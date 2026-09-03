/**
 * identify_pole.h — 极对数/方向辨识（移植 AC_Control Para_identify_PoleDirection）
 *
 * 方法：电压强拖开环（VF），正/反扫受控电角度各一整圈机械角行程：
 *   p_f=|Δθe,f/Δθm,f|、p_r=|Δθe,r/Δθm,r|、p=round(½(p_f+p_r))、dir=sign(Δθe·Δθm)；
 *   质量门限 p≥1、正扫整圈、正反方向一致、|p−p_f|≤0.25、|p−p_r|≤0.25。
 * 复用模型 dq 电压模式（ctrl_mode=3）+ 开环电角度覆盖（identify_set_eleangle_override）。
 * 读编码器机械角（Encoder_AS5600.angle 解包）、机械速度（Speed_RPM）、电流幅值（Y.i_alpha/beta_meas）。
 * 输出 ctx->res.pole_pairs_identified / control_to_encoder_dir（不自动写回 Motor_Params，用户手动确认）。
 */
#ifndef IDENTIFY_POLE_H
#define IDENTIFY_POLE_H

#include "identify.h"

void identify_pole_Init(Identify_Context_t *ctx);
void identify_pole_Step(Identify_Context_t *ctx);

#endif /* IDENTIFY_POLE_H */
