/**
 * identify_j.h — 转动惯量 J 辨识（移植 AC_Control Inertia_Identify_Step）
 *
 * 方法：加减速对称法。4 个 iq 点（0.4~1.0×额定），每点加速采样（速度 0.30→0.50 max）、
 *   减速采样（0.50→0.30 max），J_est=(Kt·iq_avg−B·ωm_avg−fc)/α_avg；
 *   J=(J_accel+J_decel)/2、一致性 J_error。
 * 驱动复用模型转矩模式（ctrl_mode=0，identify_apply_torque）；读 PLL 转速 + LPF 加速度。
 * Kt/B/fc 来自磁链辨识结果 ctx->res（Kt 有 Motor_Params.Flux 兜底）。
 * 输出 ctx->res.J / J_error，提交 Motor_Params.Rotor_inertia。
 */
#ifndef IDENTIFY_J_H
#define IDENTIFY_J_H

#include "identify.h"

void identify_j_Init(Identify_Context_t *ctx);
void identify_j_Step(Identify_Context_t *ctx);

#endif /* IDENTIFY_J_H */
