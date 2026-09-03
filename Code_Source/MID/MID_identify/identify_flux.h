/**
 * identify_flux.h — 磁链 ψf 辨识（空载反电动势，4 转速点 LS）
 *
 * 方法（移植 AC_Control FluxLinkage_Identify_Step）：
 *   空载下注入 vq（dq 电压模式，vd=0），电机加速到 4 个目标转速，
 *   各点稳态平均 (vq, iq, ωe)，LS 拟合 vq−R·iq = ψf·ωe + Vdt → ψf（斜率）、Vdt（截距）；
 *   兼拟合摩擦 iq = B_A·ωm + f_A → B、库仑摩擦。
 * 复用模型 dq 电压模式（ctrl_mode=3）；测 Y.iq_meas、Y.speed_meas_rps。
 * 输出 ctx->res.Flux_linkage / Ke / Kt / B / friction_Coulomb。
 */
#ifndef IDENTIFY_FLUX_H
#define IDENTIFY_FLUX_H

#include "identify.h"

void identify_flux_Init(Identify_Context_t *ctx);
void identify_flux_Step(Identify_Context_t *ctx);

#endif /* IDENTIFY_FLUX_H */
