/**
 * identify_l.h — 相电感 L（Ld/Lq）辨识（HFI 高频脉冲注入，完整法）
 *
 * 方法（移植 AC_Control Inductance_Identify_HFI_Step，L_ONLY 模式）：
 *   32 方向 αβ 电压短脉冲（base→+V→−V），偶奇分解得导纳响应，
 *   DFT 得 Y0/Y2 → Lavg=1/Y0、Lmax/Lmin；凸极<1.05 → Ld=Lq=Lavg。
 * 复用模型 αβ 电压模式（ctrl_mode=4）；电流响应读 Y.i_alpha_meas/i_beta_meas。
 * 输出 ctx->res.Ld/Lq，由 Identify_Params_Update 提交 Motor_Params.Ld/Lq + 重算 CurrQ_Kp。
 */
#ifndef IDENTIFY_L_H
#define IDENTIFY_L_H

#include "identify.h"

void identify_l_Init(Identify_Context_t *ctx);
void identify_l_Step(Identify_Context_t *ctx);

#endif /* IDENTIFY_L_H */
