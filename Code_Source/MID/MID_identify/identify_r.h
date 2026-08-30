/**
 * identify_r.h — 相电阻 R 辨识（d 轴电压斜坡 + 4 点线性最小二乘）
 *
 * 方法（移植 AC_Control Resistance_Identify_Step_VoltageRamp）：
 *   锁转子于 d 轴，vd 单调斜坡升压到 4 个目标电流点，各点稳态平均 (id,vd)，
 *   LS 拟合 Vd = R·Id + Vdead → 斜率=R（相电阻）、截距=Vdead（死区压降）。
 * 复用模型 dq 电压模式（ctrl_mode=3）：identify_apply_voltage(vd, 0)。
 */
#ifndef IDENTIFY_R_H
#define IDENTIFY_R_H

#include "identify.h"

void identify_r_Init(Identify_Context_t *ctx);
void identify_r_Step(Identify_Context_t *ctx);

#endif /* IDENTIFY_R_H */
