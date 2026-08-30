/**
 * identify_dead.h — 死区补偿 LUT 辨识（20 点电流注入）
 *
 * 方法（移植 AC_Control DeadTime_Identify_Step，改为电压斜坡法）：
 *   d 轴锁转子，vd 单调斜坡升压到 20 个目标电流点（密低疏高 0.05~2.2A），
 *   各点稳态平均 |I| 与 vd，得 Vcomp[|I|] = vd − R·|I|（纯逆变器畸变，截 ≥0）。
 * 复用模型 dq 电压模式（ctrl_mode=3）；R 用 Motor_Params.Phase_R（上次辨识值）。
 * 结果存 ctx->res.dead_lut_I/V，由 Identify_Params_Update 提交 + DeadComp_En=1。
 */
#ifndef IDENTIFY_DEAD_H
#define IDENTIFY_DEAD_H

#include "identify.h"

#define DEAD_LUT_N  20   /* 死区 LUT 点数（与模型网格一致） */

void identify_dead_Init(Identify_Context_t *ctx);
void identify_dead_Step(Identify_Context_t *ctx);

#endif /* IDENTIFY_DEAD_H */
