/**
 * 速度 S 曲线轨迹规划（固件侧，仿 AC_Control Traj_Plan.c 的 SCurve_Speed_Profile）
 *
 * 用法：在 10kHz FOC step 每次调用 SpeedPlan_Step10k(...)，内部 10:1 降采样到 1kHz，
 *       输出平滑速度 ref_speed (RPS) 与规划加速度 ref_accel (RPS/s)，
 *       供 Simulink 生成的速度环参考 + 三项前馈（iq_ff = J2piKt·α + B·ω + Fc·sign(ω)）。
 *
 * 常开：速度模式 + 未 coast（active=1）时总是平滑；否则直通原始 cmd_rps、acc=0。
 * jerk 受限三段（加加速/匀加速/减加速），方向反转走“先刹到 0 再反向”两段。
 */
#ifndef SPEED_PLAN_H
#define SPEED_PLAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 复位/初始化：当前速度锚定到 0、is_moving=0、按默认时间。 */
void SpeedPlan_Reset(void);

/* 每次 10kHz FOC step 调用。
   active: 1=速度模式且未 coast（由上层给）；cmd_rps: 原始速度目标(RPS)；fbk_rps: 实测转速(RPS)。
   常开：active=1 即平滑（内部无额外使能位）；输出 *ref_rps / *acc_rps2（平滑或直通）。 */
void SpeedPlan_Step10k(int active, float cmd_rps, float fbk_rps, float *ref_rps, float *acc_rps2);

/* accel/decel 时间 ms，默认 300ms；<=10ms 会被抬到 10ms。 */
void SpeedPlan_SetTimeMs(uint32_t accel_ms, uint32_t decel_ms);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_PLAN_H */
