/**
 * 位置 S 曲线规划（固件侧；移植 AC_Control SCurve_Position，非对称梯形/三角，无 jerk）
 *
 * 与 speed_plan 同层(MID_foc)、同风格：10kHz 调用，内部 10:1 降采样到 1kHz；
 * 输出平滑位置 pos_ref(rad) + 规划速度 vel_ref(RPS) + 规划加速度 acc_ref(RPS/s)：
 *   - vel_ref → 位置环速度前馈（PositionController 的 vel_ff）
 *   - acc_ref → 速度环加速度前馈（U.ref_accel → iq_ff 惯量项）
 * 绝对目标：SetTarget 传绝对位置(rad)，内部以“当前规划位置/速度”为起点闭式重规划（非 AC 的零基准）。
 * 加减速时间共用 plT_ms（在 FOC_generated 侧同步）。
 */
#ifndef POSITION_PLAN_H
#define POSITION_PLAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void PositionPlan_Reset(void);
void PositionPlan_SetTimeMs(uint32_t ms);      /* 加/减速时间(共用 plT) */
void PositionPlan_SetMaxVelRps(float rps);     /* 规划峰值限速 */
/* 绝对目标(rad)；运行中变化超过阈值会从当前规划状态平滑重规划 */
void PositionPlan_SetTarget(float target_rad);

/* 每次 10kHz FOC step 调用；active=位置模式且未 coast。
 * pos_ref(rad) / vel_ref(RPS) / acc_ref(RPS/s) 为平滑输出；非 active 时位置直通 fbk、速度加速度 0。 */
void PositionPlan_Step10k(int active, float fbk_rad,
                          float *pos_ref, float *vel_ref, float *acc_ref);

#ifdef __cplusplus
}
#endif
#endif /* POSITION_PLAN_H */
