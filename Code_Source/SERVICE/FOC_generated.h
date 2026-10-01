#ifndef FOC_GENERATED_H
#define FOC_GENERATED_H

#include <stdint.h>

/* Simulink 生成 FOC 算法的固件侧适配层（SERVICE）
 * - FOC_Generated_Init:  按电机参数/公式计算全部可调参数，并调用模型 initialize()
 * - FOC_Generated_Step:  填充模型输入（来自 MotorCtrl 命令）→ 调 step → 写 TIM1 占空比
 * - FOC_Generated_Reset: 启动时清零模型积分器（防上次运行残留）
 * 生成的算法函数在 Simulink_Model/generated/（由 MATLAB 生成）
 */
void FOC_Generated_Init(void);
void FOC_Generated_Step(void);
void FOC_Generated_Reset(void);

/* 读取模型 PLL 输出的机械转速（RPS），coast 期间仍实时更新 */
float FOC_Generated_GetSpeedRps(void);

/* 读取模型 PLL 的机械加速度估计（RPS/s）。
 * 由模型内 AccelEst 子系统对 speed_meas_rps 做"一阶低通 + 微分"得到：
 *   a[k] = (1-K)*a[k-1] + (K/Ts)*(v[k]-v[k-1]),  fc = 100 Hz, Ts = 1e-4 s
 * 单位与规划器的 ref_accel 一致（RPS/s），可直接相减比较。
 * 注意：它是"测量"加速度，含 PLL 延迟与 100 Hz 低通相位滞后，不要当理想微分用。 */
float FOC_Generated_GetAccelRps2(void);

/* ---- 速度 S 曲线规划：常开（速度模式+未 coast 自动平滑）；仅机动时间可调（RTT plT） ---- */
void FOC_SpeedPlan_SetTimeMs(uint32_t t_ms);   /* 加减速时间对称，默认 300ms */

/* ---- LESO 扰动观测：带宽可调（像 Kp/Ki 传参）；<=0 即关闭（模型自动旁路） ---- */
void  FOC_Leso_SetBwHz(float hz);
float FOC_Leso_GetBwHz(void);

#endif /* FOC_GENERATED_H */
