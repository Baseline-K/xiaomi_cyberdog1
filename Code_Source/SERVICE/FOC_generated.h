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

/* ---- 速度 S 曲线规划：常开（速度模式+未 coast 自动平滑）；仅机动时间可调（RTT plT） ---- */
void FOC_SpeedPlan_SetTimeMs(uint32_t t_ms);   /* 加减速时间对称，默认 300ms */

/* ---- LESO 扰动观测：带宽可调（像 Kp/Ki 传参）；<=0 即关闭（模型自动旁路） ---- */
void  FOC_Leso_SetBwHz(float hz);
float FOC_Leso_GetBwHz(void);

#endif /* FOC_GENERATED_H */
