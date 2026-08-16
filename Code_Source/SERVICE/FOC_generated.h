#ifndef FOC_GENERATED_H
#define FOC_GENERATED_H

/* Simulink 生成 FOC 算法的固件侧适配层（SERVICE）
 * - FOC_Generated_Init:  按电机参数/公式计算全部可调参数，并调用模型 initialize()
 * - FOC_Generated_Step:  填充模型输入（来自 MotorCtrl 命令）→ 调 step → 写 TIM1 占空比
 * - FOC_Generated_Reset: 启动时清零模型积分器（防上次运行残留）
 * 生成的算法函数在 Simulink_Model/generated/（由 MATLAB 生成）
 */
void FOC_Generated_Init(void);
void FOC_Generated_Step(void);
void FOC_Generated_Reset(void);

#endif /* FOC_GENERATED_H */
