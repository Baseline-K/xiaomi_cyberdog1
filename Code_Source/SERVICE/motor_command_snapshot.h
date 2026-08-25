/**
 * motor_command_snapshot.h
 *
 * 控制命令快照（计划 §7.1）：MotorStateTask 单写发布，FOC ISR 只读。
 * 双缓冲避免多字段并发撕裂：发布写非活动槽再翻转 active，ISR 读 active 后读该槽。
 */
#ifndef MOTOR_COMMAND_SNAPSHOT_H
#define MOTOR_COMMAND_SNAPSHOT_H

#include <stdint.h>

typedef struct {
    uint8_t  enable;     /* 1=驱动算法生效，0=停机 */
    uint8_t  coast;      /* 1=滑行（模型 duty 强制 0.5，PLL 仍跑给实时转速） */
    uint8_t  mode;       /* MC_MODE_SPEED / MC_MODE_TORQUE */
    float    speed_rps;  /* 速度模式目标 */
    float    iq_ref_A;   /* 转矩模式目标 */
    float    pos_ref;    /* 位置模式目标（预留） */
} MotorCommand_t;

/* 状态机（任务上下文）发布命令；内部短写 + 翻转 active */
void MotorCommand_Publish(const MotorCommand_t *cmd);

/* FOC ISR 读取当前命令快照（读 active 后返回对应槽指针） */
const MotorCommand_t *MotorCommand_Get(void);

#endif /* MOTOR_COMMAND_SNAPSHOT_H */
