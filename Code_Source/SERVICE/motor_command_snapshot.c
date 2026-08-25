/**
 * motor_command_snapshot.c
 *
 * 命令快照双缓冲实现（计划 §7.1）。
 * - 发布（任务）：写非活动槽 → 翻转 g_active（原子字节写）。
 * - 读取（ISR）：读 g_active → 读该槽。发布永不写正在被读的槽，ISR 始终读到完整槽。
 * - 一次只翻转一字节，无撕裂；ISR 可能读到上一周期的命令（可接受）。
 */
#include "motor_command_snapshot.h"

static volatile uint8_t g_active = 0U;
static MotorCommand_t   g_slot[2];

void MotorCommand_Publish(const MotorCommand_t *cmd)
{
    uint8_t new_active = (uint8_t)(1U - g_active);
    g_slot[new_active] = *cmd;
    g_active = new_active;
}

const MotorCommand_t *MotorCommand_Get(void)
{
    return &g_slot[g_active];
}
