#ifndef CANOPEN_OD_H
#define CANOPEN_OD_H

/* CANopen SDO 式命令解析（通信协议层，放 APP，同参考项目 And_uartcom.c） */

void CANopen_OD_Process(void);   /* 主循环 1ms 调：弹帧 → 解析 → 写 MotorCtrl 影子 */

#endif /* CANOPEN_OD_H */
