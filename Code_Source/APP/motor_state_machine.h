/**
 * motor_state_machine.h
 *
 * 业务状态机（参照参考工程 StateMachine.c 的设计移植）。
 *
 * - 8 状态 / 9 转换命名事件 / eventMap(转换表) / actionMap(Enter/Do/Exit)。
 * - 内核相对参考工程的修复（计划 §5.4）：
 *   1) 单 pending 槽 → 静态事件队列（8 项）
 *   2) EVENT_toFAULT 覆盖 S_INIT（INIT 失败进安全态）
 *   3) S_RUN→S_RUN 不重复 Exit/Enter（视为命令更新/无操作）
 *   4) 主循环 action_perfrom → MotorStateTask 消费（队列 + 1ms 巡检 fault_pending）
 *
 * 对应《FreeRTOS_Hybrid_Architecture_Migration_Plan.md》§5 / §9 Phase 3。
 */
#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H

#include <stdint.h>

/*-----------------------------------------------------------
 * 状态（参照参考工程 system_state_t，8 个）
 *----------------------------------------------------------*/
typedef enum {
    S_INIT,
    S_IDLE,
    S_CALIB,
    S_RUN,
    S_STOP,
    S_FAULT_NOW,
    S_FAULT_OVER,
    S_STANDBY
} motor_state_t;

/*-----------------------------------------------------------
 * 事件（参照参考工程，按转换命名，9 个）
 *----------------------------------------------------------*/
typedef enum {
    EVENT_toFAULT = 1,
    EVENT_toIDLE,
    EVENT_toRUN,
    EVENT_RUNtoSTOP,
    EVENT_IDLEtoCALIB,
    EVENT_IDLEtoSTANDBY,
    EVENT_FAULTtoOVER,
    EVENT_IDLEtoINIT,
    EVENT_INIT
} motor_event_id_t;

/*-----------------------------------------------------------
 * 事件负载（计划 §5.2；命令参数随 START 事件携带）
 *----------------------------------------------------------*/
typedef struct {
    uint8_t  id;
    uint32_t timestamp_ms;
    uint8_t  mode;       /* 命令模式 */
    float    speed_rps;  /* 命令速度 */
    float    iq_ref_A;   /* 命令电流 */
    float    pos_ref;    /* 命令位置 */
} MotorEvent_t;

/*-----------------------------------------------------------
 * 表驱动定义
 *----------------------------------------------------------*/
typedef void (*MotorStateAction)(void);

typedef struct {
    motor_state_t    state;
    MotorStateAction enter;
    MotorStateAction run;    /* Do */
    MotorStateAction exit;
} MotorActionMap_t;

typedef struct {
    motor_event_id_t event;
    motor_state_t    cur_state;
    motor_state_t    next_state;
} MotorEventMap_t;

/*-----------------------------------------------------------
 * 对外接口
 *----------------------------------------------------------*/
/* 初始化：创建静态事件队列，状态置 S_INIT，投递 EVENT_INIT（调度器启动前调用） */
void MotorStateMachine_Init(void);

/* 投递事件（任务上下文；EVENT_toFAULT 走 fault_pending 锁存兜底）。返回 1=成功/已兜底，0=队列满丢弃 */
int MotorStateMachine_PostEvent(const MotorEvent_t *evt);

/* 故障锁存（可由 ISR 原子调用，计划 §4.1 两段式）。mask=故障位，与故障字取或 */
void MotorStateMachine_PostFault(uint32_t mask);

/* 清除故障位（恢复判定：故障位清零后 FAULT_NOW_do 才转 FAULT_OVER） */
void MotorStateMachine_ClearFault(uint32_t mask);

/* STOP 锁存兜底（队列满时 STOP 不丢失，计划 §5.3） */
void MotorStateMachine_PostStop(void);

/* 状态机步进：由 MotorStateTask 循环调用（1ms 超时取事件 + 巡检 pending + 跑 Do） */
void MotorStateMachine_Step(void);

/* 当前状态（FOC ISR 只读） */
motor_state_t MotorState_GetCurrent(void);

/* 统计（posted/dropped/illegal/fault_forced） */
void MotorState_GetStats(uint32_t *posted, uint32_t *dropped,
                         uint32_t *illegal, uint32_t *fault_forced);

#endif /* MOTOR_STATE_MACHINE_H */
