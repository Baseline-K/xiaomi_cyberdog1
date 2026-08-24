/**
 * motor_state_machine.c
 *
 * 业务状态机实现（Phase 3a：纯软件框架）。
 * - 移植参考工程 StateMachine.c 的表驱动结构（eventMap/actionMap）。
 * - 内核改为 RTOS 化：静态事件队列 + MotorStateTask 消费。
 * - 3a 阶段状态动作全部为空桩，仅验证状态转换与事件处理。
 *
 * 状态/事件表与动作映射见 motor_state_machine.h 与迁移计划 §5。
 */
#include "motor_state_machine.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"          /* HAL_GetTick */

#include <stdio.h>

#define MOTOR_EVT_QUEUE_LEN  8U

/*-----------------------------------------------------------
 * 静态事件队列
 *----------------------------------------------------------*/
static QueueHandle_t  g_evt_queue;
static StaticQueue_t  g_evt_queue_buf;
static uint8_t        g_evt_queue_storage[MOTOR_EVT_QUEUE_LEN * sizeof(MotorEvent_t)]
                      __attribute__((aligned(8)));

/*-----------------------------------------------------------
 * 内部状态
 *----------------------------------------------------------*/
static motor_state_t          g_cur_state = S_INIT;
static volatile uint8_t       g_fault_pending = 0U;   /* FOC ISR 原子置位 */
static volatile uint32_t      g_stats_posted = 0U;
static volatile uint32_t      g_stats_dropped = 0U;
static volatile uint32_t      g_stats_illegal = 0U;
static volatile uint32_t      g_stats_fault_forced = 0U;

/*-----------------------------------------------------------
 * 动作函数（Phase 3a 全部为空桩；Phase 3b 接入真实动作）
 *----------------------------------------------------------*/
static void None_entry(void) { }
static void None_do(void) { }
static void None_exit(void) { }

static void state_INIT_entry(void) { }
static void state_INIT_do(void)
{
    /* 本工程硬件初始化已在 main_user（调度器启动前）完成；
     * 此处为业务初始化"通过"检查：直接转入 IDLE（计划 §5.5 步骤 2 接入 IDLE 前为纯软件）。 */
    MotorEvent_t evt = { 0 };
    evt.id = EVENT_toIDLE;
    evt.timestamp_ms = HAL_GetTick();
    MotorStateMachine_PostEvent(&evt);
}
static void state_INIT_exit(void) { }

static void state_IDLE_entry(void) { }
static void state_IDLE_do(void) { }

static void state_CALIB_entry(void) { }
static void state_CALIB_do(void) { }
static void state_CALIB_exit(void) { }

static void state_RUN_entry(void) { }
static void state_RUN_do(void) { }
static void state_RUN_exit(void) { }

static void state_STOP_entry(void) { }
static void state_STOP_do(void)
{
    /* 3a 软件测试：置停止后立即回 IDLE（占位；3b 改为"检测电机实际静止"再回 IDLE） */
    MotorEvent_t evt = { 0 };
    evt.id = EVENT_toIDLE;
    evt.timestamp_ms = HAL_GetTick();
    MotorStateMachine_PostEvent(&evt);
}

static void state_FAULT_entry(void) { }
static void state_FAULT_do(void) { }
static void state_FAULT_exit(void) { }

static void state_FAULTOVER_do(void) { }

/*-----------------------------------------------------------
 * 动作表（参照参考工程 actionMap；状态序与 motor_state_t 一致）
 *----------------------------------------------------------*/
static const MotorActionMap_t actionMap[] = {
    {S_INIT,       state_INIT_entry,     state_INIT_do,     state_INIT_exit},
    {S_IDLE,       state_IDLE_entry,     state_IDLE_do,     None_exit},
    {S_CALIB,      state_CALIB_entry,    state_CALIB_do,    state_CALIB_exit},
    {S_RUN,        state_RUN_entry,      state_RUN_do,      state_RUN_exit},
    {S_STOP,       state_STOP_entry,     state_STOP_do,     None_exit},
    {S_FAULT_NOW,  state_FAULT_entry,    state_FAULT_do,    state_FAULT_exit},
    {S_FAULT_OVER, None_entry,           state_FAULTOVER_do, None_exit},
    {S_STANDBY,    None_entry,           None_do,           None_exit},
};

/*-----------------------------------------------------------
 * 事件表（参照参考工程 eventMap，+ 计划 §5.4 修复）
 * 注意：S_RUN→S_RUN 保留但内核不重复 Exit/Enter（命令更新/无操作）。
 *----------------------------------------------------------*/
static const MotorEventMap_t eventMap[] = {
    /* toFAULT：保持参考工程原样，不覆盖 S_INIT（INIT 失败的处理由 INIT Do 决定） */
    {EVENT_toFAULT, S_IDLE,       S_FAULT_NOW},
    {EVENT_toFAULT, S_CALIB,      S_FAULT_NOW},
    {EVENT_toFAULT, S_RUN,        S_FAULT_NOW},
    {EVENT_toFAULT, S_STOP,       S_FAULT_NOW},
    {EVENT_toFAULT, S_FAULT_OVER, S_FAULT_NOW},
    {EVENT_toFAULT, S_STANDBY,    S_FAULT_NOW},

    {EVENT_toIDLE,  S_INIT,       S_IDLE},
    {EVENT_toIDLE,  S_STOP,       S_IDLE},
    {EVENT_toIDLE,  S_FAULT_OVER, S_IDLE},
    {EVENT_toIDLE,  S_CALIB,      S_IDLE},
    {EVENT_toIDLE,  S_STANDBY,    S_IDLE},

    {EVENT_toRUN,   S_IDLE,       S_RUN},
    {EVENT_toRUN,   S_RUN,        S_RUN},   /* 参照参考：重复 Exit/Enter */
    {EVENT_toRUN,   S_STOP,       S_RUN},

    {EVENT_INIT,        S_INIT,   S_INIT},   /* INIT 自转，跑 Do */
    {EVENT_RUNtoSTOP,   S_RUN,    S_STOP},
    {EVENT_IDLEtoCALIB, S_IDLE,   S_CALIB},
    {EVENT_IDLEtoSTANDBY, S_IDLE, S_STANDBY},
    {EVENT_FAULTtoOVER, S_FAULT_NOW, S_FAULT_OVER},
    {EVENT_IDLEtoINIT,  S_IDLE,   S_INIT},
};

#define MOTOR_EVT_MAP_LEN  (sizeof(eventMap) / sizeof(eventMap[0]))
#define MOTOR_ACT_MAP_LEN  (sizeof(actionMap) / sizeof(actionMap[0]))

/*-----------------------------------------------------------
 * 名称映射（日志）
 *----------------------------------------------------------*/
static const char *state_name(motor_state_t s)
{
    switch (s) {
    case S_INIT:       return "S_INIT";
    case S_IDLE:       return "S_IDLE";
    case S_CALIB:      return "S_CALIB";
    case S_RUN:        return "S_RUN";
    case S_STOP:       return "S_STOP";
    case S_FAULT_NOW:  return "S_FAULT_NOW";
    case S_FAULT_OVER: return "S_FAULT_OVER";
    case S_STANDBY:    return "S_STANDBY";
    default:           return "S_UNKNOWN";
    }
}

static const char *event_name(motor_event_id_t e)
{
    switch (e) {
    case EVENT_toFAULT:      return "toFAULT";
    case EVENT_toIDLE:       return "toIDLE";
    case EVENT_toRUN:        return "toRUN";
    case EVENT_RUNtoSTOP:    return "RUNtoSTOP";
    case EVENT_IDLEtoCALIB:  return "IDLEtoCALIB";
    case EVENT_IDLEtoSTANDBY:return "IDLEtoSTANDBY";
    case EVENT_FAULTtoOVER:  return "FAULTtoOVER";
    case EVENT_IDLEtoINIT:   return "IDLEtoINIT";
    case EVENT_INIT:         return "INIT";
    default:                 return "EVENT_UNKNOWN";
    }
}

/*-----------------------------------------------------------
 * 内核：处理单个事件（查表 → Exit/Enter → 提交）
 *----------------------------------------------------------*/
static void MotorStateMachine_ProcessEvent(const MotorEvent_t *evt)
{
    motor_state_t cur = g_cur_state;
    uint32_t i;

    for (i = 0; i < MOTOR_EVT_MAP_LEN; i++) {
        if ((eventMap[i].event == evt->id) && (eventMap[i].cur_state == cur)) {
            motor_state_t next = eventMap[i].next_state;
            /* 参照参考 action_perfrom：即使 next==cur（S_RUN→S_RUN / S_INIT→S_INIT）
             * 也重复执行 Exit(当前) → Enter(下一) → 提交。 */
            actionMap[cur].exit();
            actionMap[next].enter();
            g_cur_state = next;
            printf("FSM: %s --[%s]--> %s\r\n",
                   state_name(cur), event_name(evt->id), state_name(g_cur_state));
            return;
        }
    }
    /* 非法事件（当前态不匹配任何转换） */
    g_stats_illegal++;
    printf("FSM: illegal event [%s] in %s\r\n",
           event_name(evt->id), state_name(cur));
}

/*-----------------------------------------------------------
 * 内核：执行当前状态的 Do（RunningAct）
 *----------------------------------------------------------*/
static void MotorStateMachine_Do(void)
{
    uint32_t i;
    for (i = 0; i < MOTOR_ACT_MAP_LEN; i++) {
        if (actionMap[i].state == g_cur_state) {
            actionMap[i].run();
            return;
        }
    }
}

/*-----------------------------------------------------------
 * 对外接口
 *----------------------------------------------------------*/
void MotorStateMachine_Init(void)
{
    g_evt_queue = xQueueCreateStatic(MOTOR_EVT_QUEUE_LEN,
                                     sizeof(MotorEvent_t),
                                     g_evt_queue_storage,
                                     &g_evt_queue_buf);
    configASSERT(g_evt_queue != NULL);

    g_cur_state = S_INIT;
    g_fault_pending = 0U;

    /* 投递 EVENT_INIT：任务首个循环处理后进入 S_INIT Do（3b 做业务初始化检查） */
    MotorEvent_t evt = { 0 };
    evt.id = EVENT_INIT;
    evt.timestamp_ms = HAL_GetTick();
    MotorStateMachine_PostEvent(&evt);
}

int MotorStateMachine_PostEvent(const MotorEvent_t *evt)
{
    BaseType_t ok;

    if (evt->id == EVENT_toFAULT) {
        /* FAULT 走独立锁存兜底（计划 §5.3/§4.1），保证不因队列满丢失 */
        MotorStateMachine_PostFault();
        return 1;
    }

    ok = xQueueSend(g_evt_queue, evt, 0U);
    if (ok == pdTRUE) {
        g_stats_posted++;
        return 1;
    }
    g_stats_dropped++;   /* 队列满（FAULT/STOP 的兜底在 3c 完善） */
    return 0;
}

void MotorStateMachine_PostFault(void)
{
    g_fault_pending = 1U;   /* 原子字节写，ISR 可安全调用 */
}

void MotorStateMachine_Step(void)
{
    MotorEvent_t evt;

    /* 1. fault_pending 兜底（FOC ISR 置位，1ms 巡检） */
    if (g_fault_pending != 0U) {
        g_fault_pending = 0U;
        g_stats_fault_forced++;
        evt.id = EVENT_toFAULT;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_ProcessEvent(&evt);
    }

    /* 2. 取一个事件（1ms 超时） */
    if (xQueueReceive(g_evt_queue, &evt, pdMS_TO_TICKS(1)) == pdTRUE) {
        MotorStateMachine_ProcessEvent(&evt);
    }

    /* 3. 跑当前态 Do（周期 ≈1ms，空闲时也在走） */
    MotorStateMachine_Do();
}

motor_state_t MotorState_GetCurrent(void)
{
    return g_cur_state;
}

void MotorState_GetStats(uint32_t *posted, uint32_t *dropped,
                         uint32_t *illegal, uint32_t *fault_forced)
{
    if (posted)      *posted      = g_stats_posted;
    if (dropped)     *dropped     = g_stats_dropped;
    if (illegal)     *illegal     = g_stats_illegal;
    if (fault_forced)*fault_forced= g_stats_fault_forced;
}
