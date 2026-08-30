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
#include "motor_command_snapshot.h"
#include "FOC_generated.h"
#include "MotorCtrl.h"
#include "FOC_run.h"       /* MotorState */
#include "main_user.h"     /* Param_init / Param_deinit */
#include "identify.h"      /* 离线辨识：S_CALIB 复用 */

#include <stdio.h>
#include <math.h>

#define STOP_SPEED_THRESHOLD_RPS  0.1f   /* 判定电机静止的转速阈值 */
#define STOP_TIMEOUT_MS           3000U  /* 停机超时兜底 */

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
static volatile uint8_t       g_stop_pending  = 0U;   /* STOP 锁存兜底（队列满不丢） */
static volatile uint32_t      g_fault_bits    = 0U;   /* 故障位锁存 */
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

/* ---- 待提交命令缓存（EVENT_toRUN 携带，S_RUN_entry 使用） ---- */
static MotorCommand_t g_pending_cmd = { 0 };
static uint32_t g_stop_tick = 0U;

/* 发布 coast 快照（停机/滑行：duty 0.5、转矩模式 iq=0 避免速度环积分饱和） */
static void publish_coast(void)
{
    MotorCommand_t cmd = { 0 };
    cmd.enable    = 0U;
    cmd.coast     = 1U;
    cmd.mode      = 0U;       /* MC_MODE_TORQUE */
    cmd.speed_rps = 0.0f;
    cmd.iq_ref_A  = 0.0f;
    cmd.pos_ref   = 0.0f;
    MotorCommand_Publish(&cmd);
}

/* 发布运行快照（enable=1、coast=0，命令来自 g_pending_cmd） */
static void publish_run(void)
{
    MotorCommand_t cmd = g_pending_cmd;
    cmd.enable = 1U;
    cmd.coast  = 0U;
    MotorCommand_Publish(&cmd);
}

static void state_INIT_entry(void)
{
    /* 初始安全态：coast（PLL 常跑，duty 0.5） */
    publish_coast();
}
static void state_INIT_do(void)
{
    /* 本工程硬件初始化已在 main_user（调度器启动前）完成；
     * 业务初始化"通过"检查：直接转入 IDLE。 */
    MotorEvent_t evt = { 0 };
    evt.id = EVENT_toIDLE;
    evt.timestamp_ms = HAL_GetTick();
    MotorStateMachine_PostEvent(&evt);
}
static void state_INIT_exit(void) { }

static void state_IDLE_entry(void)
{
    publish_coast();
    MotorState.run_state = RUNSTATE_STOPPED;
    MotorCtrl.enable = 0U;
    MotorCtrl.state  = MC_STATE_STOPPED;
    PowerStage_Disable();
}
static void state_IDLE_do(void) { }

static void state_CALIB_entry(void)
{
    publish_coast();                 /* 先 coast 撤力，再启动辨识 */
    MotorState.run_state = RUNSTATE_IDENTIFYING;
    MotorState.ctrl_mode = MT_IDENTIFY;
    Identify_Start();                /* 启动离线辨识任务链（任务掩码由 identify 命令预设） */
    PowerStage_Enable();             /* 辨识需 PWM 输出驱动电流（RUN entry 才有，这里补上） */
}
static void state_CALIB_do(void)
{
    Identify_Process();              /* 主循环推进辨识任务 */
    Identify_Status_e st = Identify_GetStatus();
    if (st == IDENTIFY_DONE || st == IDENTIFY_FAILED || st == IDENTIFY_ABORTED) {
        if (st == IDENTIFY_DONE) {
            Identify_Params_Update();   /* RAM 提交结果 + 重算增益 */
        }
        MotorEvent_t evt = { 0 };
        evt.id = EVENT_toIDLE;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_PostEvent(&evt);
    }
}
static void state_CALIB_exit(void)
{
    PowerStage_Disable();            /* 离开辨识立即断功率（安全） */
    Identify_End();                  /* 统一收尾：恢复堵转检测使能 + state=IDLE */
    MotorState.run_state = RUNSTATE_STOPPED;
    MotorState.ctrl_mode = MT_STOP;
    publish_coast();
}

static void state_RUN_entry(void)
{
    FOC_Generated_Reset();        /* 清模型积分器（防上次运行残留过冲） */
    publish_run();
    Param_init();
    PowerStage_Enable();
    MotorState.run_state = RUNSTATE_RUNNING;
    MotorCtrl.enable = 1U;
    MotorCtrl.state  = MC_STATE_RUNNING;
    MotorCtrl.mode     = g_pending_cmd.mode;
    MotorCtrl.speed_rps = g_pending_cmd.speed_rps;
    MotorCtrl.iq_ref_A  = g_pending_cmd.iq_ref_A;
    MotorCtrl.pos_ref   = g_pending_cmd.pos_ref;
}
static void state_RUN_do(void) { }
static void state_RUN_exit(void) { }

static void state_STOP_entry(void)
{
    publish_coast();              /* coast：duty 0.5、PLL 仍跑给实时转速 */
    PowerStage_Disable();
    Param_deinit();
    MotorState.run_state = RUNSTATE_STOPPED;
    MotorCtrl.enable = 0U;
    MotorCtrl.state  = MC_STATE_STOPPED;
    g_stop_tick = HAL_GetTick();
}
static void state_STOP_do(void)
{
    /* 实际转速低于阈值判定静止 → 回 IDLE；超时兜底强制回 IDLE */
    float speed = fabsf(FOC_Generated_GetSpeedRps());
    if (speed < STOP_SPEED_THRESHOLD_RPS) {
        MotorEvent_t evt = { 0 };
        evt.id = EVENT_toIDLE;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_PostEvent(&evt);
    } else if ((HAL_GetTick() - g_stop_tick) >= STOP_TIMEOUT_MS) {
        printf("FSM: STOP timeout (speed=%.2f), force to IDLE\r\n", (double)speed);
        MotorEvent_t evt = { 0 };
        evt.id = EVENT_toIDLE;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_PostEvent(&evt);
    }
}

static void state_FAULT_entry(void)
{
    publish_coast();              /* coast：PLL 仍跑给转速，duty 0.5 */
    PowerStage_Disable();         /* 立即关功率（硬件级安全） */
    MotorState.run_state = RUNSTATE_STOPPED;
    MotorCtrl.enable = 0U;
    MotorCtrl.state  = MC_STATE_STOPPED;
    MotorCtrl.faults |= (uint16_t)g_fault_bits;   /* 锁存故障位供状态上报 */
    printf("FSM: FAULT_NOW, fault=0x%08lX\r\n", (unsigned long)g_fault_bits);
}
static void state_FAULT_do(void)
{
    /* 恢复判定：故障位全部清除后进入 FAULT_OVER（Phase 4 由 Safety 细化） */
    if (g_fault_bits == 0U) {
        MotorEvent_t evt = { 0 };
        evt.id = EVENT_FAULTtoOVER;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_PostEvent(&evt);
    }
}
static void state_FAULT_exit(void)
{
    MotorCtrl.faults = 0U;   /* 清除锁存（分类清除策略 Phase 4 细化） */
}

static void state_FAULTOVER_do(void)
{
    /* 电机静止后才允许回 IDLE */
    float speed = fabsf(FOC_Generated_GetSpeedRps());
    if (speed < STOP_SPEED_THRESHOLD_RPS) {
        MotorEvent_t evt = { 0 };
        evt.id = EVENT_toIDLE;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_PostEvent(&evt);
    }
}

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

    /* 缓存 EVENT_toRUN 携带的命令参数（供 S_RUN_entry 提交快照） */
    if (evt->id == EVENT_toRUN) {
        g_pending_cmd.mode     = evt->mode;
        g_pending_cmd.speed_rps = evt->speed_rps;
        g_pending_cmd.iq_ref_A  = evt->iq_ref_A;
        g_pending_cmd.pos_ref   = evt->pos_ref;
    }

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

    /* 初始安全快照：coast（调度器启动前 FOC ISR 就读取，需先处于安全态） */
    publish_coast();

    /* 投递 EVENT_INIT：任务首个循环处理后进入 S_INIT Do（业务初始化检查） */
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
        MotorStateMachine_PostFault(0xFFFFFFFFUL);   /* 未知故障源，置全部位 */
        return 1;
    }
    if (evt->id == EVENT_RUNtoSTOP) {
        /* STOP 也走锁存兜底（计划 §5.3），队列满不丢 */
        MotorStateMachine_PostStop();
        return 1;
    }

    ok = xQueueSend(g_evt_queue, evt, 0U);
    if (ok == pdTRUE) {
        g_stats_posted++;
        return 1;
    }
    g_stats_dropped++;
    return 0;
}

void MotorStateMachine_PostFault(uint32_t mask)
{
    g_fault_bits   |= mask;       /* 原子字节写（32 位对齐，M4 单写），ISR 可安全调用 */
    g_fault_pending = 1U;
}

void MotorStateMachine_ClearFault(uint32_t mask)
{
    g_fault_bits &= ~mask;
}

void MotorStateMachine_PostStop(void)
{
    g_stop_pending = 1U;          /* 原子字节写，ISR/任务可安全调用 */
}

void MotorStateMachine_Step(void)
{
    MotorEvent_t evt;

    /* 1. fault_pending 兜底（FOC ISR 置位，1ms 巡检） */
    if (g_fault_pending != 0U) {
        g_fault_pending = 0U;
        /* 已在故障态：故障位已由 PostFault 更新（可上报），不重复触发 toFAULT（防非法事件刷屏） */
        if ((g_cur_state != S_FAULT_NOW) && (g_cur_state != S_FAULT_OVER)) {
            g_stats_fault_forced++;
            evt.id = EVENT_toFAULT;
            evt.timestamp_ms = HAL_GetTick();
            MotorStateMachine_ProcessEvent(&evt);
        }
    }

    /* 2. stop_pending 兜底（队列满时 STOP 不丢） */
    if (g_stop_pending != 0U) {
        g_stop_pending = 0U;
        evt.id = EVENT_RUNtoSTOP;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_ProcessEvent(&evt);
    }

    /* 3. 取一个事件（1ms 超时） */
    if (xQueueReceive(g_evt_queue, &evt, pdMS_TO_TICKS(1)) == pdTRUE) {
        MotorStateMachine_ProcessEvent(&evt);
    }

    /* 4. 跑当前态 Do（周期 ≈1ms，空闲时也在走） */
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
