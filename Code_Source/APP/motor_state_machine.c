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
#include "Safety_Module.h" /* FAULT_*_MASK（定向重初始化分类） */
#include "CAN_bsp.h"       /* CAN_bsp_Recover（通信类故障恢复） */
#include "drv83xx.h"       /* DRV83XX_ReadAndClearFaults（栅极驱动类故障恢复） */

#include <stdio.h>
#include <math.h>

#define STOP_SPEED_THRESHOLD_RPS  0.1f   /* 判定电机静止的转速阈值 */
#define STOP_TIMEOUT_MS           3000U  /* 停机超时兜底 */
/* S_FAULT_OVER 的绝对超时兜底。取 3s：正常情况 1ms 内速度就低于阈值，
 * 这个超时只在"速度拿不到有效值"时才用得上（见 state_FAULTOVER_do 的说明）。 */
#define FAULTOVER_TIMEOUT_MS      3000U

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
/* 故障 CAN 上报（CommTask 读、状态机维护；见 motor_state_machine.h） */
volatile uint32_t             g_fault_report_word   = 0U;   /* 故障时刻锁存的 32 位掩码 */
volatile uint8_t              g_fault_report_active = 0U;   /* 1=故障态，持续上报 */
volatile uint8_t              g_fault_report_dirty  = 0U;   /* 1=新故障，需立即发一帧 */
/* FAULT_NOW 定向软重初始化（编码器/CAN 通信，非阻塞） */
static uint32_t               g_recovery_mask = 0U;
static uint8_t                g_recovery_done = 0U;
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
/* 进入 S_FAULT_OVER 的时刻（在 state_FAULT_exit 里打点），用于绝对超时兜底 */
static uint32_t g_faultover_tick = 0U;

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
    g_fault_report_active = 0U;   /* 回 IDLE 停止故障上报 */
}
static void state_IDLE_do(void)
{
    /* 兜底：S_INIT 期间锁存的故障位没人处理。
     * eventMap 里没有 {EVENT_toFAULT, S_INIT}（S_INIT 只是单拍直通，见 state_INIT_do），
     * 所以上电时锁存的故障 —— 例如 FOC 中断在调度器启动前就已开始跑
     * （main_user 里 FOC_Interrupt_Start 早于 AppTasks_Init），编码器在这段时间
     * 判故障调用 Encoder_Adapt 的 PostFault(FAULT_ENCODER_MASK) —— 会变成
     * "illegal event [toFAULT] in S_INIT" 被丢掉，故障位却留在 g_fault_bits 里。
     * 进 IDLE 时补一次检查，把它转成正常的 toFAULT（于是会关功率 + 发 EMCY 上报）。
     *
     * 不会自激：state_FAULT_exit 离开 FAULT_NOW 时已把 g_fault_bits 清零，
     * 所以恢复回 IDLE 之后这里不会又触发一次。 */
    if (g_fault_bits != 0U) {
        MotorEvent_t evt = { 0 };
        evt.id = EVENT_toFAULT;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_PostEvent(&evt);
    }
}

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
    MotorCtrl.faults |= g_fault_bits;   /* 锁存故障位供状态上报（32 位，不再截断） */

    /* CAN 上报准备：锁存故障字，CommTask 立即发一帧、之后每 10ms 重发 */
    g_fault_report_word   = g_fault_bits;
    g_fault_report_active = 1U;
    g_fault_report_dirty  = 1U;

    /* 需定向软重初始化的硬件类（编码器 / 栅极驱动 / CAN 通信）；
     * 其余类（软过流、过压、欠压、堵转、转速异常、任务看门狗…）下一拍即自动恢复。 */
    g_recovery_mask = g_fault_bits & (FAULT_ENCODER_MASK | FAULT_GATE_DRIVER_MASK | FAULT_COMM_TIMEOUT_MASK);
    g_recovery_done = (g_recovery_mask == 0U) ? 1U : 0U;

    printf("FSM: FAULT_NOW, fault=0x%08lX\r\n", (unsigned long)g_fault_bits);
}
static void state_FAULT_do(void)
{
    /* 定向软重初始化（非阻塞；失败下拍重试，仿 AC_Control 的 ops 重初始化）。
     * 不在此做 DRV83XX_Init / Offset_Current_Start 等阻塞操作（会饿死 CommTask）。 */
    if (g_recovery_done == 0U) {
        uint8_t ok = 1U;

        if ((g_recovery_mask & FAULT_ENCODER_MASK) != 0U) {
            Param_init();            /* 编码器跟踪软复位（无总线访问） */
            Param_deinit();          /* 清 dq/PLL 滤波状态（ISR 并发仅影响个别采样） */
            FOC_Generated_Reset();   /* 清模型控制环状态 */
        }
        if ((g_recovery_mask & FAULT_GATE_DRIVER_MASK) != 0U) {
            /* 读 FSR1/FSR2：读操作本身就清除 DRV 内部锁存。OCP_MODE=LATCH 时这是
             * 唯一的解锁途径（不解锁的话 DRV 会一直把栅极按住、PWM 给了也不通）。
             * 返回 0 = PA12 仍为低 → 真故障没消失，下拍重试（本函数非阻塞，安全）。 */
            if (DRV83XX_ReadAndClearFaults() == 0) {
                ok = 0U;
            }
        }
        if ((g_recovery_mask & FAULT_COMM_TIMEOUT_MASK) != 0U) {
            if (CAN_bsp_Recover() == 0) {   /* bus-off 恢复；本已就绪则直接返回 1 */
                ok = 0U;
            }
        }
        g_recovery_done = ok;
    }

    /* 重初始化完成（或本类无需）→ 自动转 FAULT_OVER（不再需要手动清故障位） */
    if (g_recovery_done != 0U) {
        MotorEvent_t evt = { 0 };
        evt.id = EVENT_FAULTtoOVER;
        evt.timestamp_ms = HAL_GetTick();
        MotorStateMachine_PostEvent(&evt);
    }
}
static void state_FAULT_exit(void)
{
    /* 自动恢复：退出 FAULT_NOW 即清故障字与锁存（对应 AC 的 state_FAULT_exit） */
    g_fault_bits     = 0U;
    MotorCtrl.faults = 0U;
    /* 本函数在"离开 FAULT_NOW"时执行，也就是刚进入 FAULT_OVER —— 在此打点，
     * 供 state_FAULTOVER_do 的绝对超时兜底使用。 */
    g_faultover_tick = HAL_GetTick();
}

static void state_FAULTOVER_do(void)
{
    /* 又出现故障 → 回 FAULT_NOW（防抖，仿 AC_Control） */
    if (g_fault_bits != 0U) {
        MotorEvent_t e = { 0 };
        e.id = EVENT_toFAULT;
        e.timestamp_ms = HAL_GetTick();
        MotorStateMachine_PostEvent(&e);
        return;
    }

    /* 电机静止 → 停止上报并回 IDLE。
     * 判据必须有**两道保险**，否则会死锁（实测踩过）：
     *   1) 速度必须是有限值。若模型因为 Vbus=0 之类的原因算出 NaN，
     *      fabsf(NaN) < 0.1 恒为 false，本状态永远退不出去；
     *   2) 绝对超时兜底 —— 与 S_STOP 一致。凡是"靠外部量判据"的退出，
     *      都要有超时，因为那个外部量可能压根不更新
     *      （例如 FOC 中断没在跑时转速是冻结值）。 */
    float speed = FOC_Generated_GetSpeedRps();
    int   stopped = isfinite(speed) && (fabsf(speed) < STOP_SPEED_THRESHOLD_RPS);
    int   timeout = ((HAL_GetTick() - g_faultover_tick) >= FAULTOVER_TIMEOUT_MS);

    if (stopped || timeout) {
        if (timeout && !stopped) {
            printf("FSM: FAULT_OVER timeout (speed=%s), force to IDLE\r\n",
                   isfinite(speed) ? "non-zero" : "non-finite");
        }
        g_fault_report_active = 0U;
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
    /* 调度器启动前 FOC 中断就已经在跑（main_user 里 FOC_Interrupt_Start 早于
     * AppTasks_Init），所以这里可能已经有 PostFault 锁存的故障位。
     * 不能直接把 pending 清 0 —— 那样 g_fault_bits 有值却没人处理，
     * 掩码会在后面被 EVENT_toFAULT 兜底路径覆盖成 0xFFFFFFFF，线索就丢了。
     * 保留 pending，让 Step() 的第一个分支正常走 toFAULT（掩码原样带过去）。 */
    g_fault_pending = (g_fault_bits != 0U) ? 1U : 0U;

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
        /* FAULT 走独立锁存兜底（计划 §5.3/§4.1），保证不因队列满丢失。
         *
         * 但**不能无条件置全部位**：本分支也被"已经锁存了具体故障位"的调用者走到
         *   - state_IDLE_do() 的 S_INIT 兜底（接住上电期间被 eventMap 丢掉的故障）
         *   - state_FAULTOVER_do() 的"又出故障"
         * 无条件写 0xFFFFFFFF 会把真实掩码（1<<2 过压 / 1<<13 通信超时 / 1<<14 编码器…）
         * 覆盖掉，现场只剩一句 "FSM: FAULT_NOW, fault=0xFFFFFFFF"，什么线索都没了，
         * 而且 g_recovery_mask 会变成"三类恢复全跑一遍"。
         * 只有确实没有任何来源信息时才置全部位。
         * PostFault 顺带把 g_fault_pending 置 1（本分支不走队列，靠它触发 toFAULT）。 */
        MotorStateMachine_PostFault((g_fault_bits == 0U) ? 0xFFFFFFFFUL : g_fault_bits);
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

/* 当前锁存的故障位（ISR/任务皆可读）。
 * 用途：PowerStage_Enable() 的故障闸门 —— 那是个"不能等状态机"的场合，
 * 必须在**同一时刻**知道有没有故障，而不是等 1ms 后状态机切到 FAULT。 */
uint32_t MotorStateMachine_GetFaultBits(void)
{
    return g_fault_bits;
}

void MotorStateMachine_PostStop(void)
{
    g_stop_pending = 1U;          /* 原子字节写，ISR/任务可安全调用 */
}

void MotorStateMachine_Step(void)
{
    MotorEvent_t evt;

    /* 1. fault_pending 兜底（FOC ISR / EXTI 置位，1ms 巡检） */
    if (g_fault_pending != 0U) {
        g_fault_pending = 0U;
        /* 已在故障态：故障位已由 PostFault 更新（可上报），不重复触发 toFAULT（防非法事件刷屏）。
         * S_INIT 也跳过：eventMap 里没有 {EVENT_toFAULT, S_INIT}（S_INIT 是单拍直通），
         * 投了只会打一行 "illegal event [toFAULT] in S_INIT" 并污染 illegal 计数。
         * 上电期间锁存的故障由 state_IDLE_do() 的兜底检查接手 —— 掩码留在
         * g_fault_bits 里不会丢，进 IDLE 时会被转成正常的 toFAULT（含 CAN 上报）。
         * 实测：上电时 PA12 nFAULT 的 EXTI 会在 MX_GPIO_Init() 里就锁存一次，
         * 走的正是这条路径。 */
        if ((g_cur_state != S_FAULT_NOW) && (g_cur_state != S_FAULT_OVER) &&
            (g_cur_state != S_INIT)) {
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
