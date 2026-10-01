/**
 * RTT_Cmd —— RTT 命令解释器（APP，实时电机控制）
 *
 * 用 RTT Viewer 输入栏输入文本命令，主循环非阻塞解析并调 MotorCtrl，
 * 实现不停机实时控电机（比调试器暂停-改-恢复安全）。
 *
 * 命令：
 *   spd <rps>  设置目标速度   mode <0|1> 设置模式
 *   iq <A>     设置目标电流   start/stop 启停
 *   get        打印状态        help       打印命令表
 *
 * 通道：RTT 下行通道 0（终端）。与 J-Scope（通道2）隔离，无冲突。
 */
#include "RTT_Cmd.h"
#include "SEGGER_RTT.h"
#include "MotorCtrl.h"
#include "motor_state_machine.h"
#include "Safety_Module.h"
#include "foc.h"   /* foc_abc_current_i */
#include "app_tasks.h"   /* AppTasks_Diag */
#include "identify.h"   /* 离线辨识命令 */
#include "FOC_generated.h"   /* 速度 S 曲线规划 / 前馈开关 */
#include "FOC_run.h"   /* Vbus / V_SCALE（get 命令的母线电压自检） */
#include "Encoder_Adapt.h"   /* 编码器状态自检 */
/* 直接引用 AS5047 驱动只为一件事：打印阻塞读超时的时基状态 as5047_dwt_state。
 * 适配层是给"换编码器不动别处"用的，但"超时判据有没有失效"是驱动内部的
 * 安全属性，不该被适配层抽象掉 —— 所以这里刻意穿透一层。 */
#include "AS5047.h"
#include "Task_Watchdog.h"  /* 复位原因 + 任务看门狗自检 */
#include "main_user.h"      /* Hall_ADC_DMA / Vbus_ADC_DMA（双路径电压与霍尔原始码自检） */
#include "main.h"   /* HAL_GetTick */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RTT_CMD_LINE_MAX  64U

static char    cmd_line[RTT_CMD_LINE_MAX];
static uint8_t cmd_len = 0U;

static void print_help(void)
{
    printf("\r\n==== Motor Control Commands ====\r\n");
    printf("  spd <rps>   set target speed   e.g. spd 5\r\n");
    printf("  mode <0|1|2> 0=torque 1=speed 2=position\r\n");
    printf("  iq <A>      set target current (torque)\r\n");
    printf("  pos <rad>   set target position (rad, multi-turn)\r\n");
    printf("  start       start motor (commit+enable)\r\n");
    printf("  stop        stop motor\r\n");
    printf("  identify r    offline R identification\r\n");
    printf("  identify dead offline dead-time LUT identification (needs R first)\r\n");
    printf("  identify l    offline inductance (HFI) identification\r\n");
    printf("  identify flux offline flux linkage identification (motor spins no-load)\r\n");
    printf("  identify pole  offline pole-pairs/direction (voltage drag fwd+rev)\r\n");
    printf("  identify j     offline rotor inertia (accel/decel, needs flux first)\r\n");
    printf("  identify abort\r\n");
    printf("  plT <ms>       S-curve planner accel/decel time (e.g. 300)\r\n");
    printf("  lesoBw <hz>    LESO observer bandwidth (0=off) e.g. 60\r\n");
    printf("  showparams  print motor params + identify results\r\n");
    printf("  get         print status\r\n");
    printf("  help        show this\r\n");
    printf("===============================\r\n");
}

static void print_status(void)
{
    uint32_t posted, dropped, illegal, fault_forced;
    const char *sn = "S_?";
    switch (MotorState_GetCurrent()) {
    case S_INIT:       sn = "S_INIT";       break;
    case S_IDLE:       sn = "S_IDLE";       break;
    case S_CALIB:      sn = "S_CALIB";      break;
    case S_RUN:        sn = "S_RUN";        break;
    case S_STOP:       sn = "S_STOP";       break;
    case S_FAULT_NOW:  sn = "S_FAULT_NOW";  break;
    case S_FAULT_OVER: sn = "S_FAULT_OVER"; break;
    case S_STANDBY:    sn = "S_STANDBY";    break;
    default:           break;
    }
    MotorState_GetStats(&posted, &dropped, &illegal, &fault_forced);
    printf("enable=%u mode=%u speed_rps=%.3f iq_ref_A=%.3f pos_ref=%.3f state=%u | FSM=%s posted=%lu dropped=%lu illegal=%lu fault=%lu\r\n",
           (unsigned)MotorCtrl.enable, (unsigned)MotorCtrl.mode,
           (double)MotorCtrl.speed_rps, (double)MotorCtrl.iq_ref_A,
           (double)MotorCtrl.pos_ref, (unsigned)MotorCtrl.state, sn,
           (unsigned long)posted, (unsigned long)dropped,
           (unsigned long)illegal, (unsigned long)fault_forced);
    /* 母线电压（16:1 分压满量程 52.8V，24V 应约 1861 码）：
     * 来源 = ADC3 注入 rank2（PC3 / ADC3_IN13），与两个电流同被 TIM1_TRGO 触发、
     * PWM 同步采样，由 FOC ISR 读进 Vbus。慢安全用的是同一个值。 */
    printf("  vbus=%.2fV  adc=%u  full-scale=%.1fV  ov_hard=%.1fV ov_soft=%.1fV uv=%.1fV\r\n",
           (double)Vbus,
           (unsigned)((Vbus / V_SCALE) + 0.5f),
           (double)(16.0f * 3.3f),
           (double)Safety_Config.v_bus_over_V,
           (double)Safety_Config.vbus_over_soft_V,
           (double)Safety_Config.v_bus_under_V);
    /* 输出端 6 路线性霍尔原始码（1kHz 扫描，rank 顺序 = PA4 PA5 PA6 PA7 PC4 PC5）。
     * 静止时应都在中点附近（约 2048）且 6 路不全相同；手动转输出轴时应看到
     * 6 路依次呈正弦变化。全 0 = ADC1 规则组没被 SWSTART 启动；
     * 6 路完全相同 = DMA 长度不对或只采到了一个通道。 */
    printf("  hall: %u %u %u %u %u %u\r\n",
           (unsigned)Hall_ADC_DMA[0], (unsigned)Hall_ADC_DMA[1],
           (unsigned)Hall_ADC_DMA[2], (unsigned)Hall_ADC_DMA[3],
           (unsigned)Hall_ADC_DMA[4], (unsigned)Hall_ADC_DMA[5]);
    /* 电流采样链自检：
     *   cal_done=1 表示上电零偏校准采满 OFFSET_CAL_SAMPLES 拍；0 = 超时失败
     *   （已退回中点 2048 并上报 FAULT_CURRENT_SENSE_MASK，且 main_user 会跳过
     *   强拖和编码器零位校准）。n = 实际采到的拍数。
     *   ib/ic 是实测零偏码值，静止无电流时应都在 ~2048（1.65V 中点）附近，
     *   两者差几十个码属正常（运放/采样电阻容差）。 */
    printf("  cur: cal_done=%u n=%u  offset ib=%ld ic=%ld (mid=%u)\r\n",
           (unsigned)g_offset_cal_done,
           (unsigned)g_offset_cal_count,
           (long)foc_abc_current_i.offset_phase_ib,
           (long)foc_abc_current_i.offset_phase_ic,
           (unsigned)2048U);
    /* 编码器自检：err 是通信错误累计，streak 是"连续取不到样本"的当前拍数
     * （到 ENCODER_FAULT_STREAK_N 就报 FAULT_ENCODER_MASK 并清零重计时），
     * posts 是累计上报次数。streak 持续增长 = 编码器有问题。 */
    printf("  enc=%s status=%d err=%lu miss=%lu streak=%lu/%u posts=%lu\r\n",
           Encoder_GetName(),
           (int)Encoder_GetStatus(),
           (unsigned long)Encoder.data.error_cnt,
           (unsigned long)Encoder.data.miss_cnt,
           (unsigned long)Encoder.fault_streak,
           (unsigned)ENCODER_FAULT_STREAK_N,
           (unsigned long)Encoder.fault_posts);
    /* 阻塞读超时的时基状态（AS5047.h 的 as5047_dwt_state）：
     *   1 = DWT 周期计数器在跑，使用微秒级帧截止时间（另有轮询次数硬上限）；
     *   2 = **没在跑**，超时已退化成"最大轮询次数"兜底 —— 不会死等，但精度很差。
     *   0 = 还没探测（编码器没初始化）。正常必须恒为 1；出现 2 说明 DWT 被谁关了，
     *   同时 FOC ISR 的 WCET 也会是 0，别拿那时的 WCET 数字去做优化决策。 */
    printf("  enc-timebase: dwt=%u (0=untested 1=ok 2=fallback)\r\n",
           (unsigned)as5047_dwt_state);
    printf("  enc-stream: mode=%u real=%lu age=%luus prime=%lu timeout=%lu stale=%lu read=%lu/max=%lu cycles\r\n",
           (unsigned)AS5047_READ_MODE,
           (unsigned long)Encoder_AS5047.valid_sample_seq,
           (unsigned long)Encoder_AS5047.sample_age_us,
           (unsigned long)Encoder_AS5047.priming_frames,
           (unsigned long)Encoder_AS5047.transfer_timeouts,
           (unsigned long)Encoder_AS5047.stale_samples,
           (unsigned long)Encoder_AS5047.read_cycles,
           (unsigned long)Encoder_AS5047.max_read_cycles);
    /* 速度预测补帧自检（驱动读失败时用上一拍速度外推一帧角度，保证喂给模型 PLL
     * 的角度连续，避免 PLL 相位跟踪器在"输入冻结"时速度先过冲再塌陷）：
     *   pred = 累计补帧数（缓慢增长 = 偶发丢帧被平滑掉了）
     *   now  = 当前连续补帧数（持续不为 0 = 链路不稳）
     *   max  = 历史最长连续补帧数（到 window 就是"预测额度用尽→报故障"那一刻）
     * 补帧**不算**健康样本，仍计入 streak，所以 streak 与 now 会同步增长。 */
    printf("  enc-pred: pred=%lu now=%lu max=%lu (window=%u frames)\r\n",
           (unsigned long)Encoder.data.predicted_cnt,
           (unsigned long)Encoder.data.predict_streak,
           (unsigned long)Encoder.data.max_predict_streak,
           (unsigned)ENCODER_FAULT_STREAK_N);
    /* 看门狗自检：
     *   reset=     上一次复位原因（IWDG_TIMEOUT 表示上一轮有喂狗点漏了）
     *   stall=     任务看门狗累计判定卡死次数（正常恒为 0）
     *   rebuild=   卡死的非关键任务自动重建次数（正常恒为 0）
     * 详细分任务计数见 RTT `diag` 命令。 */
    printf("  wdg: reset=%s was_iwdg=%u stall=%lu rebuild=%lu\r\n",
           Twdt_ResetCauseName(Twdt_GetResetCause()),
           (unsigned)Twdt_IsWatchdogReset(),
           (unsigned long)Twdt_GetStallCount(),
           (unsigned long)Twdt_GetRebuildCount());
}

/* 把命令构造为事件投递（参数取自 MotorCtrl 影子值，3b 由状态机提交快照） */
static void post_cmd_event(motor_event_id_t id)
{
    MotorEvent_t evt = { 0 };
    evt.id          = id;
    evt.timestamp_ms = HAL_GetTick();
    evt.mode        = MotorCtrl.mode_temp;
    evt.speed_rps   = MotorCtrl.speed_rps_temp;
    evt.iq_ref_A    = MotorCtrl.iq_ref_A_temp;
    evt.pos_ref     = MotorCtrl.pos_ref_temp;
    MotorStateMachine_PostEvent(&evt);
}

static void handle_line(char *line)
{
    char *p = line;
    while ((*p == ' ') || (*p == '\t')) p++;   /* 跳过前导空白 */

    if      (strncmp(p, "spd ", 4U) == 0)  { MotorCtrl_SetSpeedRPS((float)atof(p + 4)); printf("speed set to %.3f RPS\r\n", (double)MotorCtrl.speed_rps_temp); }
    else if (strncmp(p, "mode ", 5U) == 0) { MotorCtrl_SetMode((uint8_t)atoi(p + 5));   printf("mode set to %u\r\n", (unsigned)MotorCtrl.mode_temp); }
    else if (strncmp(p, "iq ", 3U) == 0)   { MotorCtrl_SetTorqueA((float)atof(p + 3));  printf("iq set to %.3f A\r\n", (double)MotorCtrl.iq_ref_A_temp); }
    else if (strncmp(p, "pos ", 4U) == 0)  { MotorCtrl_SetPosition((float)atof(p + 4)); printf("pos set to %.3f rad\r\n", (double)MotorCtrl.pos_ref_temp); }
    else if (strcmp(p, "start") == 0)      { post_cmd_event(EVENT_toRUN);  printf("start event posted\r\n"); }
    else if (strcmp(p, "stop") == 0)       { post_cmd_event(EVENT_RUNtoSTOP); printf("stop event posted\r\n"); }
    else if (strcmp(p, "fault") == 0)      { MotorStateMachine_PostFault(0x1U); printf("fault injected (bit0)\r\n"); }
    /* clearfault 已移除：故障为全自动恢复（FAULT_NOW->FAULT_OVER->IDLE），无需手动清 */
    else if (strcmp(p, "calib") == 0)      { post_cmd_event(EVENT_IDLEtoCALIB); printf("calib event posted\r\n"); }
    /* 安全检测测试强制（调试用） */
    else if (strcmp(p, "safedbg overcurrent") == 0) { Safety_TestForce(SAFETY_TEST_FORCE_OVERCURRENT); printf("safedbg: force overcurrent\r\n"); }
    else if (strcmp(p, "safedbg overvolt") == 0)    { Safety_TestForce(SAFETY_TEST_FORCE_OVERVOLT);    printf("safedbg: force overvolt\r\n"); }
    else if (strcmp(p, "safedbg undervolt") == 0)   { Safety_TestForce(SAFETY_TEST_FORCE_UNDERVOLT);   printf("safedbg: force undervolt\r\n"); }
    else if (strcmp(p, "safedbg clear") == 0)       { Safety_TestClear(0xFFFFFFFFUL); MotorStateMachine_ClearFault(0xFFFFFFFFUL); printf("safedbg: force+latched cleared\r\n"); }
    /* 安全阈值调参（bring-up 校准用，确认后写回默认值） */
    else if (strncmp(p, "safethr overcurrent ", 20U) == 0) { Safety_Config.current_over_A = (float)atof(p + 20); printf("overcurrent thr=%.2fA\r\n", (double)Safety_Config.current_over_A); }
    else if (strncmp(p, "safethr short ", 14U) == 0)      { Safety_Config.shortcircuit_A = (float)atof(p + 14);    printf("short thr=%.2fA\r\n", (double)Safety_Config.shortcircuit_A); }
    else if (strcmp(p, "cur") == 0)                      { printf("ia=%.3f ib=%.3f ic=%.3f\r\n", (double)foc_abc_current_i.ia, (double)foc_abc_current_i.ib, (double)foc_abc_current_i.ic); }
    else if (strcmp(p, "diag") == 0)                     { AppTasks_Diag(); }
    else if (strcmp(p, "get") == 0)        { print_status(); }
    /* 离线参数辨识：identify r 等；复用 S_CALIB（先 stop 再 identify） */
    else if (strncmp(p, "identify", 8U) == 0) {
        char *arg = p + 8;
        while ((*arg == ' ') || (*arg == '\t')) arg++;
        if (strcmp(arg, "r") == 0) {
            Identify_Task_Set(1u << TASK_R);
            post_cmd_event(EVENT_IDLEtoCALIB);
            printf("identify: R task set, entering CALIB\r\n");
        } else if (strcmp(arg, "dead") == 0) {
            Identify_Task_Set(1u << TASK_DEAD);
            post_cmd_event(EVENT_IDLEtoCALIB);
            printf("identify: DEAD task set, entering CALIB\r\n");
        } else if (strcmp(arg, "l") == 0) {
            Identify_Task_Set(1u << TASK_L);
            post_cmd_event(EVENT_IDLEtoCALIB);
            printf("identify: L (HFI) task set, entering CALIB\r\n");
        } else if (strcmp(arg, "flux") == 0) {
            Identify_Task_Set(1u << TASK_FLUX);
            post_cmd_event(EVENT_IDLEtoCALIB);
            printf("identify: FLUX task set, entering CALIB\r\n");
        } else if (strcmp(arg, "pole") == 0) {
            Identify_Task_Set(1u << TASK_POLE);
            post_cmd_event(EVENT_IDLEtoCALIB);
            printf("identify: POLE task set, entering CALIB\r\n");
        } else if (strcmp(arg, "j") == 0) {
            Identify_Task_Set(1u << TASK_INERTIA);
            post_cmd_event(EVENT_IDLEtoCALIB);
            printf("identify: J (inertia) task set, entering CALIB\r\n");
        } else if (strcmp(arg, "abort") == 0) {
            Identify_Abort();
            printf("identify: abort requested\r\n");
        } else {
            printf("identify <r|l|flux|pole|j|dead|cog|abort>  (先 stop 再辨识)\r\n");
        }
    }
    else if (strncmp(p, "plT ", 4U) == 0) {
        uint32_t ms = (uint32_t)atoi(p + 4);
        FOC_SpeedPlan_SetTimeMs(ms);
        printf("S-curve accel/decel time -> %u ms\r\n", (unsigned)ms);
    }
    else if (strncmp(p, "lesoBw ", 7U) == 0) {
        FOC_Leso_SetBwHz((float)atof(p + 7));
        printf("LESO bandwidth -> %.1f Hz (0=off)\r\n", (double)FOC_Leso_GetBwHz());
    }
    else if (strcmp(p, "showparams") == 0) {
        printf("VBUS=%.1f R=%.4f Ld=%.5f Lq=%.5f Flux=%.6f P=%.0f J=%.8f\r\n",
               (double)Motor_Params.VBUS, (double)Motor_Params.Phase_R,
               (double)Motor_Params.Ld, (double)Motor_Params.Lq,
               (double)Motor_Params.Flux,
               (double)Motor_Params.Pole_Pairs, (double)Motor_Params.Rotor_inertia);
        printf("friction: B_nm_rps=%.6f fc_nm=%.6f\r\n",
               (double)Motor_Params.friction_B_nm_rps,
               (double)Motor_Params.friction_Coulomb_nm);
        printf("LESO: bw=%.1f Hz (0=off)\r\n", (double)FOC_Leso_GetBwHz());
        printf("ident: state=%d Rs=%.4f Vdead=%.4f\r\n",
               (int)Identify_GetStatus(), (double)g_identify.res.Rs,
               (double)g_identify.res.Vdead_est);
    }
    else if (strcmp(p, "help") == 0)       { print_help(); }
    else if (p[0] != '\0')                 { printf("unknown cmd: '%s' (type help)\r\n", p); }
}

void RTT_Cmd_Process(void)
{
    int c;
    /* 非阻塞逐字符读 RTT 下行通道 0（GetKey 固定通道 0） */
    while ((c = SEGGER_RTT_GetKey()) >= 0) {
        if ((c == '\r') || (c == '\n')) {
            cmd_line[cmd_len] = '\0';
            if (cmd_len > 0U) handle_line(cmd_line);
            cmd_len = 0U;
        } else if (cmd_len < (RTT_CMD_LINE_MAX - 1U)) {
            cmd_line[cmd_len++] = (char)c;
        }
        /* 超长：丢弃 */
    }
}
