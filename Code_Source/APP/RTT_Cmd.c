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
    printf("  mode <0|1>  mode 0=torque 1=speed\r\n");
    printf("  iq <A>      set target current (torque)\r\n");
    printf("  start       start motor (commit+enable)\r\n");
    printf("  stop        stop motor\r\n");
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
    printf("enable=%u mode=%u speed_rps=%.3f iq_ref_A=%.3f state=%u | FSM=%s posted=%lu dropped=%lu illegal=%lu fault=%lu\r\n",
           (unsigned)MotorCtrl.enable, (unsigned)MotorCtrl.mode,
           (double)MotorCtrl.speed_rps, (double)MotorCtrl.iq_ref_A,
           (unsigned)MotorCtrl.state, sn,
           (unsigned long)posted, (unsigned long)dropped,
           (unsigned long)illegal, (unsigned long)fault_forced);
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
    else if (strcmp(p, "start") == 0)      { post_cmd_event(EVENT_toRUN);  printf("start event posted\r\n"); }
    else if (strcmp(p, "stop") == 0)       { post_cmd_event(EVENT_RUNtoSTOP); printf("stop event posted\r\n"); }
    else if (strcmp(p, "fault") == 0)      { MotorStateMachine_PostFault(0x1U); printf("fault injected (bit0)\r\n"); }
    else if (strcmp(p, "clearfault") == 0) { MotorStateMachine_ClearFault(0x1U); printf("fault cleared\r\n"); }
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
    else if (strcmp(p, "get") == 0)        { print_status(); }
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
