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
    printf("enable=%u mode=%u speed_rps=%.3f iq_ref_A=%.3f state=%u\r\n",
           (unsigned)MotorCtrl.enable, (unsigned)MotorCtrl.mode,
           (double)MotorCtrl.speed_rps, (double)MotorCtrl.iq_ref_A,
           (unsigned)MotorCtrl.state);
}

static void handle_line(char *line)
{
    char *p = line;
    while ((*p == ' ') || (*p == '\t')) p++;   /* 跳过前导空白 */

    if      (strncmp(p, "spd ", 4U) == 0)  { MotorCtrl_SetSpeedRPS((float)atof(p + 4)); printf("speed set to %.3f RPS\r\n", (double)MotorCtrl.speed_rps_temp); }
    else if (strncmp(p, "mode ", 5U) == 0) { MotorCtrl_SetMode((uint8_t)atoi(p + 5));   printf("mode set to %u\r\n", (unsigned)MotorCtrl.mode_temp); }
    else if (strncmp(p, "iq ", 3U) == 0)   { MotorCtrl_SetTorqueA((float)atof(p + 3));  printf("iq set to %.3f A\r\n", (double)MotorCtrl.iq_ref_A_temp); }
    else if (strcmp(p, "start") == 0)      { MotorCtrl_Start();  printf("motor STARTED (enable=%u)\r\n", (unsigned)MotorCtrl.enable); }
    else if (strcmp(p, "stop") == 0)       { MotorCtrl_Stop();   printf("motor STOPPED\r\n"); }
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
