/**
 * identify_framework.c — 离线辨识任务框架（镜像 AC_Control Para_Identify_Adapt）
 *
 * - ISR step 置 task_done_flag / task_fail_flag
 * - 主循环 Identify_Process() 推进到下一任务（并调其 init_handler）
 * - 全部完成 → Identify_Params_Update() 提交结果（RAM 生效 + 重算增益）
 */
#include "identify.h"
#include "identify_r.h"           /* R 任务 */
#include "identify_dead.h"        /* 死区 LUT 任务 */
#include "CyberDog_Motor_FOC.h"   /* 生成模型：U/Y 全局 */
#include "FOC_run.h"              /* MotorState / run_state */
#include "FOC_generated.h"        /* FOC_Generated_Init 重算增益 */
#include "Safety_Module.h"        /* 辨识期间禁用堵转检测 */
#include "foc.h"                  /* Motor_Params */
#include <string.h>
#include <stdio.h>

Identify_Context_t g_identify;
Identify_TaskHandler_t g_task_handlers[TASK_MAX] = {0};

static uint8_t s_saved_enable_stall = 1U;   /* 辨识前保存的堵转检测使能 */

/* 各任务注册（后续 TASK_DEAD/L/FLUX/INERTIA/COG 在此追加） */
static void Identify_Register_All(void)
{
    g_task_handlers[TASK_R].init_handler    = identify_r_Init;
    g_task_handlers[TASK_R].step_handler    = identify_r_Step;
    g_task_handlers[TASK_DEAD].init_handler = identify_dead_Init;
    g_task_handlers[TASK_DEAD].step_handler = identify_dead_Step;
    /* 后续 TASK_L/FLUX/INERTIA/COG 在此追加 */
}

void Identify_Init(void)
{
    memset(&g_identify, 0, sizeof(g_identify));
    memset(g_task_handlers, 0, sizeof(g_task_handlers));
    Identify_Register_All();
}

void Identify_Task_Set(uint32_t mask)
{
    g_identify.task_mask = mask;
}

uint32_t Identify_GetTaskMask(void)
{
    return g_identify.task_mask;
}

/* 按固定顺序建任务链（R 最先；后续 DEAD 复用 R 结果排在 R 后） */
static int build_task_list(uint32_t mask, Identify_Task_e *list)
{
    int idx = 0;
    if (mask & (1u << TASK_R))        list[idx++] = TASK_R;
    if (mask & (1u << TASK_DEAD))     list[idx++] = TASK_DEAD;
    if (mask & (1u << TASK_L))        list[idx++] = TASK_L;
    if (mask & (1u << TASK_FLUX))     list[idx++] = TASK_FLUX;
    if (mask & (1u << TASK_INERTIA))  list[idx++] = TASK_INERTIA;
    if (mask & (1u << TASK_COG))      list[idx++] = TASK_COG;
    return idx;
}

void Identify_Start(void)
{
    /* 辨识期间禁用堵转检测（锁转子/注入电流是设计行为，非真堵转；servo_hub 亦无堵转检测） */
    s_saved_enable_stall = Safety_Config.enable_stall;
    Safety_Config.enable_stall = 0U;

    if (g_identify.task_mask == 0U) {
        g_identify.state = IDENTIFY_DONE;   /* 无任务：直接完成 */
        g_identify.current_task = TASK_NONE;
        return;
    }
    g_identify.task_total = build_task_list(g_identify.task_mask, g_identify.task_list);
    g_identify.task_index = 0;
    g_identify.current_task = TASK_NONE;
    g_identify.state = IDENTIFY_RUNNING;
    g_identify.task_done_flag = 0;
    g_identify.task_fail_flag = 0;

    if (g_identify.task_total > 0) {
        g_identify.current_task = g_identify.task_list[0];
        if (g_task_handlers[g_identify.current_task].init_handler) {
            g_task_handlers[g_identify.current_task].init_handler(&g_identify);
        }
    }
}

void Identify_Process(void)
{
    if (g_identify.state != IDENTIFY_RUNNING) return;

    /* 失败 */
    if (g_identify.task_fail_flag) {
        g_identify.task_fail_flag = 0;
        g_identify.state = IDENTIFY_FAILED;
        g_identify.current_task = TASK_NONE;
        printf("IDENTIFY: FAILED\r\n");
        return;
    }
    /* 等待当前任务完成 */
    if (!g_identify.task_done_flag) return;
    g_identify.task_done_flag = 0;

    g_identify.task_index++;
    if (g_identify.task_index >= g_identify.task_total) {
        g_identify.state = IDENTIFY_DONE;
        g_identify.current_task = TASK_NONE;
        printf("IDENTIFY: DONE (Rs=%.4f)\r\n", (double)g_identify.res.Rs);
    } else {
        g_identify.current_task = g_identify.task_list[g_identify.task_index];
        if (g_task_handlers[g_identify.current_task].init_handler) {
            g_task_handlers[g_identify.current_task].init_handler(&g_identify);
        }
    }
}

/* ISR 每 10kHz：调当前任务的 step_handler（填模型电压输入 + 采样读数） */
void Identify_FocIsrStep(void)
{
    if (g_identify.state != IDENTIFY_RUNNING) return;
    Identify_Task_e task = g_identify.current_task;
    if (task < TASK_MAX && g_task_handlers[task].step_handler) {
        g_task_handlers[task].step_handler(&g_identify);
    }
}

void Identify_Abort(void)
{
    g_identify.state = IDENTIFY_ABORTED;
    g_identify.current_task = TASK_NONE;
    /* 释放电压注入：下次正常 step 前由 FOC_Generated_Step 恢复正常 ctrl_mode */
    printf("IDENTIFY: ABORTED\r\n");
}

/* 离开 CALIB 统一调用：恢复辨识期间禁用的堵转检测 */
void Identify_End(void)
{
    Safety_Config.enable_stall = s_saved_enable_stall;
    g_identify.state = IDENTIFY_IDLE;
    g_identify.current_task = TASK_NONE;
}

Identify_Status_e Identify_GetStatus(void)
{
    return g_identify.state;
}

/* 辨识完成：结果提交到 Motor_Params / LUT，并重算模型增益（RAM 生效） */
void Identify_Params_Update(void)
{
    uint32_t mask = g_identify.task_mask;
    int gains_changed = 0;

    if (mask & (1u << TASK_R)) {
        if (g_identify.res.Rs > 0.0f) {
            Motor_Params.Phase_R = g_identify.res.Rs;
            printf("IDENTIFY: Phase_R -> %.4f ohm\r\n", (double)Motor_Params.Phase_R);
            gains_changed = 1;
        }
    }
    /* 先重算增益（FOC_Generated_Init 会重置死区/齿槽使能与占位 LUT，须在其后再提交） */
    if (gains_changed) FOC_Generated_Init();

    /* 死区 LUT 提交（在 Init 之后，避免被占位覆盖）+ 使能 */
    if (mask & (1u << TASK_DEAD)) {
        for (int k = 0; k < DEAD_LUT_N; k++) {
            DeadComp_Lut_I[k] = g_identify.res.dead_lut_I[k];
            DeadComp_Lut_V[k] = g_identify.res.dead_lut_V[k];
        }
        DeadComp_En = 1.0f;
        printf("IDENTIFY: DeadComp LUT committed (Vdead@max=%.3f), DeadComp_En=1\r\n",
               (double)DeadComp_Lut_V[DEAD_LUT_N - 1]);
    }
    /* 后续 TASK_L/FLUX/INERTIA/COG 在此追加 */
}
