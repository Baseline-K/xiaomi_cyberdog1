/**
 * identify.h — 离线参数辨识公共类型与框架接口（每参数独立 .c/.h，镜像 AC_Control Para_Identify_Adapt）
 *
 * 框架：任务注册表（init_handler 主循环 + step_handler ISR 每 10kHz）+ Start/Process/FocIsrStep。
 * 复用 Simulink 模型：电压模式（ctrl_mode=3 dq / =4 αβ）+ 电流测量输出（id/iq/iα/iβ）。
 */
#ifndef IDENTIFY_H
#define IDENTIFY_H

#include <stdint.h>

/*-----------------------------------------------------------
 * 辨识状态
 *----------------------------------------------------------*/
typedef enum {
    IDENTIFY_IDLE = 0,
    IDENTIFY_RUNNING,
    IDENTIFY_DONE,
    IDENTIFY_FAILED,
    IDENTIFY_ABORTED,
} Identify_Status_e;

/*-----------------------------------------------------------
 * 任务（每项对应一个独立 .c/.h）
 *----------------------------------------------------------*/
typedef enum {
    TASK_NONE = 0,
    TASK_R,        /* 相电阻（电压斜坡 4 点 LS） */
    TASK_DEAD,     /* 死区 LUT（20 点电流注入） */
    TASK_L,        /* 电感（HFI） */
    TASK_FLUX,     /* 磁链（空载反电动势 LS） */
    TASK_INERTIA,  /* 转动惯量（加减速对称） */
    TASK_COG,      /* 齿槽 LUT（扰动观测器 360 点） */
    TASK_MAX
} Identify_Task_e;

/*-----------------------------------------------------------
 * 辨识结果（RAM 运行时生效）
 *----------------------------------------------------------*/
typedef struct {
    /* 电气 */
    float Rs;                     /* 相电阻 Ω */
    float Ld;                     /* d 轴电感 H */
    float Lq;                     /* q 轴电感 H */
    float Flux_linkage;           /* 磁链 Wb */
    float Ke;                     /* 反电势常数 V·s/rad_mech */
    float Kt;                     /* 转矩常数 N·m/A = 1.5·P·ψf */
    float Vdead_est;              /* 死区误差电压估计 V */
    float rotor_init_elec_angle;  /* 转子初始电角度 rad */
    uint8_t rotor_init_angle_valid;
    /* 力学 */
    float B;                      /* 粘性摩擦 N·m·s/rad */
    float friction_Coulomb;       /* 库伦摩擦 N·m */
    float J;                      /* 转动惯量 kg·m² */
    float J_error;                /* 加/减速一致性相对误差 */
    /* 死区补偿 LUT（20 点，辨识后经 Params_Update 提交到 DeadComp_Lut_I/V） */
    float dead_lut_I[20];
    float dead_lut_V[20];
    int error;
} Identify_Result_t;

/*-----------------------------------------------------------
 * 任务处理函数类型与注册表
 *----------------------------------------------------------*/
typedef struct Identify_Context Identify_Context_t;
typedef void (*Identify_TaskHandler)(Identify_Context_t *ctx);

typedef struct {
    Identify_TaskHandler init_handler;   /* 主循环，进任务时调（复位 FSM、置电压模式前置） */
    Identify_TaskHandler step_handler;   /* ISR 每 10kHz，置 task_done/fail_flag */
} Identify_TaskHandler_t;

extern Identify_TaskHandler_t g_task_handlers[TASK_MAX];

/*-----------------------------------------------------------
 * 上下文
 *----------------------------------------------------------*/
struct Identify_Context {
    Identify_Status_e state;
    uint32_t task_mask;            /* 请求的任务掩码（bit = 1<<TASK_x） */
    Identify_Task_e current_task;
    Identify_Task_e task_list[TASK_MAX];
    int task_total;
    int task_index;
    volatile uint8_t task_done_flag;
    volatile uint8_t task_fail_flag;
    Identify_Result_t res;
};

extern Identify_Context_t g_identify;

/*-----------------------------------------------------------
 * 框架 API
 *----------------------------------------------------------*/
void Identify_Init(void);                          /* 启动时注册全部任务 */
void Identify_Task_Set(uint32_t mask);             /* 设置请求任务掩码 */
void Identify_Start(void);                         /* S_CALIB entry 调用，建任务链 + 初始化第一个任务 */
void Identify_Process(void);                       /* 主循环：推进任务 / DONE / FAILED */
void Identify_FocIsrStep(void);                    /* ISR 每 10kHz：调当前任务 step_handler */
void Identify_Abort(void);
void Identify_End(void);                         /* 离开 CALIB 统一调用：恢复安全配置 + state=IDLE */
Identify_Status_e Identify_GetStatus(void);
uint32_t Identify_GetTaskMask(void);
void Identify_Params_Update(void);                 /* 辨识完成提交结果（RAM + 重算增益） */

/*-----------------------------------------------------------
 * 公共执行原语（identify_apply.c）——复用模型电压模式
 *----------------------------------------------------------*/
void identify_apply_voltage(float vd, float vq);       /* 填 U.vd/vq_ref + ctrl_mode=3（dq 电压模式） */
void identify_apply_alpha_beta(float v_alpha, float v_beta); /* 填 U.v_alpha/v_beta_ref + ctrl_mode=4（αβ 电压模式） */

#endif /* IDENTIFY_H */
