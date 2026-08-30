#ifndef MOTORCTRL_H
#define MOTORCTRL_H

#include <stdint.h>

/* 工作模式（映射到生成模型的 ctrl_mode：0=转矩,1=速度,2=位置） */
typedef enum {
    MC_MODE_TORQUE = 0,
    MC_MODE_SPEED = 1,
    MC_MODE_POSITION = 2      /* 位置环（模型内级联：位置→速度→电流） */
} MotorCtrl_Mode_t;

typedef enum {
    MC_STATE_STOPPED = 0,
    MC_STATE_RUNNING = 1,
    MC_STATE_FAULT   = 2
} MotorCtrl_State_t;

/* 安全限幅 */
#define MOTOR_SPEED_LIMIT_RPS  40.0f    /* = 额定 2400 RPM */
#define MOTOR_IREF_LIMIT_A     8.0f

/*
 * 统一控制结构体（CAN 与 J-Link 都改这一份，FOC ISR 读命令字段）：
 *   - 影子参数 *_temp：主循环里 CAN 写，Start() 一次性提交到命令字段
 *   - 命令字段 enable/mode/speed_rps/iq_ref_A/pos_ref：ISR 读；
 *     J-Link 调试可直接改这些（实时生效），或改 *_temp 后调 Start()
 */
typedef struct {
    /* 影子参数（主循环 CAN 写） */
    uint8_t  mode_temp;
    float    speed_rps_temp;
    float    iq_ref_A_temp;
    float    pos_ref_temp;
    /* 命令（FOC ISR 读；J-Link 直接改） */
    volatile uint8_t  enable;      /* 1=驱动，0=停止 */
    volatile uint8_t  mode;        /* MotorCtrl_Mode_t */
    volatile float    speed_rps;   /* 目标速度 (RPS) */
    volatile float    iq_ref_A;    /* 目标电流 (A, 转矩模式) */
    volatile float    pos_ref;     /* 目标位置 (rad, 预留) */
    /* 状态 */
    uint8_t  state;                /* MotorCtrl_State_t */
    uint32_t faults;
} MotorCtrl_t;
extern volatile MotorCtrl_t MotorCtrl;

void MotorCtrl_Init(void);
void MotorCtrl_SetMode(uint8_t mode);
void MotorCtrl_SetSpeedRPS(float rps);
void MotorCtrl_SetTorqueA(float a);
void MotorCtrl_SetPosition(float rad);
void MotorCtrl_Start(void);         /* 提交全部影子 → 使能 */
void MotorCtrl_Stop(void);
void MotorCtrl_Process(void);       /* 主循环 1ms（预留：看门狗等） */
void MotorCtrl_GetStatus(uint8_t out[4]);

/* FOC sampling remains active while the motor is stopped. */
void FOC_Interrupt_Start(void);
void FOC_Interrupt_Stop(void);

/* Three-phase PWM and DRV83xx gate-output control. */
void PowerStage_Enable(void);
void PowerStage_Disable(void);

#endif /* MOTORCTRL_H */
