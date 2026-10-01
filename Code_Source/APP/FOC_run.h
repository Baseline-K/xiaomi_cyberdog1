#ifndef FOC_RUN_H
#define FOC_RUN_H

#include "main_user.h"
#include "foc.h"

/* 使用 Simulink 生成的 FOC 算法(1) 还是手写 FOC(0) —— A/B 对比用 */
#define USE_GENERATED_FOC 1
#include "FOC_generated.h"




#define SHUNT_RESISTENCE	0.001f								// 相电流采样电阻 1 mΩ（新版硬件；必须 Kelvin 四线接法）
#define V_SCALE (16.0f * 3.3f / 4095.0f)     					// 12.894 mV/码；16:1分压+3.3V基准 -> 满量程 52.8V，24V≈1861码
#define CSA_GAIN_VV        20.0f       /* DRV8323 电流放大器增益 V/V。
                                        * !! 必须与 drv83xx.c 里 .CSA_GAIN = CSA_GAIN_20 一致 !!
                                        * 不一致则所有电流读数/过流阈值成比例错，是最危险的配置错误。
                                        * main_user 启动时会读回 CSACR 做一致性检查并打印。 */
#define I_SCALE  ((3.3f / 4095.0f) / (SHUNT_RESISTENCE * CSA_GAIN_VV))	
/* Amps per A/D Count = (3.3/4095) / (Rs × Gain)
 *   = 8.0586e-4 / (0.001 × 20) = 4.02931e-2 A/码（40.3 mA/码）
 * 双向采样（偏置 VREF/2 = 1.65V）量程 = ±1.65 / (0.001 × 20) = ±82.5 A
 *   20 A（Current_Rating 峰值额定）  =  496 码
 *   40 A（Current_Max）             =  993 码
 *   48 A（软件过流 Current_Max×1.2）= 1191 码（58% 满量程）
 *   60 A（软件短路 ×1.5）           = 1489 码
 * 分流电阻功耗：20 A 峰值 0.4 W / 40 A 峰值 1.6 W → 选 2 W 的 2512/2725。
 * 注意：改 Rs / 增益后所有电流阈值全部跟着变，必须重新跑 identify r → l → flux → j。 */	





/* 工作模式（mt_ 风格，用户定；辨识时用 MT_IDENTIFY） */
typedef enum {
    MT_STOP = 0,
    MT_OPENLOOP_CURRENT,
    MT_OPENLOOP_VOLTAGE,
    MT_VOLTAGE,
    MT_TORQUE,
    MT_SPEED,
    MT_POSITION,
    MT_IDENTIFY,
} motor_work_mode_t;

typedef enum {
    RUNSTATE_STOPPED = 0,
    RUNSTATE_STARTING,    // ramp up
    RUNSTATE_RUNNING,     // steady
    RUNSTATE_BRAKING,     // ramp down
    RUNSTATE_STALLED,     // detected stall
    RUNSTATE_CALIBRATING, // e.g. voltage calibration
    RUNSTATE_IDENTIFYING, // 离线参数辨识（复用 S_CALIB）
    RUNSTATE_LOCKED,      // locked state
} run_state_t;

typedef enum {
    DIR_UNKNOWN = 0,
    DIR_CW,
    DIR_CCW,
} direction_t;

/* 故障位掩码（可扩展）*/
typedef enum {
    FAULT_NONE         = 0,
    FAULT_OVERVOLTAGE  = 1<<0,
    FAULT_UNDERVOLTAGE = 1<<1,
    FAULT_OVERCURRENT  = 1<<2,
    FAULT_STALL        = 1<<3,
    FAULT_SENSOR       = 1<<4,
    // ...
} fault_bits_t;

/* 一些运行标志 */
typedef struct {
    uint8_t calibrated : 1;
    uint8_t homed      : 1;
    uint8_t servo_on   : 1;
    uint8_t reserved   : 5;
} run_flags_t;

/* 主状态结构体 */
typedef struct {
    motor_work_mode_t ctrl_mode;   // 工作模式：MT_TORQUE/SPEED/POSITION/IDENTIFY...
    volatile run_state_t run_state; // ISR/foreground shared run state
    direction_t    direction;   // 方向
    run_flags_t    flags;       // 小标志位
    uint32_t       faults;      // 故障位掩码
    // 可附带上下文参数：
//    float          target;      // 目标值（rpm/torque/position/voltage）
//    float          actual;      // 实时量
} MotorState_t;
extern MotorState_t MotorState;

extern float g_mech_pos_rad;   /* 连续机械角(rad, 多圈)，10kHz ISR 累加 */
extern float Vbus;             /* 母线电压实测值 V（FOC ISR 每拍更新） */




extern volatile int16_t comm[6];
extern  float time_us;
extern  void (* func_ptr)(ADC_HandleTypeDef *hadc);


void Svpwm_Outpwm(void);
uint32_t IF_id_RUN(uint16_t eleangle_step, uint16_t Id_ref);
uint32_t VF_IF_RUN(uint16_t RPM_ref, uint16_t Time_ms);
float VF_IF_RUN_Float(float RPM_ref, float Time_ms);
#endif
