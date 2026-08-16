#ifndef FOC_RUN_H
#define FOC_RUN_H

#include "main_user.h"
#include "foc.h"

/* 使用 Simulink 生成的 FOC 算法(1) 还是手写 FOC(0) —— A/B 对比用 */
#define USE_GENERATED_FOC 1
#include "FOC_generated.h"




#define SHUNT_RESISTENCE	0.1f								// Resistance of phase current sampling resistor
#define V_SCALE (16.0f * 3.3f / 4095.0f)     					// Bus volts per A/D Count (0.015311 V)
#define I_SCALE 0.00161133f   //Amps per A/D Count ：(3.3f / 4095.0f) / SHUNT_RESISTENCE / 5.0f	
//0.0201416f   //Amps per A/D Count ：(3.3f / 4095.0f) / SHUNT_RESISTENCE / 40.0f	





typedef enum {
    CTRL_MODE_OPENLOOP = 0,
    CTRL_MODE_VOLTAGE,
    CTRL_MODE_TORQUE,
    CTRL_MODE_SPEED,
    CTRL_MODE_POSITION,
    // future: CTRL_MODE_CURRENT, etc.
} control_mode_t;

typedef enum {
    RUNSTATE_STOPPED = 0,
    RUNSTATE_STARTING,    // ramp up
    RUNSTATE_RUNNING,     // steady
    RUNSTATE_BRAKING,     // ramp down
    RUNSTATE_STALLED,     // detected stall
    RUNSTATE_CALIBRATING, // e.g. voltage calibration
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
    control_mode_t ctrl_mode;   // 控制模式：速度/扭矩/位置/电压...
    volatile run_state_t run_state; // ISR/foreground shared run state
    direction_t    direction;   // 方向
    run_flags_t    flags;       // 小标志位
    uint32_t       faults;      // 故障位掩码
    // 可附带上下文参数：
//    float          target;      // 目标值（rpm/torque/position/voltage）
//    float          actual;      // 实时量
} MotorState_t;
extern MotorState_t MotorState;




extern volatile int16_t comm[6];
extern  float time_us;
extern  void (* func_ptr)(ADC_HandleTypeDef *hadc);


void Svpwm_Outpwm(void);
uint32_t IF_id_RUN(uint16_t eleangle_step, uint16_t Id_ref);
uint32_t VF_IF_RUN(uint16_t RPM_ref, uint16_t Time_ms);
float VF_IF_RUN_Float(float RPM_ref, float Time_ms);
#endif
