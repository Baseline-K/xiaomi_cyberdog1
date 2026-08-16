#ifndef _speed_pid_h_
#define _speed_pid_h_

#include "foc.h"
              
extern float Speed_Pid_Out;             

typedef struct
{
	float Kp;
  float Ki;
  float Max_Output;
  float Min_Output;
  float I_Sum;
	float error;
	float output;
	float up;
	
	float ref;
	float fbk;
	unsigned int update_count;
}SPEED_PID_DEF;
extern SPEED_PID_DEF speed_pid;

typedef enum {
    LIMIT_NONE = 0x00,
    LIMIT_VEL = 0x01,      // 速度超限
    LIMIT_ACCEL = 0x02,    // 加速度超限
    LIMIT_JERK = 0x04      // 加加速度超限
} LimitFlags;
/* S型曲线参数结构体 */
typedef struct {
    // 用户设置参数
    float target_vel;          // 最终目标速度
    float current_vel;         // 当前目标速度（输出）
    
    float accel_time;          // 加速总时间
    float decel_time;          // 减速总时间
    
  	float Plan_Threshold;      //S型速度规划阈值/目标速度变化多大时重新规划
    // 正转限制（速度 > 0）
    float max_vel_fwd;         // 正转最大速度
    float max_accel_fwd;       // 正转最大加速度
    float max_jerk_fwd;        // 正转最大加加速度
    
    // 反转限制（速度 < 0）
    float max_vel_rev;         // 反转最大速度（绝对值）
    float max_accel_rev;       // 反转最大加速度（绝对值）
    float max_jerk_rev;        // 反转最大加加速度（绝对值）

    // 三段时间参数
    float T1;                  // 加加速时间
    float T2;                  // 匀加速时间
    float T3;                  // 减加速时间
    
    // 内部状态
    float vel_start;           // 本次运动起始速度
    float vel_end;             // 本次运动结束速度
    float time;                // 当前时间计数
    float max_accel;           // 本次运动的最大加速度
    float jerk;                // 本次运动的加加速度
    
    uint8_t limit_flags;       // 限制标志位
    int is_moving;             // 是否正在运动
    int direction;             // 1=加速, -1=减速, 0=静止
} SCurveProfile;



void speed_pid_init(Motor_Params_t Motor_Params, unsigned int update_counter);
void Speed_pid_c(SPEED_PID_DEF* speed_pid_temp);
void speed_Loop_Handle(float speed_ref ,SPEED_PID_DEF* speed_pid_temp, float speedupAc, float speeddownAc);
void Re_speed_init(void);


void SCurve_Init(SCurveProfile *profile, float accel_time, float decel_time);
void SCurve_Plan(SCurveProfile *profile, float new_target);
float SCurve_Update(SCurveProfile *profile, float target_vel, float dt);
#endif                            
