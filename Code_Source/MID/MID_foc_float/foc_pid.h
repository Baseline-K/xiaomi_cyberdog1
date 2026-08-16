#ifndef _FOC_PID_H_
#define _FOC_PID_H_

#include "foc.h"

#define Lpf(a,a_last,lpf) (a * lpf + a_last * (1 - lpf))

typedef struct
{
    float kp;
    float ki;
    float err;
    float sum;
    float output;
    float maxout;
    float minout;
    float up;
		float ui;
	
	  float ref;
	  float fbk;
}CURRENT_PID_DEF;
extern CURRENT_PID_DEF current_q_pid;
extern CURRENT_PID_DEF current_d_pid;




void current_q_pid_Init(CURRENT_PID_DEF *current_temp, Motor_Params_t Motor_Params);
void current_d_pid_Init(CURRENT_PID_DEF *current_temp, Motor_Params_t Motor_Params);
void current_pid_Control(CURRENT_PID_DEF *current_temp);

#endif
