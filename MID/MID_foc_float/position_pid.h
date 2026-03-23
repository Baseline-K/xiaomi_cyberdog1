#ifndef _POSITION_PID_H_
#define _POSITION_PID_H_

#include "foc.h"

typedef struct
{
    float kp;
    float ki;
    float err;
//    float sum;
    float output;
    float maxout;
    float minout;
    float up;
		float ui;
    
		float ref;
	  float fbk;
		float ff;
	  float (*FF)();    //PI控制前馈项,由于是可变的，因此这里为函数指针
   // uint16_t pos_update_divider;    // 每 N 次 FOC 运行一次位置环（N>=1）
    uint16_t update_counter;    // 内部计数器
}POSITION_PID_DEF;
extern POSITION_PID_DEF position_pid;

//void position_PID_Control(POSITION_PID_DEF *position_temp);
void position_PID_Init(POSITION_PID_DEF *position_temp,Motor_Params_t Motor_Params);

void Position_Loop_Handle(float Position_ref,float Position_fbk);
#endif
