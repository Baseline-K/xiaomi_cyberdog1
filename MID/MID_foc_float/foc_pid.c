#include "foc_pid.h"


CURRENT_PID_DEF current_q_pid;
CURRENT_PID_DEF current_d_pid;
//SPEED_PID_DEF speed_pid;  

static float Limit_Sat( float fdata,float fmax, float fmin) 
{
    fdata = (fdata > fmax) ? fmax : fdata;
    fdata = (fdata < fmin) ? fmin : fdata;

    return fdata;
}

void current_q_pid_Init(CURRENT_PID_DEF *current_temp, Motor_Params_t Motor_Params)
{ 

	//Normal PI Controller
		current_temp->kp = Motor_Params.Phase_L * CurrentLoop_Bandwitch * 6.18f ;  //bandwidth ≈ 200HZ
    current_temp->ki = Motor_Params.Phase_R * CurrentLoop_Bandwitch * 6.18f * timFactor;
	
    current_temp->up = 0.0f;
  	current_temp->ui = 0.0f;
    current_temp->maxout = Motor_Params.VBUS * ONE_BY_SQRT3;
    current_temp->minout = -Motor_Params.VBUS * ONE_BY_SQRT3;
}

void current_d_pid_Init(CURRENT_PID_DEF *current_temp, Motor_Params_t Motor_Params)
{ 

	//Normal PI Controller
		current_temp->kp = Motor_Params.Phase_L * CurrentLoop_Bandwitch * 6.18f ;  //bandwidth ≈ 200HZ
    current_temp->ki = Motor_Params.Phase_R * CurrentLoop_Bandwitch * 6.18f * timFactor;
	
    current_temp->up = 0.0f;
  	current_temp->ui = 0.0f;
    current_temp->maxout = 3.0f;
    current_temp->minout = -3.0f;
}


void current_pid_Control(CURRENT_PID_DEF *current_temp)
{
	  //Normal PI Controller
    current_temp->err = current_temp->ref - current_temp->fbk;
    current_temp->up = current_temp->kp * current_temp->err;
    current_temp->ui = Limit_Sat(
                              current_temp->ui + current_temp->ki *current_temp->err, 
                              current_temp->maxout, 
                              current_temp->minout);
    current_temp->output = Limit_Sat(
                              current_temp->up + current_temp->ui, 
                              current_temp->maxout, 
                              current_temp->minout);
}

