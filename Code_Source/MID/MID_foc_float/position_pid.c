#include "position_pid.h"


static float Limit_Sat( float fdata,float fmax, float fmin) 
{
    fdata = (fdata > fmax) ? fmax : fdata;
    fdata = (fdata < fmin) ? fmin : fdata;

    return fdata;
}

void position_PID_Control(POSITION_PID_DEF *position_temp)
{	
	//Normal PI Controller
    position_temp->err = position_temp->ref - position_temp->fbk;
	
	  //Feedforward term :(optional; if not used, set pV->FF return)
     position_temp->ff =  position_temp->FF();    
	
    position_temp->up = position_temp->kp * position_temp->err;
    position_temp->ui = Limit_Sat(
                              position_temp->ui + position_temp->ki * position_temp->err, 
                              position_temp->maxout, 
                              position_temp->minout);
    position_temp->output = Limit_Sat(
                              position_temp->up + position_temp->ui + position_temp->ff, 
                              position_temp->maxout, 
                              position_temp->minout);
}

float pi_Position_FF(void)
{
	
	return  0;
}

void position_PID_Init(POSITION_PID_DEF *position_temp,Motor_Params_t Motor_Params)
{
    position_temp->kp = 0.2f;   //SpeedLoop_Bandwitch/6.0f /(2*PI)  //rad/s -> r/s
	  position_temp->ki = 0.0f;
	  position_temp->FF = pi_Position_FF;
	
	  position_temp->up = 0.0f;
    position_temp->ui = 0.0f;
    position_temp->maxout = Motor_Params.RPM_Rating/60 * 0.5f;
    position_temp->minout = -Motor_Params.RPM_Rating/60 * 0.5f;
}


POSITION_PID_DEF position_pid;
void Position_Loop_Handle(float Position_ref,float Position_fbk)
{
	position_pid.update_counter++;
	if(position_pid.update_counter >= 5*N)
	{
    	position_pid.update_counter = 0;
		  
//		  float Position_fbk;
//			Position_fbk = encod.CCW_angle_total - encod.CW_angle_total;
		  position_pid.ref = Position_ref; 
		  position_pid.fbk = Position_fbk;
		  position_PID_Control(&position_pid);

	}
}






