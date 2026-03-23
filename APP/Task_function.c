//############################################################
// FILE: Task_function.c
// Created on: 2017年1月18日
// Author: XQ
// summary: Task_function
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#include "Task_function.h"
#include "Timer.h"
#include "main_user.h"
#include "FOC_run.h"
#include "SEGGER_RTT_Port.h"

#include "foc_pid.h"

extern  logic          logicContr;
 
void knob_control(void)
{
   /*** 1、 根据电位器信号判断是否启动电机以及目标值***/
	if( ADCSampPare.RP_speed_Voltage<= 200) 
	{
	   // 启动电机控制需要初始化参数，三个PI环路参数	
		 logicContr.drive_car=0;

	}
	else
	{
		logicContr.drive_car=1;
		
//		current_q_pid.ref = 0.3f;
//		current_d_pid.ref = 0.0f;
//		if(logicContr.Run_mode==5)  //只有电流内环
//    {

//			pi_iq.Ref = (int32_t)(ADCSampPare.RP_speed_Voltage >> 2);
//			if( pi_iq.Ref>=1000)
//			{
//				pi_iq.Ref=1000;
//			}				       
//		} 
//		else
//		{
//			 pi_spd.Ref = (int32_t)(ADCSampPare.RP_speed_Voltage >> 1);   // 电位器转速信号
//			 //限制最大2000RPM，由于霍尔角度估算还是其他问题，大于2000转时转速估计突然到上限值，表现为会发生电机抽动现象
//		   //一到2000多转就发生该情况
// 	     if( pi_spd.Ref>=2000)  
// 	        pi_spd.Ref=2000;
//		}
 
	}

	
	/*** 2、 状态机 根据电机状态执行电机启停 ***/
	 if ((logicContr.olddrive_car==0) &&( logicContr.drive_car==1 ))  //说明电机开始启动
	 {
		  
	 }
	 else if ((logicContr.olddrive_car==1) &&( logicContr.drive_car==1 ))  //说明电机已经启动
	 {	
		 if(logicContr.Start_order ==0){
		   StartPWM();		// start mcu pwm output.
		 }
		 logicContr.Start_order=1;
	 }
	 else          //说明电机已经停止
	 {
		 if(logicContr.Start_order !=0){
		   StopPWM();		// stop mcu pwm output.
		 }
		 logicContr.Start_order=0;
	 }
	 logicContr.olddrive_car=logicContr.drive_car;
 
}


void StopPWM(void)
{
	//extern TIM_HandleTypeDef htim1;
	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_4);
	HAL_ADCEx_InjectedStop_IT(&hadc1);  //停止FOC运行
	HAL_ADCEx_InjectedStop(&hadc2);	
	
	Param_deinit();
}

void StartPWM(void)
{
	Param_init();	
	
	//extern TIM_HandleTypeDef htim1;
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_4);
	HAL_ADCEx_InjectedStart_IT(&hadc1);  //开启FOC运行
	HAL_ADCEx_InjectedStart(&hadc2);
}


//===========================================================================
// No more.
//===========================================================================
